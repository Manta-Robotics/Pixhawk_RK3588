#include "manta/prediction.hpp"

namespace manta {
void IntermittentFollow::reset() {
    estimator.reset();
    last_throttle = last_steering = last_forward = 0;
    raw_recovery = target_id = admitted_drive = nullptr;
    raw_recovered_at = last_now = recovery_cycle_at = recovery_step_at = last_position_good_at =
        live_command_at = missing;
    recovery_good_ms = live_command_speed = 0;
    recovery_step_good = false;
}
Json IntermittentFollow::coast(double now, const Json &e, const Json &t,
                               FollowController &controller) {
    const auto n = [&](const char *k) { return number(config, k); };
    const auto &c = controller.config;
    if (control_mode != "relative_acro" || e.value("admissionVersion", 0) != 1 ||
        e.value("source", std::string()) != "predicted" || !admitted_drive.is_object() ||
        !fresh_time(number(e, "lastMeasuredAt"), now, n("measurement_gap_ms")) ||
        !fresh_time(number(admitted_drive, "at"), now, n("measurement_gap_ms")) ||
        !std::isfinite(number(e, "uncertaintyM")) ||
        !e.value("relativeBodyM", Json(nullptr)).is_object())
        return nullptr;
    const auto body = vector_value(e["relativeBodyM"]);
    const double range = body.norm(), bearing = body.angle(),
                 slow = number(c, "fieldOfViewHalfDeg") - 25,
                 stop = number(c, "fieldOfViewHalfDeg") - 10;
    if (std::abs(bearing) >= stop)
        return nullptr;
    for (const auto &s : t.value("uwb", Json::object()).value("sensors", Json::array()))
        if (fresh_time(number(s, "rawDistanceUpdatedAt", number(s, "receivedMeasurementAt")), now,
                       n("measurement_gap_ms")) &&
            std::isfinite(number(s, "rawDistanceM")) && number(s, "rawDistanceM") >= 0 &&
            number(s, "rawDistanceM") <= n("clearance_m"))
            return nullptr;
    const double own = std::hypot(number(t["velocity"], "vx"), number(t["velocity"], "vy")),
                 clearance = range - number(e, "uncertaintyM") -
                             std::max(n("clearance_m"), number(c, "safetyDistanceM")),
                 stopping = own * .6 + squared(own) / (2 * n("braking_deceleration_mps2"));
    if (!(clearance > stopping))
        return nullptr;
    const double alignment = clamp((stop - std::abs(bearing)) / (stop - slow), 0, 1),
                 speed = std::max(0.0, std::min(number(admitted_drive, "speed") * alignment,
                                                std::sqrt(2 * n("braking_deceleration_mps2") *
                                                          std::max(0.0, clearance - own * .6))));
    const double yaw =
        number(traditional_guidance(c, bearing, number(t["attitude"], "yawRateDegS", 0), speed),
               "desiredYawRateDegS");
    controller.throttle_command =
        speed / std::min(2.0, number(c, "maxSpeedMps")) * number(c, "relativeThrottleLimitPercent");
    return {{"safe", true},
            {"status", "predicting"},
            {"reason", "accepted_position_coast"},
            {"controlMode", "relative_acro"},
            {"requiredFlightMode", "ACRO"},
            {"commandFrame", "BODY_RELATIVE"},
            {"navigationAvailable", true},
            {"targetEstimate", e},
            {"relativeBodyM", vector_json(body)},
            {"bearingDeg", bearing},
            {"distanceM", range},
            {"forwardSpeedMps", speed},
            {"desiredSpeedMps", speed},
            {"yawRateDegS", yaw},
            {"yawRateRadS", yaw * radians},
            {"throttlePercent", controller.throttle_command},
            {"steeringDegrees",
             yaw / number(c, "maxYawRateDegS") * number(c, "relativeSteeringLimitDeg")},
            {"speedLimitMps", std::min(2.0, number(c, "maxSpeedMps"))},
            {"shortGapHold", true},
            {"coastMeasurementAgeMs", now - number(e, "lastMeasuredAt")},
            {"coastClearanceM", clearance},
            {"coastStoppingDistanceM", stopping},
            {"coastPreviousCommandMps", admitted_drive["speed"]}};
}
double IntermittentFollow::recovery_scale(double now, const Json &e) {
    const auto source = e.value("source", std::string());
    const bool good = e.value("usable", false) && e.value("measurementConfirmed", false) &&
                      (source == "measured" || source == "recovering");
    const double cycle = estimator.recovered_at;
    if (source == "recovering" && cycle != recovery_cycle_at) {
        if (!std::isfinite(recovery_cycle_at) ||
            !fresh_time(last_position_good_at, now, number(config, "measurement_gap_ms")))
            recovery_good_ms = 0;
        recovery_cycle_at = cycle;
        recovery_step_good = false;
    }
    if (good && std::isfinite(recovery_cycle_at) && recovery_step_good &&
        std::isfinite(recovery_step_at))
        recovery_good_ms = std::min(number(config, "recovery_blend_ms"),
                                    recovery_good_ms + std::max(0.0, now - recovery_step_at));
    if (good)
        last_position_good_at = now;
    recovery_step_good = good;
    recovery_step_at = now;
    return std::isfinite(recovery_cycle_at)
               ? clamp(recovery_good_ms / number(config, "recovery_blend_ms"), 0, 1)
               : missing;
}
Json IntermittentFollow::neutral(const std::string &reason, const Json &estimate, bool waiting) {
    last_throttle = last_steering = last_forward = 0;
    admitted_drive = nullptr;
    Json result = {{"safe", waiting},
                   {"status", waiting ? "waiting_target" : "stopped"},
                   {"reason", reason},
                   {"pausePropulsion", waiting},
                   {"targetEstimate", estimate},
                   {"relativeBodyM", estimate.is_object()
                                         ? estimate.value("relativeBodyM", Json{{"x", 0}, {"y", 0}})
                                         : Json{{"x", 0}, {"y", 0}}},
                   {"controlMode", control_mode},
                   {"requiredFlightMode", control_mode == "relative_acro" ? "ACRO" : "MANUAL"},
                   {"commandFrame", "BODY_RELATIVE"},
                   {"throttlePercent", 0},
                   {"steeringDegrees", 0},
                   {"forwardSpeedMps", 0},
                   {"yawRateDegS", 0},
                   {"yawRateRadS", 0},
                   {"desiredSpeedMps", 0},
                   {"velocityNed", {{"x", 0}, {"y", 0}}}};
    if (estimator.calibration.value("execution_mode", std::string()) == "relative_imu")
        result.update({{"relativeImuControl", true}, {"absoluteSpeedObservable", false}});
    return result;
}
Json IntermittentFollow::update(FollowController &controller, double now, const Json &t,
                                bool connected, const std::string &requested) {
    const auto n = [&](const char *key) { return number(config, key); };
    const auto &c = controller.config;
    control_mode = c["executionMode"] == "acro" || requested == "relative_acro" ? "relative_acro"
                                                                                : "relative_manual";
    if (t.value("uwb", Json::object()).value("clearanceStop", false)) {
        reset();
        controller.reset();
        return neutral("raw_distance_clearance", nullptr, true);
    }
    if (!config.value("enabled", false))
        return controller.update(now, t, connected, requested);
    if (std::isfinite(last_now) && now < last_now)
        return neutral("prediction_clock_reset", nullptr, false);
    last_now = now;
    const auto u = t.value("uwb", Json::object()), a = t.value("attitude", Json::object()),
               acro = t.value("acroControl", Json::object()), ekf = t.value("ekf", Json::object()),
               pos = t.value("position", Json::object()), gps = t.value("gps", Json::object()),
               pilot = t.value("pilotSteering", Json::object());
    auto flight_mode = t.value("flightMode", std::string());
    for (auto &ch : flight_mode)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    std::string fault;
    if (!connected)
        fault = "pixhawk_offline";
    else if (!t.value("armed", false))
        fault = "armed_required";
    else if (flight_mode != (control_mode == "relative_acro" ? "ACRO" : "MANUAL"))
        fault = control_mode == "relative_acro" ? "acro_required" : "manual_required";
    else if (control_mode == "relative_acro" &&
             (!acro.value("ready", false) ||
              !fresh_time(number(acro, "updatedAt"), now, n("pose_timeout_ms")) ||
              !ekf.value("healthy", false) ||
              !fresh_time(number(ekf, "updatedAt"), now, n("pose_timeout_ms")) ||
              !fresh_time(number(pos, "updatedAt"), now, n("pose_timeout_ms")) ||
              number(gps, "fixType", 0) < 2))
        fault = "acro_velocity_unavailable";
    else if (!pilot.value("ready", false) || !std::isfinite(number(pilot, "type")) ||
             number(pilot, "type") != std::trunc(number(pilot, "type")) ||
             number(pilot, "type") < 0 || number(pilot, "type") > 3)
        fault = "pilot_steering_not_ready";
    else if (!std::isfinite(number(a, "yaw")))
        fault = "imu_invalid";
    else if (!fresh_time(number(a, "updatedAt"), now, n("pose_timeout_ms")))
        fault = "imu_stale";
    else if (u.value("rejectionReason", Json(nullptr)) == "target_identity_changed")
        fault = "target_identity_changed";
    if (!fault.empty())
        return neutral(fault, nullptr, false);
    Json calibration = {{"bearing_sign", c["bearingSign"]},
                        {"bearing_scale", c["bearingScale"]},
                        {"bearing_offset_deg", c["bearingOffsetDeg"]}};
    if (c["executionMode"] == "relative_imu")
        calibration["execution_mode"] = "relative_imu";
    bool changed = false;
    for (const auto &[key, value] : calibration.items())
        if (estimator.calibration.value(key, Json(nullptr)) != value)
            changed = true;
    if (changed) {
        reset();
        estimator.calibration = calibration;
    }
    const bool was_lost = std::isfinite(estimator.loss_at);
    const auto estimate = estimator.update(now, t);
    if (estimate.value("terminal", false))
        return neutral(estimate.value("reason", std::string()), estimate, false);
    const double position_scale = recovery_scale(now, estimate);
    const bool raw_fresh =
        u.value("online", false) && u.value("fresh", false) &&
        std::isfinite(number(u, "distanceM")) && number(u, "distanceM") > 0 &&
        std::isfinite(number(u, "azimuthDeg")) &&
        fresh_time(measurement_time(u), now, number(c, "uwbTimeoutMs"));
    const auto id = raw_fresh ? target_identity(u) : Json(nullptr);
    if (!id.is_null() && !target_id.is_null() && id != target_id)
        return neutral("target_identity_changed", estimate, false);
    if (!id.is_null())
        target_id = id;
    if (estimate.value("reason", std::string()) == "prediction_pose_unavailable" && raw_fresh) {
        const double bearing =
            (number(u, "azimuthDeg") * number(c, "bearingSign") * number(c, "bearingScale") +
             number(c, "bearingOffsetDeg")) *
            radians;
        const Vec body = {number(u, "distanceM") * std::cos(bearing),
                          number(u, "distanceM") * std::sin(bearing)};
        if (was_lost) {
            const auto previous = raw_recovery;
            const auto token = measurement_key(u);
            const double at = measurement_time(u);
            if (!previous.is_object() ||
                (at > number(previous, "at") && previous["token"] != token)) {
                const double dt = previous.is_object() ? (at - number(previous, "at")) / 1000 : 0;
                const bool consistent =
                    previous.is_object() && dt > 0 && dt * 1000 <= n("measurement_gap_ms") &&
                    (body - vector_value(previous["body"])).norm() <=
                        n("innovation_gate_m") + 2 * n("max_target_speed_mps") * dt;
                raw_recovery = {{"at", at},
                                {"token", token},
                                {"body", vector_json(body)},
                                {"samples", consistent ? number(previous, "samples") + 1 : 1}};
            }
            if (number(raw_recovery, "samples") < n("min_samples")) {
                auto e = estimate;
                e["recoverySamples"] = raw_recovery["samples"];
                return neutral("prediction_reacquiring", e, true);
            }
            estimator.reset();
            raw_recovery = nullptr;
            raw_recovered_at = now;
        } else
            estimator.loss_at = missing;
        auto result = controller.update(now, t, connected, requested);
        const bool recovering =
            std::isfinite(raw_recovered_at) && now - raw_recovered_at < n("recovery_blend_ms");
        if (!recovering) {
            result["targetEstimate"] = estimate;
            return result;
        }
        const double scale = clamp((now - raw_recovered_at) / n("recovery_blend_ms"), 0, 1);
        auto e = estimate;
        e.update({{"source", "recovering"},
                  {"lastMeasuredAt", nullable(measurement_time(u))},
                  {"relativeBodyM", vector_json(body)}});
        result.update({{"status", "recovering"},
                       {"reason", "prediction_recovering"},
                       {"targetEstimate", e},
                       {"intermittentThrottleCap", n("max_throttle_percent") * scale},
                       {"intermittentSteeringCap", n("max_steering_deg") * scale},
                       {"throttlePercent", clamp(number(result, "throttlePercent"), 0,
                                                 n("max_throttle_percent") * scale)}});
        return result;
    }
    const auto source = estimate.value("source", std::string()),
               reason = estimate.value("reason", std::string());
    const bool measured =
        (source == "measured" && estimate.value("usable", false)) ||
        (source == "recovering" && estimate.value("usable", false) && position_scale == 1) ||
        (!was_lost && reason == "prediction_pose_unavailable" && u.value("online", false) &&
         u.value("fresh", false) &&
         fresh_time(measurement_time(u), now, number(c, "uwbTimeoutMs")));
    if (measured && source == "unavailable")
        estimator.loss_at = missing;
    const bool predicted = source == "predicted";
    if (predicted && reject_nmpc && control_mode == "relative_acro") {
        controller.reset();
        auto out = neutral("reject_nmpc_planning", estimate, true);
        out["nmpcRequested"] = true;
        return out;
    }
    const auto coast_result = predicted ? coast(now, estimate, t, controller) : Json(nullptr);
    if (coast_result.is_object())
        return coast_result;
    bool allowed = estimate.value("usable", false);
    std::string pause_reason = reason;
    if (predicted) {
        const double speed = number(estimate, estimate.value("referenceFrame", std::string()) ==
                                                      "yaw_stabilized_relative"
                                                  ? "relativeClosingSpeedMps"
                                                  : "boatSpeedMps"),
                     stop = squared(speed) / (2 * n("braking_deceleration_mps2")),
                     clearance = vector_value(estimate["relativeBodyM"]).norm() -
                                 number(estimate, "uncertaintyM");
        if (speed > n("max_speed_mps")) {
            allowed = false;
            pause_reason = "prediction_speed_limit";
        }
        if (clearance <= n("clearance_m") + stop) {
            allowed = false;
            pause_reason = "prediction_clearance";
        }
        if (estimate.value("reacquiring", false)) {
            allowed = false;
            pause_reason = "prediction_reacquiring";
        }
        for (const auto &s : u.value("sensors", Json::array()))
            if (s.value("measurementStatus", std::string()) == "suspect" &&
                fresh_time(number(s, "rawDistanceUpdatedAt"), now, n("measurement_gap_ms")) &&
                std::isfinite(number(s, "rawDistanceM")) && number(s, "rawDistanceM") >= 0 &&
                number(s, "rawDistanceM") <= n("clearance_m")) {
                allowed = false;
                pause_reason = "prediction_clearance";
            }
    }
    if (!measured && !allowed) {
        if (c["executionMode"] == "relative_imu")
            controller.relative.pause();
        else
            controller.reset();
        return neutral(pause_reason.empty() ? "prediction_limit" : pause_reason, estimate, true);
    }
    const double measurement_clearance = estimate.value("measurementConfirmed", false)
                                             ? vector_value(estimate["relativeBodyM"]).norm() -
                                                   number(estimate, "uncertaintyM") -
                                                   number(c, "safetyDistanceM")
                                             : missing;
    if (std::isfinite(measurement_clearance) && measurement_clearance <= 0) {
        controller.reset();
        return neutral("measurement_clearance", estimate, true);
    }
    const bool admitted_acro = estimate.value("admissionVersion", 0) == 1 &&
                               estimate.value("measurementConfirmed", false) &&
                               control_mode == "relative_acro";
    if (admitted_acro && !std::isfinite(controller.previous_at)) {
        const double psi = number(a, "yaw") * radians,
                     own = std::max(0.0, number(t["velocity"], "vx") * std::cos(psi) +
                                             number(t["velocity"], "vy") * std::sin(psi));
        controller.throttle_command = std::min(own, number(c, "maxSpeedMps")) /
                                      number(c, "maxSpeedMps") *
                                      number(c, "relativeThrottleLimitPercent");
    }
    auto result = controller.update(now, t, connected, requested,
                                    measured ? Json(nullptr) : estimate, estimate);
    if (!result.value("safe", false)) {
        result["targetEstimate"] = estimate;
        return result;
    }
    if (std::isfinite(measurement_clearance)) {
        const double cap = std::sqrt(2 * n("braking_deceleration_mps2") * measurement_clearance);
        result["measurementClearanceM"] = measurement_clearance;
        result["measurementSpeedLimitMps"] = cap;
        result["forwardSpeedMps"] = std::min(number(result, "forwardSpeedMps", 0), cap);
        result["desiredSpeedMps"] =
            std::min(number(result, "desiredSpeedMps", number(result, "forwardSpeedMps", 0)), cap);
    }
    if (result.value("reason", std::string()) == "holding_distance") {
        result["targetEstimate"] = estimate;
        return result;
    }
    if (admitted_acro) {
        const double yaw = number(a, "yaw") * radians,
                     own = number(t["velocity"], "vx") * std::cos(yaw) +
                           number(t["velocity"], "vy") * std::sin(yaw),
                     dt = std::isfinite(live_command_at) ? clamp((now - live_command_at) / 1000, 0,
                                                                 n("measurement_gap_ms") / 1000)
                                                         : 0;
        const bool recent =
            std::isfinite(live_command_at) && now - live_command_at <= n("measurement_gap_ms");
        const double start = recent ? live_command_speed : std::max(0.0, own);
        const double speed_limit =
                         std::min(number(c, "maxSpeedMps"), c.value("commissioningMode", true)
                                                                ? number(c, "commissioningSpeedMps")
                                                                : number(c, "maxSpeedMps")),
                     acceleration = speed_limit * number(c, "throttleAccelPercentS") /
                                    number(c, "relativeThrottleLimitPercent"),
                     cap = start + acceleration * dt;
        result["forwardSpeedMps"] = std::min(number(result, "forwardSpeedMps", 0), cap);
        result["desiredSpeedMps"] =
            std::min(number(result, "desiredSpeedMps", number(result, "forwardSpeedMps", 0)), cap);
        live_command_speed = number(result, "forwardSpeedMps");
        live_command_at = now;
        result.update({{"targetEstimate", estimate},
                       {"liveRecoverySpeedCapMps", cap},
                       {"liveRecoveryReferenceMps", start},
                       {"liveRecoveryStepS", dt},
                       {"liveRecoveryAccelerationMps2", acceleration}});
        if (source == "recovering") {
            result["status"] = "recovering";
            result["reason"] = "measurement_recovering";
        }
        return result;
    }
    if (measured) {
        if ((std::isfinite(position_scale) && position_scale < 1) ||
            (std::isfinite(raw_recovered_at) && now - raw_recovered_at < n("recovery_blend_ms"))) {
            const double scale =
                std::isfinite(position_scale)
                    ? position_scale
                    : clamp((now - raw_recovered_at) / n("recovery_blend_ms"), 0, 1);
            auto e = estimate;
            e["source"] = "recovering";
            result.update({{"status", "recovering"},
                           {"reason", "prediction_recovering"},
                           {"targetEstimate", e},
                           {"intermittentThrottleCap", n("max_throttle_percent") * scale},
                           {"intermittentSteeringCap", control_mode == "relative_acro"
                                                           ? number(c, "relativeSteeringLimitDeg")
                                                           : n("max_steering_deg") * scale},
                           {"throttlePercent", clamp(number(result, "throttlePercent"), 0,
                                                     n("max_throttle_percent") * scale)}});
            return result;
        }
        result["targetEstimate"] = estimate;
        return result;
    }
    const double decay =
                     predicted
                         ? std::max(0.0, 1 - number(estimate, "predictionAgeMs") / n("horizon_ms"))
                     : std::isfinite(position_scale) ? position_scale
                                                     : number(estimate, "recoveryScale"),
                 throttle_cap = n("max_throttle_percent") * decay;
    const bool turn = control_mode == "relative_acro" && !predicted && source == "recovering" &&
                      estimate.value("usable", false);
    const double steering_cap =
        turn ? number(c, "relativeSteeringLimitDeg") : n("max_steering_deg") * decay;
    result.update(
        {{"status", predicted ? "predicting" : "recovering"},
         {"reason", predicted ? "prediction_tracking" : "prediction_recovering"},
         {"targetEstimate", estimate},
         {"intermittentThrottleCap", throttle_cap},
         {"intermittentSteeringCap", steering_cap},
         {"throttlePercent", clamp(number(result, "throttlePercent"), 0, throttle_cap)},
         {"steeringDegrees", clamp(number(result, "steeringDegrees"), -steering_cap, steering_cap)},
         {"forwardSpeedMps",
          clamp(number(result, "forwardSpeedMps", 0), 0, n("max_speed_mps") * decay)},
         {"desiredSpeedMps",
          clamp(number(result, "desiredSpeedMps", number(result, "forwardSpeedMps", 0)), 0,
                n("max_speed_mps") * decay)}});
    return result;
}
Json IntermittentFollow::govern_acro(const Json &result, double speed_limit, double yaw_limit,
                                     double steering_span, double throttle_span) {
    if (!result.value("safe", false) || result.value("pausePropulsion", false)) {
        last_forward = 0;
        return {{"forwardSpeedMps", 0}, {"yawRateDegS", 0}};
    }
    double speed = clamp(number(result, "forwardSpeedMps", 0), -.2, std::min(2.0, speed_limit)),
           yaw = clamp(number(result, "yawRateDegS", 0), -yaw_limit, yaw_limit);
    const auto estimate = result.value("targetEstimate", Json::object());
    if (result.contains("intermittentThrottleCap")) {
        double cap =
            std::min(number(config, "max_speed_mps"),
                     speed_limit * std::max(0.0, number(result, "intermittentThrottleCap")) /
                         std::max(throttle_span, 1.0));
        if (estimate.is_object() && estimate.value("source", std::string()) == "predicted")
            cap = std::min(cap, std::max(0.0, last_forward));
        speed = clamp(speed, 0, cap);
        const double turn_cap = yaw_limit *
                                std::max(0.0, number(result, "intermittentSteeringCap", 0)) /
                                std::max(steering_span, 1.0);
        yaw = clamp(yaw, -turn_cap, turn_cap);
    }
    last_forward = speed;
    if (estimate.is_object() && estimate.value("admissionVersion", 0) == 1 &&
        estimate.value("measurementConfirmed", false))
        admitted_drive = {{"at", estimate["evaluatedAt"]}, {"speed", speed}};
    else if (result.value("shortGapHold", false) && admitted_drive.is_object())
        admitted_drive["speed"] = std::min(number(admitted_drive, "speed"), speed);
    return {{"forwardSpeedMps", speed}, {"yawRateDegS", yaw}};
}
Json IntermittentFollow::govern_drive(const Json &output, const Json &result) {
    auto out = output;
    if (result.value("pausePropulsion", false) || !result.value("safe", false)) {
        last_throttle = last_steering = 0;
        out.update({{"throttle", 0}, {"steering", 0}});
        return out;
    }
    double throttle = number(output, "throttle"), steering = number(output, "steering");
    if (result.contains("intermittentThrottleCap")) {
        const auto estimate = result.value("targetEstimate", Json::object());
        const double cap =
            estimate.is_object() && estimate.value("source", std::string()) == "predicted"
                ? std::min(number(result, "intermittentThrottleCap"), std::max(0.0, last_throttle))
                : number(result, "intermittentThrottleCap");
        throttle = clamp(throttle, 0, cap);
        const double turn_cap = std::min(number(result, "intermittentSteeringCap"), throttle * .45);
        steering = clamp(steering, -turn_cap, turn_cap);
    }
    last_throttle = throttle;
    last_steering = steering;
    out.update({{"throttle", throttle}, {"steering", steering}});
    return out;
}
} // namespace manta
