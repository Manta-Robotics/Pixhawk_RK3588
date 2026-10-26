#include "manta/control_runtime.hpp"
#include <cctype>

namespace manta {
namespace {
std::string upper(std::string text) {
    for (auto &ch : text)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    return text;
}
Json follow_config(const Json &config) {
    auto raw = config.value("uwb_follow", Json::object());
    const auto beacon = config.value("gimbal", Json::object()).value("beacon", Json::object());
    for (const auto &[src, dst] : std::array<std::pair<const char *, const char *>, 3>{
             {{"yaw_source_sign", "bearing_sign"},
              {"yaw_source_scale", "bearing_scale"},
              {"yaw_offset_deg", "bearing_offset_deg"}}})
        if (beacon.contains(src) && !beacon[src].is_null())
            raw[dst] = beacon[src];
    if (!raw.contains("neural_model_path"))
        raw["neural_model_path"] = "models/smart_usv_uwb_policy.json";
    return raw;
}
} // namespace
ControlRuntime::ControlRuntime(Json config, MavlinkCore &core, MavlinkCore::Event event)
    : config(std::move(config)), raw(follow_config(this->config)), core(core),
      event(std::move(event)) {
    auto traditional_config = raw;
    traditional_config["manual_propulsion"] = "single_side";
    traditional = std::make_unique<FollowController>(traditional_config);
    try {
        neural = std::make_unique<NeuralFollow>(raw);
    } catch (const std::exception &e) {
        neural_error = e.what();
    }
    if (raw.value("tracking_method", std::string()) == "neural" && neural)
        method = "neural";
    intermittent =
        std::make_unique<IntermittentFollow>(raw.value("prediction", Json::object()), raw);
    resume = std::make_unique<AutoResume>(raw);
    drift = std::make_unique<DriftHold>(raw);
    const auto &c = this->config;
    throttle_limit = clamp(number(c, "rover_manual_throttle_limit_percent", 100), 0, 100);
    throttle_span = std::max(std::abs(number(c, "rover_throttle_min", -100)),
                             std::abs(number(c, "rover_throttle_max", 100)));
    steering_span = std::max(std::abs(number(c, "rover_steering_min", -45)),
                             std::abs(number(c, "rover_steering_max", 45)));
    steering_limit =
        steering_span * clamp(number(c, "rover_manual_steering_limit_percent", 100), 0, 100) / 100;
    Json mc = {{"throttle_limit", throttle_limit},
               {"steering_limit", steering_limit},
               {"throttle_span", throttle_span},
               {"steering_span", steering_span},
               {"maximum_dt_s", 2 / clamp(number(c, "rover_manual_slew_rate_hz", 20), 10, 50)},
               {"throttle_deadband", number(c, "rover_manual_throttle_deadband_percent", 4)},
               {"steering_deadband", number(c, "rover_manual_steering_deadband_deg", 2)},
               {"input_expo", number(c, "rover_manual_input_expo", 1.4)}};
    for (const auto &[key, dst] : std::array<std::pair<const char *, const char *>, 4>{
             {{"rover_manual_throttle_accel_percent_s", "throttle_accel_percent_s"},
              {"rover_manual_throttle_decel_percent_s", "throttle_decel_percent_s"},
              {"rover_manual_reverse_neutral_ms", "reverse_neutral_ms"},
              {"rover_manual_side_switch_neutral_ms", "side_switch_neutral_ms"}}})
        if (c.contains(key))
            mc[dst] = c[key];
    manual_motor = std::make_unique<MotorLimiter>(mc);
    neural_motor = std::make_unique<MotorLimiter>(mc);
    mc.update({{"throttle_limit", std::min(throttle_limit, number(traditional->config,
                                                                  "relativeThrottleLimitPercent"))},
               {"steering_limit",
                std::min(steering_limit, number(traditional->config, "relativeSteeringLimitDeg"))},
               {"throttle_accel_percent_s", traditional->config["throttleAccelPercentS"]},
               {"throttle_decel_percent_s", traditional->config["throttleDecelPercentS"]},
               {"maximum_dt_s", 2 / clamp(number(raw, "control_rate_hz", 10), 2, 20)}});
    traditional_motor = std::make_unique<MotorLimiter>(mc, true);
    state = idle("not_started", "idle");
    state.update({{"active", false}, {"trackingMethod", method}, {"updatedAt", 0}});
    state.update(resume->snapshot());
    rover = normalized(0, 0);
}
FollowController &ControlRuntime::controller() {
    return method == "neural" ? static_cast<FollowController &>(*neural) : *traditional;
}
void ControlRuntime::emit(const std::string &type, const Json &payload) {
    event({{"type", type}, {"payload", payload}, {"timestamp", wall / 1000}});
}
Json ControlRuntime::send(const std::string &name, const Json &params) {
    return core.command({{"command", name}, {"params", params}}, wall, mono);
}
Json ControlRuntime::snapshot() const {
    return {{"ok", true},
            {"state", state},
            {"roverControl", rover},
            {"transitionId", transition},
            {"busy", !phase.empty()},
            {"trackingMethod", method},
            {"neuralAvailable", bool(neural)},
            {"neuralLoadError", neural_error.empty() ? Json(nullptr) : Json(neural_error)},
            {"owner", "cpp"}};
}
void ControlRuntime::guard_uwb() {
    auto u = t.value("uwb", Json::object());
    // Preserve the original shared beacon/follow angular slew, on a service
    // clock that remains available when no browser or Web process is running.
    const double az = number(u, "azimuthDeg", 0), el = number(u, "elevationDeg", 0);
    if (std::isfinite(number(u, "measurementSampleAt"))) {
        // Already filtered in source time and rotated to the current IMU
        // heading. Re-slewing it here would introduce another angular lag and
        // detach the bearing from its declared heading/reference frame.
        source_guard = nullptr;
        guarded_uwb = u;
    } else if (!u.value("online", false) || !u.value("fresh", false)) {
        source_guard = nullptr;
        guarded_uwb = u;
    } else if (mono >= next_guard || !guarded_uwb.is_object() || !source_guard.is_object()) {
        if (!source_guard.is_object() || wall - number(source_guard, "updatedAt") > 700)
            source_guard = {{"azimuthDeg", az}, {"elevationDeg", el}, {"updatedAt", wall}};
        else {
            const double dt = clamp((wall - number(source_guard, "updatedAt")) / 1000, .001, .1),
                         step = std::max(.8, 90 * dt);
            source_guard = {
                {"azimuthDeg",
                 wrap(number(source_guard, "azimuthDeg") +
                      clamp(wrap(az - number(source_guard, "azimuthDeg")), -step, step))},
                {"elevationDeg", number(source_guard, "elevationDeg") +
                                     clamp(el - number(source_guard, "elevationDeg"), -step, step)},
                {"updatedAt", wall}};
        }
        guarded_uwb = u;
        guarded_uwb["azimuthDeg"] = source_guard["azimuthDeg"];
        guarded_uwb["elevationDeg"] = source_guard["elevationDeg"];
    } else {
        guarded_uwb = u;
        guarded_uwb["azimuthDeg"] = source_guard["azimuthDeg"];
        guarded_uwb["elevationDeg"] = source_guard["elevationDeg"];
    }
    if (mono >= next_guard) {
        next_guard = mono + 50;
        emit("control_uwb", guarded_uwb);
    }
    t["uwb"] = guarded_uwb;
}
Json ControlRuntime::idle(const std::string &reason, const std::string &status,
                          const Json &pre) const {
    auto r = zero_control(reason, {{"controlMode", pre.value("controlMode", control_mode)}});
    r["navigationAvailable"] = pre.value("navigationAvailable", false);
    r["navigationReason"] = pre.value("navigationReason", Json(nullptr));
    if (pre.contains("requiredFlightMode"))
        r["requiredFlightMode"] = pre["requiredFlightMode"];
    if (!status.empty())
        r["status"] = status;
    return r;
}
void ControlRuntime::publish(Json result, bool running) {
    active = running;
    result.update(resume->snapshot());
    result.update({{"trackingMethod", method},
                   {"active", active},
                   {"updatedAt", wall},
                   {"transitionBusy", !phase.empty()},
                   {"transitionId", transition}});
    if (!result.contains("status") || !result["status"].is_string() || result["status"] == "")
        result["status"] = active && result.value("safe", false) ? "tracking"
                           : result.value("reason", std::string()) == "safety_distance"
                               ? "safety_stop"
                               : "stopped";
    state = std::move(result);
    emit("uwb_follow", state);
    last_publish = mono;
}
Json ControlRuntime::normalized(double throttle, double steering) const {
    const double tr = throttle, sr = steering, center = number(config, "default_motor_pwm", 1500),
                 maximum = number(config, "max_motor_pwm", 2000),
                 minimum = number(config, "min_motor_pwm", 1000);
    throttle =
        clamp(throttle, std::max(number(config, "rover_throttle_min", -100), -throttle_limit),
              std::min(number(config, "rover_throttle_max", 100), throttle_limit));
    steering = clamp(steering, std::max(number(config, "rover_steering_min", -45), -steering_limit),
                     std::min(number(config, "rover_steering_max", 45), steering_limit));
    const double ts = (maximum - center) / std::max(1.0, throttle_span),
                 ss = (maximum - center) / std::max(1.0, steering_span);
    return {{"throttle", throttle},
            {"steering", steering},
            {"leftPwm",
             std::floor(clamp(center + throttle * ts + steering * ss, minimum, maximum) + .5)},
            {"rightPwm",
             std::floor(clamp(center + throttle * ts - steering * ss, minimum, maximum) + .5)},
            {"clamped", tr != throttle || sr != steering}};
}
Json ControlRuntime::drive(double throttle, double steering, const std::string &label) {
    auto output = normalized(throttle, steering);
    const auto sent = send("ROVER_DRIVE", {{"throttle", output["throttle"]},
                                           {"steering", output["steering"]},
                                           {"motorTargetSemantics", true}});
    output["safe"] = sent.value("ok", false);
    if (!sent.value("ok", false))
        output["reason"] = sent.value("error", std::string("drive_failed"));
    rover = {{"throttle", output["throttle"]},
             {"steering", output["steering"]},
             {"leftPwm", output["leftPwm"]},
             {"rightPwm", output["rightPwm"]}};
    auto event = rover;
    event["source"] = label;
    event["timestamp"] = wall;
    emit("rover_control", event);
    return output;
}
void ControlRuntime::neutral(const std::string &reason) {
    manual_motor->reset(mono);
    neural_motor->reset(mono);
    traditional_motor->reset(mono);
    send("CONTROL_NEUTRAL", {{"reason", reason}});
    rover = normalized(0, 0);
    auto out = rover;
    out["source"] = "SAFETY";
    out["timestamp"] = wall;
    emit("rover_control", out);
}
void ControlRuntime::disarm(const std::string &reason) {
    send("DISARM", {{"reason", reason}});
    send("SET_MODE", {{"mode", "MANUAL"}, {"reason", reason}});
}
void ControlRuntime::reset_hold() {
    holding_at = manual_confirmed = mode_requested = resume_started = resume_requested = 0;
}
void ControlRuntime::stop(const std::string &reason, bool disarm_vehicle, bool cancel_intent) {
    ++transition;
    phase.clear();
    if (cancel_intent)
        resume->cancel();
    else
        resume->stopped(reason);
    drift->reset();
    intermittent->reset();
    controller().reset();
    reset_hold();
    neutral(reason);
    if (disarm_vehicle)
        disarm(reason);
    publish(idle(reason), false);
    control_mode = "auto";
}
Json ControlRuntime::select(const std::string &requested) {
    if (requested != "traditional" && requested != "neural")
        return {{"ok", false}, {"error", "tracking_method_must_be_traditional_or_neural"}};
    if (requested == "neural" && !neural)
        return {{"ok", false}, {"error", "neural_policy_unavailable:" + neural_error}};
    if (active || !phase.empty() || resume->pending)
        return {{"ok", false}, {"error", "stop_tracking_before_switching_method"}};
    intermittent->reset();
    neutral("tracking_method_switch");
    resume->cancel();
    traditional->reset();
    if (neural)
        neural->reset();
    method = requested;
    publish(idle("method_selected"), false);
    return {{"ok", true}, {"method", method}};
}
Json ControlRuntime::start(bool automatic_start) {
    if (!raw.value("enabled", false))
        return {{"ok", false}, {"error", "UWB follow is disabled in config"}};
    if (!phase.empty())
        return {{"ok", false}, {"error", "start_in_progress"}};
    if (active)
        return {{"ok", true}, {"alreadyActive", true}};
    if (!automatic_start && resume->pending)
        return {{"ok", true}, {"searching", true}, {"alreadySearching", true}};
    automatic = automatic_start;
    if (!automatic)
        resume->cancel();
    intermittent->reset();
    controller().reset();
    drift->reset();
    reset_hold();
    const auto pre = controller().preflight(wall, t, core.connected(), "auto");
    if (!pre.value("safe", false)) {
        const auto reason = pre.value("reason", std::string("start_failed"));
        neutral(reason);
        disarm(reason);
        if (!automatic && reason == "uwb_stale") {
            ++transition;
            resume->search();
            control_mode = "auto";
            auto r = idle("awaiting_trusted_uwb", "awaiting_signal");
            r["waitingReason"] = "uwb_stale";
            publish(r, false);
            return {{"ok", true}, {"searching", true}};
        }
        if (automatic)
            resume->attempt_failed(reason);
        publish(pre, false);
        return {{"ok", false}, {"error", reason}};
    }
    control_mode = pre.at("controlMode").get<std::string>();
    required_mode = pre.at("requiredFlightMode").get<std::string>();
    ++transition;
    neutral("start_sequence");
    if (t.value("armed", false)) {
        phase = "disarm";
        send("DISARM");
        publish(idle("disarming", "disarming", pre), false);
    } else {
        phase = "mode";
        send("SET_MODE", {{"mode", required_mode}});
        publish(idle("switching_mode", "switching_mode", pre), false);
    }
    deadline =
        mono + (automatic ? number(raw.value("prediction", Json::object()), "wait_timeout_ms", 5000)
                          : 5000);
    return {{"ok", true}, {"pending", true}, {"transitionId", transition}};
}
void ControlRuntime::advance_start() {
    auto fail = [&](const std::string &reason) {
        const bool was_automatic = automatic;
        phase.clear();
        ++transition;
        controller().reset();
        neutral(reason);
        disarm(reason);
        if (was_automatic)
            resume->attempt_failed(reason);
        publish(idle(reason), false);
        control_mode = "auto";
    };
    if (!core.connected()) {
        fail("pixhawk_offline");
        return;
    }
    if (mono >= deadline) {
        const auto pre = controller().preflight(wall, t, core.connected(), control_mode);
        fail(automatic && pre.value("reason", std::string()) == "uwb_stale"
                 ? "auto_resume_signal_timeout"
             : phase == "mode" ? std::string(required_mode == "MANUAL" ? "manual_mode_timeout"
                                             : required_mode == "ACRO" ? "acro_mode_timeout"
                                                                       : "guided_mode_timeout")
                               : phase + "_timeout");
        return;
    }
    const auto pre = controller().preflight(wall, t, core.connected(), control_mode);
    if (phase != "disarm") {
        if (automatic) {
            const auto reason = resume->transition_failure(wall, t, pre);
            if (reason.is_string()) {
                fail(reason.get<std::string>());
                return;
            }
            neutral("auto_resume_waiting_fresh_measurement");
            if (!pre.value("safe", false))
                return;
        } else if (!pre.value("safe", false)) {
            fail(pre.value("reason", std::string("start_failed")));
            return;
        }
    }
    if (phase == "disarm" && !t.value("armed", false)) {
        phase = "mode";
        if (!automatic)
            deadline = mono + 5000;
        send("SET_MODE", {{"mode", required_mode}});
        publish(idle("switching_mode", "switching_mode", pre), false);
        return;
    }
    if (phase == "mode" && upper(t.value("flightMode", std::string())) == required_mode) {
        phase = "arming";
        if (!automatic)
            deadline = mono + 8000;
        send("ARM");
        publish(idle("arming", "arming", pre), false);
        return;
    }
    if (phase == "arming" && t.value("armed", false)) {
        if (automatic) {
            resume->clear_window();
            phase = "recovered_uwb";
            publish(idle("confirming_recovered_uwb", "confirming_signal"), false);
            return;
        }
    } else if (phase == "recovered_uwb") {
        if (!resume->observe(wall, t, pre, true))
            return;
    } else
        return;
    controller().reset();
    auto result = controller().update(wall, t, core.connected(), control_mode);
    if (!result.value("safe", false)) {
        fail(result.value("reason", std::string("start_failed")));
        return;
    }
    resume->started(t, wall, automatic);
    result = resume->ramp(result, wall);
    if (control_mode == "relative_acro" || result.value("relativeImuControl", false)) {
        auto zero = result;
        zero.update({{"forwardSpeedMps", 0},
                     {"yawRateDegS", 0},
                     {"throttlePercent", 0},
                     {"steeringDegrees", 0}});
        relative_drive(zero);
    }
    phase.clear();
    publish(result, true);
}
Json ControlRuntime::relative_drive(Json &r) {
    if (r.value("relativeImuControl", false)) {
        const auto requested =
                       intermittent->govern_drive({{"throttle", number(r, "throttlePercent", 0)},
                                                   {"steering", number(r, "steeringDegrees", 0)}},
                                                  r),
                   bounded = normalized(number(requested, "throttle", 0),
                                        number(requested, "steering", 0));
        const auto allocation = allocate_forward_drive(number(bounded, "throttle"),
            number(bounded, "steering"), throttle_span, steering_span,
            std::min(throttle_span, throttle_limit));
        const double throttle = number(allocation, "throttle"),
                     limit = throttle * std::max(1.0, steering_span) /
                             std::max(1.0, throttle_span) * .8;
        auto out =
            drive(throttle, number(allocation, "steering"), "UWB_IMU_RELATIVE");
        r["driveAllocation"] = allocation;
        controller().relative.track_applied(number(out, "throttle"), number(out, "steering"));
        r["throttlePercent"] = out["throttle"];
        r["steeringDegrees"] = out["steering"];
        out.update({{"sameDirectionSteeringLimit", limit},
                    {"sameDirectionLimited", std::abs(number(bounded, "steering")) > limit + 1e-9},
                    {"propulsion",
                     r.value("planner", std::string()) == "nmpc" ? "optional_gps_relative_nmpc"
                     : r.value("gpsSpeedControlActive", false)   ? "uwb_gps_speed"
                                                                 : "relative_rate_throttle"},
                    {"absoluteSpeedObservable", r.value("gpsSpeedControlActive", false)}});
        return out;
    }
    if (control_mode == "relative_acro") {
        auto input = r;
        input["pausePropulsion"] = r.value("pausePropulsion", false) ||
                                   r.value("reason", std::string()) == "holding_distance";
        const auto &c = controller().config;
        const auto command = intermittent->govern_acro(
            input, std::min(2.0, number(r, "speedLimitMps", 2)), number(c, "maxYawRateDegS"),
            number(c, "relativeSteeringLimitDeg"), number(c, "relativeThrottleLimitPercent"));
        const auto sent = send("ROVER_ACRO", command);
        r.update(command);
        r["commandedForwardSpeedMps"] = command["forwardSpeedMps"];
        r["yawRateRadS"] = number(command, "yawRateDegS") * radians;
        auto out = command;
        out.update({{"safe", sent.value("ok", false)},
                    {"throttle", number(r, "throttlePercent", 0)},
                    {"steering", number(r, "steeringDegrees", 0)},
                    {"propulsion", "acro_velocity"},
                    {"sameDirectionLimited", false},
                    {"sameDirectionSteeringLimit", nullptr}});
        if (!sent.value("ok", false))
            out["reason"] = sent.value("error", std::string("drive_failed"));
        return out;
    }
    auto &motor = method == "neural" ? *neural_motor : *traditional_motor;
    const auto output = motor.propulsion(mono, r, t, core.connected());
    if (!output.value("safe", false)) {
        neutral(output.value("reason", std::string()));
        return output;
    }
    const auto governed = intermittent->govern_drive(output, r);
    auto out = drive(number(governed, "throttle", 0), number(governed, "steering", 0),
                     method == "neural" ? "UWB_NEURAL" : "UWB_TRADITIONAL");
    out.update(
        {{"propulsion", method == "neural" ? "dual_thrust" : "dual_motor"},
         {"sameDirectionLimited", false},
         {"sameDirectionSteeringLimit", std::abs(number(out, "throttle")) * motor.steering_span /
                                            motor.throttle_span * (method == "neural" ? 1 : .8)}});
    return out;
}
bool ControlRuntime::neutral_feedback() const {
    const auto outputs = t.value("servoOutputs", Json::object());
    const double left =
                     number(outputs, "ch" + std::to_string(config.value("rover_left_channel", 1))),
                 right =
                     number(outputs, "ch" + std::to_string(config.value("rover_right_channel", 3))),
                 center = number(config, "default_motor_pwm", 1500);
    return left > 0 && right > 0 && std::abs(left - center) <= 20 && std::abs(right - center) <= 20;
}
bool ControlRuntime::holding(Json &result) {
    if (control_mode == "relative_acro") {
        const auto hold = drift->update(wall, t, result);
        const auto sent = send("ROVER_ACRO", {{"forwardSpeedMps", hold["forwardSpeedMps"]},
                                              {"yawRateDegS", hold["yawRateDegS"]}});
        if (!sent.value("ok", false)) {
            stop(sent.value("error", std::string("hold_failed")), true);
            return false;
        }
        result.update({{"gpsDriftHold", hold},
                       {"forwardSpeedMps", hold["forwardSpeedMps"]},
                       {"yawRateDegS", hold["yawRateDegS"]},
                       {"yawRateRadS", number(hold, "yawRateDegS") * radians},
                       {"holdFlightMode", "ACRO"},
                       {"outputNeutralVerified", neutral_feedback()}});
        return true;
    }
    if (holding_at == 0) {
        holding_at = mono;
        manual_confirmed = mode_requested = resume_started = resume_requested = 0;
    }
    const auto mode = upper(t.value("flightMode", std::string()));
    if (mode != "MANUAL" && mono - mode_requested >= 500) {
        send("SET_MODE", {{"mode", "MANUAL"}});
        mode_requested = mono;
    }
    if (mode == "MANUAL" && manual_confirmed == 0)
        manual_confirmed = mono;
    const bool verified = neutral_feedback();
    const auto outputs = t.value("servoOutputs", Json::object());
    result.update(
        {{"holdFlightMode", "MANUAL"},
         {"outputNeutralVerified", verified},
         {"leftPwm",
          nullable(number(outputs, "ch" + std::to_string(config.value("rover_left_channel", 1))))},
         {"rightPwm", nullable(number(outputs, "ch" + std::to_string(config.value(
                                                          "rover_right_channel", 3))))}});
    if (t.value("armed", false) && mode == "MANUAL" && manual_confirmed > 0 &&
        mono - manual_confirmed > 1500 && !verified) {
        send("EMERGENCY_STOP");
        stop("holding_output_not_neutral", true);
        return false;
    }
    return true;
}
void ControlRuntime::update_follow() {
    if (!phase.empty()) {
        advance_start();
        return;
    }
    if (!active) {
        const auto pre = controller().preflight(wall, t, core.connected(), "auto");
        if (resume->pending) {
            const bool ready = resume->observe(wall, t, pre);
            auto r = idle(ready ? "auto_resume_ready" : "awaiting_trusted_uwb", "awaiting_signal");
            r["waitingReason"] =
                pre.value("safe", false) ? Json(nullptr) : pre.value("reason", Json(nullptr));
            publish(r, false);
            if (ready && resume->begin_attempt())
                start(true);
            return;
        }
        const auto reason =
            pre.value("safe", false) ? "method_selected" : pre.value("reason", std::string());
        if (state.value("reason", std::string()) != reason)
            publish(pre.value("safe", false) ? idle(reason, "", pre) : pre, false);
        return;
    }
    auto result = intermittent->update(controller(), wall, t, core.connected(), control_mode);
    if (result.value("safe", false) && !result.value("pausePropulsion", false))
        result = resume->ramp(result, wall);
    if (!result.value("safe", false)) {
        const auto reason = result.value("reason", std::string("follow_failed")),
                   status = state.value("status", std::string());
        if (control_mode != "relative_acro" && (status == "holding" || status == "resuming") &&
            (reason == "manual_required" || reason == "acro_required" ||
             reason == "guided_required")) {
            const auto mode = result.value("requiredFlightMode", Json(nullptr));
            required_mode = mode.is_string()                    ? mode.get<std::string>()
                            : control_mode == "relative_manual" ? "MANUAL"
                            : control_mode == "guided_velocity" ? "GUIDED"
                                                                : "ACRO";
            if (resume_started == 0) {
                resume_started = mono;
                resume_requested = 0;
                controller().reset();
            }
            neutral("resume_mode_transition");
            if (mono - resume_requested >= 500) {
                send("SET_MODE", {{"mode", required_mode}});
                resume_requested = mono;
            }
            if (mono - resume_started > 5000) {
                stop("resume_mode_timeout", true);
                return;
            }
            auto r = state;
            r.update(idle("resume_mode_transition", "resuming",
                          {{"requiredFlightMode", required_mode}}));
            r["safe"] = true;
            publish(r, true);
            return;
        }
        stop(reason, true, false);
        return;
    }
    if (result.value("pausePropulsion", false)) {
        if (control_mode != "relative_acro")
            neutral(result.value("reason", std::string()));
        if (holding(result))
            publish(result, true);
        return;
    }
    drift->track(wall, t, result);
    const bool hold = result.value("reason", std::string()) == "holding_distance";
    if (!hold)
        reset_hold();
    if (control_mode == "relative_acro" || control_mode == "relative_manual") {
        const auto output = relative_drive(result);
        if (!output.value("safe", false)) {
            stop(output.value("reason", std::string("drive_failed")), true);
            return;
        }
        result.update({{"throttlePercent", output["throttle"]},
                       {"steeringDegrees", output["steering"]},
                       {"sameDirectionSteeringLimitDeg",
                        output.value("sameDirectionSteeringLimit", Json(nullptr))},
                       {"sameDirectionLimited", output.value("sameDirectionLimited", false)}});
        if (output.contains("leftPwm"))
            result["leftPwm"] = output["leftPwm"];
        if (output.contains("rightPwm"))
            result["rightPwm"] = output["rightPwm"];
    } else {
        const auto sent =
            send("ROVER_BODY_VELOCITY",
                 {{"forwardSpeedMps", hold ? 0 : number(result, "forwardSpeedMps", 0)},
                  {"yawRateRadS", hold ? 0 : number(result, "yawRateRadS", 0)},
                  {"speedLimitMps", number(result, "speedLimitMps", 2)}});
        if (!sent.value("ok", false)) {
            stop(sent.value("error", std::string("drive_failed")), true);
            return;
        }
        if (hold)
            neutral("holding_distance");
    }
    if (hold && !holding(result))
        return;
    publish(result, true);
}
Json ControlRuntime::command(const Json &request, double at, double clock) {
    wall = at;
    mono = clock;
    t = core.telemetry(wall, mono);
    guard_uwb();
    if (t.value("uwb", Json::object()).value("clearanceStop", false)) {
        if (active || !phase.empty() || resume->pending)
            neutral("raw_distance_clearance");
        core.acknowledge_uwb_clearance(wall);
    }
    try {
        const auto name = request.value("command", std::string());
        const auto params = request.value("params", Json::object());
        if (!params.is_object())
            throw std::invalid_argument("invalid_params");
        Json result;
        if (name == "CONTROL_STATUS")
            return snapshot();
        if (name == "FOLLOW_METHOD")
            result = select(params.value("method", std::string()));
        else if (name == "FOLLOW_START") {
            if (params.contains("trackingMethod")) {
                if (resume->pending)
                    resume->cancel();
                result = select(params.value("trackingMethod", std::string()));
                if (!result.value("ok", false))
                    return result;
            }
            result = start();
        } else if (name == "FOLLOW_STOP") {
            stop(params.value("reason", std::string("manual_stop")), params.value("disarm", true));
            result = {{"ok", true}};
        } else if (name == "MANUAL_TARGET") {
            if (!std::isfinite(number(params, "throttle")) ||
                !std::isfinite(number(params, "steering")))
                throw std::invalid_argument("invalid_manual_target");
            const auto requested =
                normalized(number(params, "throttle"), number(params, "steering"));
            const bool nonzero =
                number(requested, "throttle") != 0 || number(requested, "steering") != 0;
            if (nonzero && (active || !phase.empty() || resume->pending))
                stop("manual_override", !phase.empty());
            if (nonzero && !core.connected())
                throw std::invalid_argument("pixhawk_offline");
            if (nonzero && !t.value("pilotSteering", Json::object()).value("ready", false))
                throw std::invalid_argument("pilot_steering_not_ready");
            const double throttle =
                             shape_axis(number(requested, "throttle"), throttle_limit,
                                        number(config, "rover_manual_throttle_deadband_percent", 4),
                                        number(config, "rover_manual_input_expo", 1.4)),
                         steering =
                             shape_axis(number(requested, "steering"), steering_limit,
                                        number(config, "rover_manual_steering_deadband_deg", 2),
                                        number(config, "rover_manual_input_expo", 1.4));
            manual_motor->set_target({{"throttle", throttle},
                                      {"steering", steering},
                                      {"rawSteering", requested["steering"]}});
            last_manual = mono;
            source = params.value("source", std::string("WEB"));
            result = rover;
            result.update({{"ok", true},
                           {"targetThrottle", throttle},
                           {"targetSteering", steering},
                           {"clamped", requested["clamped"]}});
            return result;
        } else if (name == "CONTROL_NEUTRAL") {
            stop(params.value("reason", std::string("neutral")), false);
            return {{"ok", true}};
        } else {
            if (name == "DISARM" || name == "EMERGENCY_STOP" || name == "IMU_CALIBRATION_START")
                stop(name == "DISARM"           ? "disarmed"
                     : name == "EMERGENCY_STOP" ? "emergency_stop"
                                                : "imu_calibration",
                     false);
            // Legacy low-level drive callers cannot compete with an active
            // controller. Their neutral command is still an explicit stop.
            if (name.starts_with("ROVER_"))
                stop("manual_override", !phase.empty());
            return core.command(request, wall, mono);
        }
        result["state"] = state;
        result["transitionId"] = transition;
        return result;
    } catch (const std::exception &e) {
        if (request.is_object() && request.value("command", std::string()) == "MANUAL_TARGET")
            neutral("invalid_manual_target");
        return {{"ok", false}, {"error", e.what()}};
    }
}
void ControlRuntime::tick(double at, double clock) {
    wall = at;
    mono = clock;
    t = core.telemetry(wall, mono);
    guard_uwb();
    if (t.value("uwb", Json::object()).value("clearanceStop", false)) {
        if (active || !phase.empty() || resume->pending)
            neutral("raw_distance_clearance");
        core.acknowledge_uwb_clearance(wall);
    }
    if (std::isfinite(last_tick) &&
        mono - last_tick > number(config, "rover_command_timeout_ms", 500) &&
        (active || !phase.empty()))
        stop("command_lease_expired", true);
    last_tick = mono;
    if ((active || !phase.empty()) && !core.connected())
        stop("pixhawk_offline", true);
    if (mono >= next_follow) {
        next_follow = mono + 1000 / clamp(number(raw, "control_rate_hz", 10), 2, 20);
        try {
            update_follow();
        } catch (const std::exception &e) {
            stop("control_internal_error", true);
            emit("log", {{"level", "ERROR"}, {"message", e.what()}});
        }
    }
    if (mono >= next_manual) {
        next_manual = mono + 1000 / clamp(number(config, "rover_manual_slew_rate_hz", 20), 10, 50);
        if (!active && phase.empty()) {
            const bool nonzero = number(manual_motor->target, "throttle", 0) != 0 ||
                                 number(manual_motor->target, "steering", 0) != 0;
            if (nonzero && (mono - last_manual > number(config, "rover_command_timeout_ms", 500) ||
                            !core.connected()))
                neutral("command_lease_expired");
            else {
                const auto previous = manual_motor->output, out = manual_motor->step(mono);
                if (previous != out || number(out, "throttle", 0) != 0 ||
                    number(out, "steering", 0) != 0)
                    drive(number(out, "throttle", 0), number(out, "steering", 0), "SLEW:" + source);
            }
        }
    }
    if (mono - last_publish >= 1000) {
        emit("control_status", snapshot());
        last_publish = mono;
    }
}
void ControlRuntime::shutdown(double at, double clock) {
    wall = at;
    mono = clock;
    stop("service_stopped", false);
}
} // namespace manta
