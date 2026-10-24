#include "manta/follow_control.hpp"

namespace manta {
void RelativeControl::reset() {
    at = measurement_at = distance = bearing = missing;
    range_rate = cruise = throttle = steering = 0;
    gps.reset();
    nmpc.reset();
}
void RelativeControl::pause() {
    if (c["relativeNmpc"].value("enabled", false)) {
        throttle = steering = 0;
        nmpc.track_applied(0, 0);
        gps.reset();
        return;
    }
    const double previous = cruise;
    reset();
    cruise = previous;
}
void RelativeControl::track_applied(double t, double s) {
    if (c["relativeNmpc"].value("enabled", false)) {
        throttle = t;
        steering = s;
        nmpc.track_applied(t, s);
    }
}
Json RelativeControl::update(double now, const Json &preflight, const Json &telemetry,
                             const Json &estimate, const Json &policy) {
    const auto n = [&](const char *key) { return number(c, key); };
    const auto &p = c["prediction"];
    const double d = number(preflight, "distanceM"),
                 dt = std::isfinite(at) ? clamp((now - at) / 1000, .001, .25) : .1;
    const double calibrated =
                     number(preflight, "azimuthDeg") * n("bearingSign") * n("bearingScale") +
                     n("bearingOffsetDeg"),
                 measured_bearing = wrap(calibrated);
    bearing = std::isfinite(bearing)
                  ? wrap(bearing + n("bearingFilterAlpha") * wrap(measured_bearing - bearing))
                  : measured_bearing;
    const double sample = number(telemetry["uwb"], "measurementUpdatedAt");
    const auto source =
        estimate.is_object() ? estimate.value("source", std::string()) : std::string("measured");
    if ((source == "measured" || source == "recovering") && std::isfinite(sample) &&
        (!std::isfinite(measurement_at) || sample > measurement_at)) {
        if (std::isfinite(distance) && std::isfinite(measurement_at) &&
            sample - measurement_at <= n("uwbTimeoutMs")) {
            const double sample_dt = (sample - measurement_at) / 1000,
                         measured = clamp((d - distance) / sample_dt, -3, 3),
                         alpha = 1 - std::pow(1 - n("targetSpeedAlpha"), sample_dt / .1);
            range_rate += alpha * (measured - range_rate);
        }
        distance = d;
        measurement_at = sample;
    }
    at = now;
    const double error = d - n("followingDistanceM"),
                 controlled = std::abs(error) <= n("rangeDeadbandM") ? 0 : error;
    const double progress =
        clamp((d - n("distanceHoldUpperM")) /
                  std::max(.01, n("safetyResumeDistanceM") - n("distanceHoldUpperM")),
              0, 1);
    const double approach = n("closeApproachMaxSpeedMps") * progress +
                            n("rangeGain") * std::max(0.0, d - n("safetyResumeDistanceM"));
    const double deceleration = std::max(.05, number(p, "braking_deceleration_mps2", .3)),
                 age = std::max(0.0, (now - sample) / 1000),
                 reaction = .6 + std::min(age, n("uwbTimeoutMs") / 1000);
    const double margin = std::max(.05, number(p, "measurement_noise_m", .12)),
                 clearance = std::max(0.0, d - n("followingDistanceM") - margin);
    const double braking_limit =
        std::max(0.0, std::sqrt(squared(deceleration * reaction) + 2 * deceleration * clearance) -
                          deceleration * reaction);
    const double desired_closing =
        clamp(n("rangeGain") * controlled, -n("maxReverseSpeedMps"),
              std::min({approach, .5 * braking_limit, n("maxSpeedMps")}));
    const double rate_error = desired_closing + range_rate,
                 nominal_scale = n("relativeThrottleLimitPercent") / n("maxSpeedMps"),
                 kp = 2 * nominal_scale, ki = .4 * nominal_scale, old_cruise = cruise;
    cruise = clamp(cruise + ki * rate_error * dt, 0, n("relativeThrottleLimitPercent"));
    double requested = cruise + kp * rate_error;
    if ((requested > n("relativeThrottleLimitPercent") && rate_error > 0) ||
        (requested < 0 && rate_error < 0))
        cruise = old_cruise;
    requested = clamp(cruise + kp * rate_error, 0, n("relativeThrottleLimitPercent"));
    if (policy.is_object())
        requested =
            clamp(number(policy, "nominalThrottlePercent"), 0, n("relativeThrottleLimitPercent"));
    const bool safety = d <= n("safetyDistanceM");
    Json result = {{"safe", true},
                   {"controlMode", "relative_manual"},
                   {"requiredFlightMode", "MANUAL"},
                   {"commandFrame", "BODY_RELATIVE"},
                   {"relativeImuControl", true},
                   {"distanceM", d},
                   {"controlBearingDeg", bearing},
                   {"rangeRateMps", range_rate},
                   {"rangeErrorM", error},
                   {"desiredClosingSpeedMps", desired_closing},
                   {"velocityNed", nullptr},
                   {"holdingDistanceM", n("safetyDistanceM")},
                   {"hardStopResumeDistanceM", n("hardStopResumeDistanceM")}};
    if (c["relativeNmpc"].value("enabled", false)) {
        const auto g = gps.update(now, telemetry, bearing, range_rate, desired_closing, throttle,
                                  missing, safety);
        const bool active = g["active"];
        const auto plan =
            safety ? Json{{"feasible", false},
                          {"reason", "holding_distance"},
                          {"mode", active ? "gps_imu" : "uwb_imu_relative"},
                          {"path", Json::array()}}
                   : nmpc.solve(now, telemetry, d, measured_bearing, estimate, requested, active);
        const bool feasible = plan["feasible"];
        throttle = feasible ? number(plan, "throttlePercent") : 0;
        steering = feasible ? number(plan, "steeringDegrees") : 0;
        nmpc.track_applied(throttle, steering);
        gps.track_applied(throttle, dt);
        cruise = clamp(cruise + (throttle - requested) * dt, 0, n("relativeThrottleLimitPercent"));
        result.update(
            {{"reason", safety     ? "holding_distance"
                        : feasible ? "tracking"
                                   : "nmpc_" + plan["reason"].get<std::string>()},
             {"status", safety     ? "holding"
                        : feasible ? "tracking"
                                   : "waiting_target"},
             {"pausePropulsion", !safety && !feasible},
             {"planner", "nmpc"},
             {"nmpcActive", feasible},
             {"nmpcMode", plan["mode"]},
             {"nmpc", plan},
             {"navigationAvailable", active},
             {"navigationReason", active ? Json(nullptr) : Json("uwb_imu_only")},
             {"gpsSpeedControlActive", active},
             {"gpsSpeedControlReason", active ? Json("gps_nmpc_state_feedback") : g["reason"]},
             {"speedControlMode", active ? "gps_nmpc_closed_loop" : "uwb_relative_nmpc"},
             {"measuredForwardSpeedMps", active ? g["measuredForwardSpeedMps"] : Json(nullptr)},
             {"absoluteSpeedObservable", active},
             {"forwardSpeedMps", active && feasible ? plan["nominalSpeedMps"] : Json(nullptr)},
             {"desiredSpeedMps", active && feasible ? plan["nominalSpeedMps"] : Json(nullptr)},
             {"bearingDeg", calibrated},
             {"throttlePercent", throttle},
             {"steeringDegrees", steering},
             {"desiredThrottlePercent", throttle},
             {"yawRateDegS", feasible ? number(plan, "yawRateDegS") : 0},
             {"yawRateRadS", feasible ? number(plan, "yawRateDegS") * radians : 0},
             {"speedFeedForwardSource", active ? "gps_state_feedback_nmpc" : "relative_state_nmpc"},
             {"relativeBodyM",
              {{"x", d * std::cos(bearing * radians)}, {"y", d * std::sin(bearing * radians)}}}});
        return result;
    }
    const auto g =
        gps.update(now, telemetry, bearing, range_rate, desired_closing, throttle,
                   policy.is_object() ? number(policy, "nominalSpeedMps") : missing, safety);
    const bool active = g["active"];
    if (active)
        requested = number(g, "throttlePercent");
    const double closing = std::max(0.0, -range_rate);
    const bool braking = closing > braking_limit + .02;
    if (braking)
        requested = std::min(requested, std::max(0.0, throttle - n("throttleDecelPercentS") * dt -
                                                          kp * (closing - braking_limit)));
    const auto turn =
        policy.is_object()
            ? Json{{"speedScale",
                    std::max(0.0, std::cos(number(policy, "headingErrorDeg") * radians))},
                   {"desiredYawRateDegS", clamp(number(policy, "yawRateDegS"), -n("maxYawRateDegS"),
                                                n("maxYawRateDegS"))}}
            : traditional_guidance(c, bearing, number(preflight, "measuredYawRateDegS"));
    const double alignment = clamp((n("bearingStopDeg") - std::abs(bearing)) /
                                       (n("bearingStopDeg") - n("bearingSlowdownDeg")),
                                   0, 1);
    requested *= number(turn, "speedScale") * alignment;
    if (safety) {
        requested = 0;
        cruise = std::min(cruise, throttle);
    }
    const double step =
        (requested < throttle ? n("throttleDecelPercentS") : n("throttleAccelPercentS")) * dt;
    throttle += clamp(requested - throttle, -step, step);
    if (safety || braking || alignment == 0)
        throttle = std::min(throttle, requested);
    cruise = policy.is_object() ? 0
                                : clamp(cruise + (throttle - (cruise + kp * rate_error)) * dt, 0,
                                        n("relativeThrottleLimitPercent"));
    const double desired_steering = clamp(
        number(turn, "desiredYawRateDegS") / n("maxYawRateDegS") * n("relativeSteeringLimitDeg"),
        -n("relativeSteeringLimitDeg"), n("relativeSteeringLimitDeg"));
    steering +=
        clamp(desired_steering - steering, -n("steeringSlewDegS") * dt, n("steeringSlewDegS") * dt);
    if (safety)
        steering = 0;
    gps.track_applied(throttle, dt);
    result.update(
        {{"reason", safety ? "holding_distance" : "tracking"},
         {"status", safety ? "holding" : "tracking"},
         {"navigationAvailable", active},
         {"navigationReason", active ? Json(nullptr) : Json("uwb_imu_only")},
         {"gpsSpeedControlActive", active},
         {"gpsSpeedControlReason", g["reason"]},
         {"speedControlMode", active ? "gps_speed_closed_loop" : "uwb_relative"},
         {"measuredForwardSpeedMps", active ? g["measuredForwardSpeedMps"] : Json(nullptr)},
         {"speedErrorMps", active ? g["speedErrorMps"] : Json(nullptr)},
         {"absoluteSpeedObservable", active},
         {"forwardSpeedMps", active ? g["desiredSpeedMps"] : Json(nullptr)},
         {"desiredSpeedMps", active ? g["desiredSpeedMps"] : Json(nullptr)},
         {"targetRadialSpeedMps", nullptr},
         {"speedFeedForwardSource", active ? "gps_speed_pi" : "learned_cruise_thrust"},
         {"bearingDeg", measured_bearing},
         {"relativeApproachLimitMps", approach},
         {"brakingClosingLimitMps", braking_limit},
         {"brakingActive", braking},
         {"learnedCruiseThrottlePercent", policy.is_object() ? Json(nullptr) : Json(cruise)},
         {"desiredThrottlePercent", requested},
         {"throttlePercent", throttle},
         {"steeringDegrees", steering},
         {"yawRateDegS", turn["desiredYawRateDegS"]},
         {"yawRateRadS", number(turn, "desiredYawRateDegS") * radians},
         {"relativeBodyM",
          {{"x", d * std::cos(measured_bearing * radians)},
           {"y", d * std::sin(measured_bearing * radians)}}}});
    return result;
}
} // namespace manta
