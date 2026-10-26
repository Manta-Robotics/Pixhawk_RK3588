#include "manta/follow_control.hpp"

namespace manta {
Json normalize_follow_config(const Json &raw) {
    Json c = Json::object();
    struct Field {
        const char *target;
        const char *source;
        double fallback, low, high;
    };
    const Field fields[] = {
        {"followingDistanceM", "following_distance_m", 1.5, .1, INFINITY},
        {"traditionalHeadingGain", "traditional_heading_gain", 1.2, 0, INFINITY},
        {"traditionalYawRateDamping", "traditional_yaw_rate_damping", .35, 0, INFINITY},
        {"traditionalMinimumTurnSpeedScale", "traditional_minimum_turn_speed_scale", .15, 0, 1},
        {"safetyDistanceM", "safety_distance_m", 1, .1, INFINITY},
        {"safetyResumeDistanceM", "safety_resume_distance_m", 1.2, .2, INFINITY},
        {"hardStopResumeDistanceM", "hard_stop_resume_distance_m", 1.08, .1, INFINITY},
        {"closeApproachMinSpeedMps", "close_approach_min_speed_mps", .12, .02, 1},
        {"closeApproachMaxSpeedMps", "close_approach_max_speed_mps", .3, .02, 1.5},
        {"closeApproachMinThrottlePercent", "close_approach_min_throttle_percent", 20, 1, 100},
        {"closeApproachMaxThrottlePercent", "close_approach_max_throttle_percent", 30, 1, 100},
        {"distanceHoldUpperM", "distance_hold_upper_m", .55, .1, INFINITY},
        {"distanceRecoveryResumeM", "distance_recovery_resume_m", .5, .1, INFINITY},
        {"maxSpeedMps", "max_speed_mps", 5, .1, 5},
        {"maxReverseSpeedMps", "max_reverse_speed_mps", .2, 0, 1.5},
        {"commissioningSpeedMps", "commissioning_speed_mps", .35, .1, 5},
        {"rangeGain", "range_gain", number(raw, "position_gain", .65), .01, INFINITY},
        {"rangeRateGain", "range_rate_gain", .25, 0, INFINITY},
        {"rangeIntegralGain", "range_integral_gain", .08, 0, INFINITY},
        {"rangeIntegralLimitMps", "range_integral_limit_mps", .25, 0, 2},
        {"targetSpeedAlpha", "target_speed_alpha", number(raw, "velocity_alpha", .2), .01, 1},
        {"bearingGain", "bearing_gain", 1.2, .01, INFINITY},
        {"bearingRateGain", "bearing_rate_gain", .18, 0, INFINITY},
        {"bearingRateAlpha", "bearing_rate_alpha", .3, .01, 1},
        {"bearingFilterAlpha", "bearing_filter_alpha", .2, .01, 1},
        {"bearingDeadbandDeg", "bearing_deadband_deg", 4, 0, 30},
        {"bearingReverseConfirmDeg", "bearing_reverse_confirm_deg", 8, 0, 60},
        {"bearingReverseConfirmMs", "bearing_reverse_confirm_ms", 300, 0, 2000},
        {"ilosIntegralGain", "ilos_integral_gain", .12, 0, 2},
        {"ilosIntegralDecay", "ilos_integral_decay", .25, 0, 5},
        {"ilosMaxCorrectionDeg", "ilos_max_correction_deg", 12, 0, 45},
        {"lookaheadBaseM", "lookahead_base_m", 1.2, .2, 10},
        {"lookaheadSpeedGainS", "lookahead_speed_gain_s", .8, 0, 5},
        {"lookaheadMinM", "lookahead_min_m", 1, .2, 10},
        {"lookaheadMaxM", "lookahead_max_m", 3, .3, 20},
        {"maxCurvatureMInv", "max_curvature_m_inv", .8, .05, 5},
        {"curvatureSpeedGain", "curvature_speed_gain", 1.5, 0, 10},
        {"minimumCurveSpeedScale", "minimum_curve_speed_scale", .3, .05, 1},
        {"sameDirectionSteeringRatio", "same_direction_steering_ratio", .45, .01, 2},
        {"bearingSlowdownDeg", "bearing_slowdown_deg", 20, 0, 170},
        {"bearingStopDeg", "bearing_stop_deg", 55, 1, 179},
        {"fieldOfViewHalfDeg", "field_of_view_half_deg", 45, 30, 90},
        {"maxYawRateDegS", "max_yaw_rate_deg_s", 35, 1, 120},
        {"relativeThrottleLimitPercent", "relative_throttle_limit_percent", 25, 1, 100},
        {"minimumEffectiveThrottlePercent", "relative_min_effective_throttle_percent", 0, 0, 100},
        {"minimumThrottleRangeErrorM", "minimum_throttle_range_error_m", .35, 0, 5},
        {"relativeSteeringLimitDeg", "relative_steering_limit_deg", 30, 1, 45},
        {"steeringSlewDegS", "steering_slew_deg_s", 18, 1, 180},
        {"steeringReturnDegS", "steering_return_deg_s", 30, 1, 180},
        {"throttleAccelPercentS", "relative_throttle_accel_percent_s", 12, 1, 100},
        {"throttleDecelPercentS", "relative_throttle_decel_percent_s", 25, 1, 100},
        {"turnDecelPercentS", "relative_turn_decel_percent_s", number(raw, "relative_throttle_decel_percent_s", 25), 1, 100},
        {"turnMinimumThrottlePercent", "relative_turn_min_throttle_percent", 0, 0, 100},
        {"rangeDeadbandM", "range_deadband_m", .12, 0, 2},
        {"uwbTimeoutMs", "uwb_timeout_ms", 700, 100, INFINITY},
        {"navigationTimeoutMs", "navigation_timeout_ms", 700, 100, INFINITY},
        {"bearingScale", "bearing_scale", .8148, .01, INFINITY},
        {"bearingOffsetDeg", "bearing_offset_deg", -4.816, -INFINITY, INFINITY}};
    for (const auto &f : fields)
        c[f.target] = clamp(number(raw, f.source, f.fallback), f.low, f.high);
    const auto n = [&](const char *key) { return number(c, key); };
    c["minimumGpsFixType"] = std::max(2.0, std::floor(number(raw, "minimum_gps_fix_type", 2) + .5));
    c["bearingSign"] = number(raw, "bearing_sign", -1) < 0 ? -1 : 1;
    c["commissioningMode"] = raw.value("commissioning_mode", true);
    c["hardStopEnabled"] = raw.value("hard_stop_enabled", true);
    c["closeApproachEnabled"] = raw.value("close_approach_enabled", true);
    c["manualPropulsion"] = raw.value("manual_propulsion", std::string()) == "single_side";
    const auto execution = raw.contains("execution_mode") && raw["execution_mode"].is_string()
                               ? raw["execution_mode"].get<std::string>()
                               : std::string();
    c["executionMode"] =
        execution == "acro" || execution == "relative_imu" ? Json(execution) : Json(nullptr);
    c["controllerProfile"] = raw.value("controller_profile", std::string()) == "traditional_fast"
                                 ? "traditional_fast"
                                 : "ilos_curvature";
    c["prediction"] = raw.value("prediction", Json::object());
    c["relativeNmpc"] = {{"enabled", false},
                         {"horizon_s", 2.4},
                         {"solve_budget_ms", 15},
                         {"surge_time_constant_s", 1},
                         {"yaw_time_constant_s", .35},
                         {"surge_map_exponent", .5},
                         {"model_cruise_speed_mps", .8},
                         {"model_position_error_mps", .1},
                         {"steering_share", .8}};
    c["relativeNmpc"].update(raw.value("relative_nmpc", Json::object()));
    const auto gps = raw.value("gps_speed_closed_loop", Json::object());
    c["gpsSpeedClosedLoop"] = {
        {"enabled", gps.value("enabled", false)},
        {"acquireMs", std::max(0.0, number(gps, "acquire_ms", 2000))},
        {"kp", std::max(0.0, number(gps, "kp", 20))},
        {"ki", std::max(0.0, number(gps, "ki", 8))},
        {"maximumObservedSpeedMps", std::max(.1, number(gps, "maximum_observed_speed_mps", 5))}};
    if (n("bearingStopDeg") <= n("bearingSlowdownDeg"))
        c["bearingStopDeg"] = std::min(179.0, n("bearingSlowdownDeg") + 1);
    c["bearingReverseConfirmDeg"] =
        std::max(n("bearingDeadbandDeg"), n("bearingReverseConfirmDeg"));
    c["lookaheadMaxM"] = std::max(n("lookaheadMinM"), n("lookaheadMaxM"));
    c["hardStopResumeDistanceM"] =
        clamp(n("hardStopResumeDistanceM"), n("safetyDistanceM"), n("safetyResumeDistanceM"));
    c["closeApproachMaxSpeedMps"] =
        std::max(n("closeApproachMinSpeedMps"), n("closeApproachMaxSpeedMps"));
    c["closeApproachMinThrottlePercent"] =
        std::min(n("closeApproachMinThrottlePercent"), n("relativeThrottleLimitPercent"));
    c["closeApproachMaxThrottlePercent"] =
        clamp(n("closeApproachMaxThrottlePercent"), n("closeApproachMinThrottlePercent"),
              n("relativeThrottleLimitPercent"));
    c["distanceHoldUpperM"] =
        clamp(n("distanceHoldUpperM"), n("safetyDistanceM"), n("safetyResumeDistanceM"));
    c["distanceRecoveryResumeM"] =
        clamp(n("distanceRecoveryResumeM"), n("safetyDistanceM"), n("distanceHoldUpperM"));
    c["minimumEffectiveThrottlePercent"] =
        std::min(n("minimumEffectiveThrottlePercent"), n("relativeThrottleLimitPercent"));
    return c;
}

Json traditional_guidance(const Json &c, double bearing, double yaw_rate, double speed) {
    const double heading = wrap(bearing),
                 scale = std::max(number(c, "traditionalMinimumTurnSpeedScale"),
                                  std::cos(std::min(std::abs(heading) * radians, pi / 2)));
    const double raw = number(c, "traditionalHeadingGain") * heading -
                       number(c, "traditionalYawRateDamping") * yaw_rate;
    return {{"headingErrorDeg", heading},
            {"measuredYawRateDegS", yaw_rate},
            {"speedScale", scale},
            {"forwardSpeedMps", speed * scale},
            {"rawYawRateDegS", raw},
            {"desiredYawRateDegS",
             clamp(raw, -number(c, "maxYawRateDegS"), number(c, "maxYawRateDegS"))}};
}
} // namespace manta
