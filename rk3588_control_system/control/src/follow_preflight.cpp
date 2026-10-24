#include "manta/follow_control.hpp"

namespace manta {
bool usable_estimate(const Json &e, double now, const Json &p) {
    if (!p.value("enabled", false) || !e.is_object() || !e.value("usable", false))
        return false;
    const auto source = e.value("source", std::string());
    if (source != "predicted" && source != "recovering")
        return false;
    const double at = number(e, "lastMeasuredAt"), uncertainty = number(e, "uncertaintyM");
    const auto body = e.value("relativeBodyM", Json::object());
    if (number(e, "evaluatedAt") != now || !std::isfinite(at) || at <= 0 || at > now ||
        now - at > number(p, "horizon_ms", 1000) || !std::isfinite(number(body, "x")) ||
        !std::isfinite(number(body, "y")) || !std::isfinite(uncertainty) || uncertainty < 0)
        return false;
    return source == "recovering" && e.value("measurementConfirmed", false)
               ? now - at <= number(p, "measurement_gap_ms", 450)
               : uncertainty <= number(p, "max_uncertainty_m", .5);
}
Json zero_control(const std::string &reason, const Json &extra) {
    const auto mode = extra.value("controlMode", std::string("auto"));
    const bool guided = mode == "guided_velocity";
    Json out = {{"safe", false},
                {"reason", reason},
                {"controlMode", mode},
                {"requiredFlightMode", guided                      ? Json("GUIDED")
                                       : mode == "relative_manual" ? Json("MANUAL")
                                       : mode == "relative_acro"   ? Json("ACRO")
                                                                   : Json(nullptr)},
                {"navigationAvailable", false},
                {"commandFrame", guided ? "BODY_NED" : "BODY_RELATIVE"},
                {"relativeBodyM", {{"x", 0}, {"y", 0}}},
                {"forwardSpeedMps", 0},
                {"yawRateDegS", 0},
                {"yawRateRadS", 0},
                {"throttlePercent", 0},
                {"steeringDegrees", 0},
                {"velocityNed", {{"x", 0}, {"y", 0}}},
                {"desiredSpeedMps", 0}};
    out.update(extra);
    return out;
}
void FollowController::reset() {
    relative.reset();
    previous_distance = previous_bearing = previous_at = filtered_bearing = steering_at =
        throttle_at = missing;
    range_rate = bearing_rate = target_radial = range_integral = steering_command = reverse_at =
        cross_integral = throttle_command = 0;
    last_steering_sign = reverse_sign = 0;
    hard_stop = distance_recovery = false;
}
Json FollowController::preflight(double now, const Json &t, bool connected,
                                 const std::string &requested, const Json &estimate) const {
    const auto &c = config;
    const auto u = t.value("uwb", Json::object()), a = t.value("attitude", Json::object()),
               v = t.value("velocity", Json::object()), pos = t.value("position", Json::object()),
               ekf = t.value("ekf", Json::object()), gps = t.value("gps", Json::object());
    const bool estimated = usable_estimate(estimate, now, c["prediction"]);
    const double distance = estimated ? std::hypot(number(estimate["relativeBodyM"], "x"),
                                                   number(estimate["relativeBodyM"], "y"))
                                      : number(u, "distanceM");
    const double azimuth = estimated ? wrap(std::atan2(number(estimate["relativeBodyM"], "y"),
                                                       number(estimate["relativeBodyM"], "x")) /
                                                radians -
                                            number(c, "bearingOffsetDeg")) /
                                           (number(c, "bearingSign") * number(c, "bearingScale"))
                                     : number(u, "azimuthDeg");
    const double yaw = number(a, "yaw"), yaw_rate = number(a, "yawRateDegS", 0),
                 vx = number(v, "vx"), vy = number(v, "vy"),
                 uwb_at = number(u, u.contains("measurementUpdatedAt") ? "measurementUpdatedAt"
                                                                       : "updatedAt"),
                 pos_at = number(pos, "updatedAt"), ekf_at = number(ekf, "updatedAt");
    if (!connected)
        return zero_control("pixhawk_offline");
    if (!estimated &&
        (!u.value("online", false) || !u.value("fresh", false) || !std::isfinite(distance) ||
         !std::isfinite(azimuth) || !std::isfinite(uwb_at) || uwb_at > now ||
         now - uwb_at > number(c, "uwbTimeoutMs")))
        return zero_control("uwb_stale");
    if (!std::isfinite(yaw))
        return zero_control("imu_invalid");
    Json nav_reason = nullptr;
    if (number(gps, "fixType", 0) < number(c, "minimumGpsFixType"))
        nav_reason = "gps_fix_invalid";
    else if (!ekf.value("healthy", false) || !std::isfinite(ekf_at) ||
             now - ekf_at > number(c, "navigationTimeoutMs"))
        nav_reason = "ekf_unhealthy";
    else if (!std::isfinite(pos_at) || now - pos_at > number(c, "navigationTimeoutMs"))
        nav_reason = "navigation_stale";
    else if (!std::isfinite(vx) || !std::isfinite(vy))
        nav_reason = "navigation_invalid";
    if (c["executionMode"] == "relative_imu") {
        const double imu_at = number(a, "updatedAt");
        if (!std::isfinite(imu_at) || imu_at > now ||
            now - imu_at > number(c, "navigationTimeoutMs"))
            return zero_control("imu_stale");
        nav_reason = "uwb_imu_only";
    }
    const bool nav = nav_reason.is_null();
    const std::string selected = c["executionMode"] == "relative_imu" ? "relative_manual"
                                 : c["executionMode"] == "acro"       ? "relative_acro"
                                 : c.value("manualPropulsion", false) ? "relative_manual"
                                 : requested == "guided_velocity"     ? "guided_velocity"
                                 : requested == "relative_acro"       ? "relative_acro"
                                 : nav                                ? "guided_velocity"
                                                                      : "relative_acro";
    if (c["executionMode"] == "acro") {
        if (!nav)
            return zero_control("acro_velocity_unavailable", {{"controlMode", "relative_acro"},
                                                              {"requiredFlightMode", "ACRO"},
                                                              {"navigationReason", nav_reason}});
        const auto acro = t.value("acroControl", Json::object());
        const double at = number(acro, "updatedAt");
        if (!acro.value("ready", false) || !std::isfinite(at) || at > now ||
            now - at > number(c, "navigationTimeoutMs"))
            return zero_control("acro_parameters_not_ready",
                                {{"controlMode", "relative_acro"}, {"requiredFlightMode", "ACRO"}});
    }
    const auto pilot = t.value("pilotSteering", Json::object());
    const double pilot_type = number(pilot, "type");
    if (selected == "relative_manual" &&
        (!pilot.value("ready", false) || !std::isfinite(pilot_type) ||
         pilot_type != std::trunc(pilot_type) || pilot_type < 0 || pilot_type > 3))
        return zero_control("pilot_steering_not_ready",
                            {{"controlMode", selected}, {"requiredFlightMode", "MANUAL"}});
    if (selected == "guided_velocity" && !nav)
        return zero_control(nav_reason.is_string() ? nav_reason.get<std::string>()
                                                   : "navigation_invalid",
                            {{"controlMode", selected},
                             {"requiredFlightMode", "GUIDED"},
                             {"navigationReason", nav_reason}});
    return {{"safe", true},
            {"reason", "preflight_ok"},
            {"controlMode", selected},
            {"requiredFlightMode", selected == "relative_manual"   ? "MANUAL"
                                   : selected == "guided_velocity" ? "GUIDED"
                                                                   : "ACRO"},
            {"navigationAvailable", nav},
            {"navigationReason", nav_reason},
            {"distanceM", distance},
            {"azimuthDeg", azimuth},
            {"yawDeg", yaw},
            {"measuredYawRateDegS", yaw_rate},
            {"boatVx", nullable(vx)},
            {"boatVy", nullable(vy)}};
}
} // namespace manta
