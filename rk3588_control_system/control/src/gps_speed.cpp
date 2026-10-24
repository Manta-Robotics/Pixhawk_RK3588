#include "manta/follow_control.hpp"

namespace manta {
void GpsSpeedLoop::reset() {
    good_since = at = target_radial = measurement_at = missing;
    active = false;
    integral = request = 0;
}
std::string GpsSpeedLoop::quality(double now, const Json &t) const {
    const auto &o = c["gpsSpeedClosedLoop"];
    const auto fresh = [&](const Json &obj) {
        const double time = number(obj, "updatedAt");
        return std::isfinite(time) && time <= now && now - time <= number(c, "navigationTimeoutMs");
    };
    if (!o.value("enabled", false))
        return "disabled";
    if (number(t.value("gps", Json::object()), "fixType", 0) < number(c, "minimumGpsFixType"))
        return "gps_fix_invalid";
    if (!fresh(t.value("gps", Json::object())))
        return "gps_stale";
    if (!t.value("ekf", Json::object()).value("healthy", false) ||
        !fresh(t.value("ekf", Json::object())))
        return "ekf_unavailable";
    if (!fresh(t.value("position", Json::object())))
        return "velocity_stale";
    if (!fresh(t.value("attitude", Json::object())) ||
        !std::isfinite(number(t.value("attitude", Json::object()), "yaw")))
        return "imu_stale";
    const auto v = t.value("velocity", Json::object());
    const double x = number(v, "vx"), y = number(v, "vy");
    if (!std::isfinite(x) || !std::isfinite(y) ||
        std::hypot(x, y) > number(o, "maximumObservedSpeedMps"))
        return "velocity_invalid";
    return "";
}
Json GpsSpeedLoop::update(double now, const Json &t, double bearing, double rate, double closing,
                          double applied, double policy, bool stop) {
    const auto reason = quality(now, t);
    if (!reason.empty() || (std::isfinite(at) && now < at)) {
        reset();
        return {{"active", false}, {"reason", reason.empty() ? "clock_reset" : reason}};
    }
    const auto &o = c["gpsSpeedClosedLoop"];
    const double limit = number(c, "relativeThrottleLimitPercent"), kp = number(o, "kp"),
                 ki = number(o, "ki");
    const double dt = std::isfinite(at) ? clamp((now - at) / 1000, .001, .25) : .1;
    at = now;
    if (!std::isfinite(good_since))
        good_since = now;
    const double yaw = number(t["attitude"], "yaw") * radians, b = bearing * radians,
                 vx = number(t["velocity"], "vx"), vy = number(t["velocity"], "vy");
    const double measured = vx * std::cos(yaw) + vy * std::sin(yaw),
                 sample = number(t.value("uwb", Json::object()), "measurementUpdatedAt");
    if (std::isfinite(sample) && sample != measurement_at) {
        const double target = vx * std::cos(yaw + b) + vy * std::sin(yaw + b) + rate;
        const double elapsed = std::isfinite(measurement_at)
                                   ? std::max(0.0, (sample - measurement_at) / 1000)
                                   : .1,
                     alpha = 1 - std::exp(-std::min(1.0, elapsed));
        target_radial = std::isfinite(target_radial)
                            ? target_radial + alpha * (target - target_radial)
                            : target;
        measurement_at = sample;
    }
    const double speed_limit =
        std::min(number(c, "maxSpeedMps"), c.value("commissioningMode", true)
                                               ? number(c, "commissioningSpeedMps")
                                               : number(c, "maxSpeedMps"));
    const double desired = stop ? 0
                                : clamp(std::isfinite(policy)
                                            ? policy
                                            : ((std::isfinite(target_radial) ? target_radial : 0) +
                                               closing) /
                                                  std::max(.3, std::cos(b)),
                                        0, speed_limit),
                 error = desired - measured;
    if (now - good_since < number(o, "acquireMs"))
        return {{"active", false}, {"reason", "gps_confirming"}};
    if (!active) {
        integral = clamp(applied - kp * error, -limit, limit);
        active = true;
    } else {
        const double candidate = clamp(integral + ki * error * dt, -limit, limit),
                     output = kp * error + candidate;
        if (!((output > limit && error > 0) || (output < 0 && error < 0)))
            integral = candidate;
    }
    request = clamp(kp * error + integral, 0, limit);
    return {{"active", true},
            {"reason", "gps_speed_feedback"},
            {"throttlePercent", request},
            {"measuredForwardSpeedMps", measured},
            {"desiredSpeedMps", desired},
            {"speedErrorMps", error}};
}
void GpsSpeedLoop::track_applied(double throttle, double dt) {
    if (active)
        integral =
            clamp(integral + (throttle - request) * dt, -number(c, "relativeThrottleLimitPercent"),
                  number(c, "relativeThrottleLimitPercent"));
}
} // namespace manta
