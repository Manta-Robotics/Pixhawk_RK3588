#include "manta/recovery.hpp"
#include <set>

namespace manta {
namespace {
Json resume_identity(const Json &u) {
    std::set<std::string> tags;
    const auto active = u.value("activeReceiverIds", Json::array());
    auto add = [&](const Json &j) {
        if (!j.is_null())
            tags.insert(j.is_string() ? j.get<std::string>() : j.dump());
    };
    for (const auto &s : u.value("sensors", Json::array()))
        if (active.empty() || std::find(active.begin(), active.end(),
                                        s.value("receiverId", Json(nullptr))) != active.end())
            add(s.value("tagId", Json(nullptr)));
    add(u.value("tagId", Json(nullptr)));
    if (tags.empty())
        return nullptr;
    std::string out;
    for (const auto &tag : tags) {
        if (!out.empty())
            out += ',';
        out += tag;
    }
    return out;
}
} // namespace
void AutoResume::clear_window() {
    samples = 0;
    first_at = last_at = missing;
    point = {};
    window_target = nullptr;
}
void AutoResume::cancel() {
    requested = pending = attempting = false;
    target = last_stop_reason = nullptr;
    resume_at = missing;
    clear_window();
}
void AutoResume::search() {
    cancel();
    requested = pending = true;
    last_stop_reason = "uwb_stale";
}
void AutoResume::started(const Json &t, double now, bool automatic) {
    requested = enabled;
    pending = attempting = false;
    const auto id = resume_identity(t.value("uwb", Json::object()));
    if (!id.is_null())
        target = id;
    resume_at = automatic ? now : missing;
    clear_window();
}
void AutoResume::stopped(const std::string &reason) {
    resume_at = missing;
    clear_window();
    attempting = false;
    last_stop_reason = reason;
    pending =
        enabled && requested && (reason == "prediction_wait_timeout" || reason == "uwb_stale");
    if (!pending)
        requested = false;
}
void AutoResume::attempt_failed(const std::string &reason) {
    const std::set<std::string> transient = {
        "uwb_stale",       "auto_resume_signal_timeout", "recovered_uwb_timeout", "imu_stale",
        "imu_invalid",     "acro_velocity_unavailable",  "gps_fix_invalid",       "ekf_unhealthy",
        "navigation_stale"};
    if (!transient.contains(reason)) {
        cancel();
        return;
    }
    attempting = false;
    clear_window();
}
Json AutoResume::transition_failure(double now, const Json &t, const Json &pre) const {
    const auto a = t.value("attitude", Json::object()), v = t.value("velocity", Json::object()),
               u = t.value("uwb", Json::object());
    const auto fresh = [&](const Json &j) {
        const double at = number(j, "updatedAt");
        return std::isfinite(at) && at <= now &&
               now - at <= number(c, "navigation_timeout_ms", 700);
    };
    if (!std::isfinite(number(a, "yaw")) || !fresh(a))
        return "imu_stale";
    if (c.value("execution_mode", Json(nullptr)) == "acro") {
        const auto ekf = t.value("ekf", Json::object()),
                   acro = t.value("acroControl", Json::object());
        if (!fresh(t.value("position", Json::object())) || !fresh(ekf) ||
            !ekf.value("healthy", false) || !std::isfinite(number(v, "vx")) ||
            !std::isfinite(number(v, "vy")) ||
            number(t.value("gps", Json::object()), "fixType", 0) <
                number(c, "minimum_gps_fix_type", 2))
            return "navigation_stale";
        if (!acro.value("ready", false) || !fresh(acro))
            return "acro_parameters_not_ready";
    }
    if (!target.is_null() && resume_identity(u) != target)
        return "target_identity_changed";
    auto near = [&](double d) {
        return std::isfinite(d) && d >= 0 && d <= number(c, "safety_distance_m", .3);
    };
    if (near(number(u, "distanceM")))
        return "safety_distance";
    for (const auto &s : u.value("sensors", Json::array())) {
        double at = number(s, "rawDistanceUpdatedAt", 0);
        if (at == 0)
            at = number(s, "receivedMeasurementAt");
        if (std::isfinite(at) && at <= now && now - at <= 450 && near(number(s, "rawDistanceM")))
            return "safety_distance";
    }
    if (!pre.value("safe", false) && pre.value("reason", std::string()) != "uwb_stale")
        return pre.value("reason", Json(nullptr));
    return nullptr;
}
bool AutoResume::observe(double now, const Json &t, const Json &pre, bool during) {
    if (!pending || (attempting && !during))
        return false;
    const auto u = t.value("uwb", Json::object()), a = t.value("attitude", Json::object()),
               active = u.value("activeReceiverIds", Json::array());
    bool all = true;
    int count = 0;
    for (const auto &s : u.value("sensors", Json::array()))
        if (active.empty() || std::find(active.begin(), active.end(),
                                        s.value("receiverId", Json(nullptr))) != active.end()) {
            ++count;
            all &= s.value("measurementStatus", std::string()) == "trusted";
        }
    const bool trusted =
        u.value("measurementStatus", std::string()) == "trusted" || (count > 0 && all);
    const double at = measurement_time(u), range = number(u, "distanceM"),
                 bearing = wrap(number(u, "azimuthDeg") * number(c, "bearing_sign", 1) *
                                    number(c, "bearing_scale", 1) +
                                number(c, "bearing_offset_deg", 0)),
                 gap = number(p, "measurement_gap_ms", 450);
    const auto id = resume_identity(u);
    if (!transition_failure(now, t, pre).is_null()) {
        clear_window();
        return false;
    }
    const double yaw = number(a, "yaw"), pose_at = number(a, "updatedAt");
    if (!pre.value("safe", false) || !trusted || !u.value("online", false) ||
        !u.value("fresh", false) || !std::isfinite(at) || at > now || now - at > gap ||
        !std::isfinite(range) || range <= number(c, "safety_distance_m", .3) ||
        !std::isfinite(bearing) || std::abs(bearing) > number(c, "field_of_view_half_deg", 45) ||
        !std::isfinite(yaw) || !std::isfinite(pose_at) || pose_at > now ||
        now - pose_at > number(c, "navigation_timeout_ms", 700) ||
        (!target.is_null() && id != target)) {
        if (!std::isfinite(last_at) || now - last_at > gap ||
            std::abs(bearing) > number(c, "field_of_view_half_deg", 45))
            clear_window();
        return false;
    }
    if (std::isfinite(last_at) && at <= last_at)
        return false;
    if (std::isfinite(last_at) && id != window_target)
        clear_window();
    const Vec next{range * std::cos((yaw + bearing) * radians),
                   range * std::sin((yaw + bearing) * radians)};
    const double dt = std::isfinite(last_at) ? (at - last_at) / 1000 : 0;
    const auto v = t.value("velocity", Json::object());
    const double vx = number(v, "vx"), vy = number(v, "vy"),
                 speed = std::isfinite(vx) && std::isfinite(vy) ? std::hypot(vx, vy) : 3;
    if (std::isfinite(last_at) &&
        (at - last_at > gap ||
         (next - point).norm() > number(p, "innovation_gate_m", .6) +
                                     (number(p, "max_target_speed_mps", 3) + speed) * dt))
        clear_window();
    if (!std::isfinite(first_at))
        first_at = at;
    last_at = at;
    point = next;
    window_target = id;
    ++samples;
    return samples >= number(p, "min_samples", 3) && at - first_at >= 180;
}
bool AutoResume::begin_attempt() {
    if (!pending || attempting)
        return false;
    attempting = true;
    return true;
}
Json AutoResume::ramp(Json result, double now) {
    if (!std::isfinite(resume_at))
        return result;
    const double scale =
        clamp((now - resume_at) / std::max(1.0, number(p, "recovery_blend_ms", 800)), 0, 1);
    if (scale >= 1) {
        resume_at = missing;
        return result;
    }
    result.update({{"status", "recovering"},
                   {"reason", "auto_resume_recovering"},
                   {"autoResumeScale", scale},
                   {"throttlePercent", number(result, "throttlePercent", 0) * scale}});
    for (const auto *key : {"forwardSpeedMps", "desiredSpeedMps"})
        if (std::isfinite(number(result, key)))
            result[key] = number(result, key) * scale;
    if (result.contains("velocityNed") && result["velocityNed"].is_object()) {
        result["velocityNed"]["x"] = number(result["velocityNed"], "x") * scale;
        result["velocityNed"]["y"] = number(result["velocityNed"], "y") * scale;
    }
    return result;
}
Json AutoResume::snapshot() const {
    return {{"followRequested", requested},
            {"autoResumePending", pending},
            {"autoResumeSamples", samples},
            {"autoResumeAttempting", attempting},
            {"lastStopReason", last_stop_reason}};
}
} // namespace manta
