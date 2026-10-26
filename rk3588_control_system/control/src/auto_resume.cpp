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
    const auto u = t.value("uwb", Json::object());
    const double at = measurement_time(u), range = number(u, "distanceM");
    const auto id = resume_identity(u);
    // One complete, valid, fresh observation is sufficient. Raw mode's parser
    // owns completeness/replay checks; recovery adds no KF, motion or FOV gate.
    if (!transition_failure(now, t, pre).is_null() || !pre.value("safe", false) ||
        !u.value("online", false) || !u.value("fresh", false) ||
        !std::isfinite(at) || at > now || now - at > number(c, "uwb_timeout_ms", 700) ||
        !std::isfinite(range) || range <= number(c, "safety_distance_m", .3) ||
        !std::isfinite(number(u, "azimuthDeg"))) {
        clear_window();
        return false;
    }
    // Bind the first acquisition before asynchronous mode/arm ACKs so a
    // different tag cannot take over while a start is in progress.
    if (target.is_null() && !id.is_null()) target = id;
    first_at = last_at = at;
    window_target = id;
    samples = 1;
    return true;
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
