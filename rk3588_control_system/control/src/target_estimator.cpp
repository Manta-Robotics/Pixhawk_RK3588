#include "manta/prediction.hpp"
#include <set>

namespace manta {
bool fresh_time(double at, double now, double timeout) {
    return std::isfinite(at) && at > 0 && at <= now && now - at <= timeout;
}
Json prediction_config(const Json &raw) {
    Json c = {{"enabled", false},
              {"horizon_ms", 1000},
              {"wait_timeout_ms", 5000},
              {"pose_timeout_ms", 700},
              {"min_samples", 3},
              {"measurement_gap_ms", 450},
              {"measurement_noise_m", .12},
              {"velocity_uncertainty_mps", .2},
              {"acceleration_bound_mps2", .6},
              {"max_uncertainty_m", .5},
              {"max_target_speed_mps", 3},
              {"innovation_gate_m", .6},
              {"max_speed_mps", .35},
              {"max_throttle_percent", 15},
              {"max_steering_deg", 6},
              {"clearance_m", 1.2},
              {"braking_deceleration_mps2", .3},
              {"recovery_blend_ms", 800}};
    c.update(raw);
    return c;
}
Json target_identity(const Json &u) {
    std::set<std::string> tags;
    const auto ids = u.value("activeReceiverIds", Json::array());
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::find(ids.begin(), ids.end(), s["receiverId"]) != ids.end() &&
            std::isfinite(number(s, "tagId")))
            tags.insert(s["tagId"].dump());
    if (std::isfinite(number(u, "tagId")))
        tags.insert(u["tagId"].dump());
    std::string out;
    for (const auto &tag : tags) {
        if (!out.empty())
            out += ',';
        out += tag;
    }
    return out.empty() ? Json(nullptr) : Json(out);
}
std::string measurement_key(const Json &u) {
    std::vector<std::string> tokens;
    const auto ids = u.value("activeReceiverIds", Json::array());
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::find(ids.begin(), ids.end(), s["receiverId"]) != ids.end()) {
            if (number(s, "filterAdmissionVersion") == 1 &&
                std::isfinite(number(s, "measurementUpdatedAt")))
                tokens.push_back(s["receiverId"].dump() + ":accepted:" +
                                 s.value("acceptedFrameKey", s["measurementUpdatedAt"]).dump());
            else if (std::isfinite(number(s, "sequenceId")))
                tokens.push_back(s["receiverId"].dump() + ":" + s["sequenceId"].dump());
        }
    std::sort(tokens.begin(), tokens.end());
    std::string out;
    for (const auto &token : tokens) {
        if (!out.empty())
            out += '|';
        out += token;
    }
    return out.empty() ? "time:" + u.value("measurementUpdatedAt", Json(nullptr)).dump() : out;
}
double sensor_uncertainty(const Json &u) {
    double result = 0;
    const auto ids = u.value("activeReceiverIds", Json::array());
    for (const auto &s : u.value("sensors", Json::array()))
        if (ids.empty() || std::find(ids.begin(), ids.end(), s["receiverId"]) != ids.end())
            result = std::max(result, number(s, "filterUncertaintyM", 0));
    return result;
}
static std::pair<bool, bool> admission_contract(const Json &u, double now, double gap) {
    const auto ids = u.value("activeReceiverIds", Json::array());
    bool versioned = false, selected = false, confirmed = true;
    double latest = -INFINITY;
    for (const auto &s : u.value("sensors", Json::array())) {
        versioned |= number(s, "filterAdmissionVersion") == 1;
        if (std::find(ids.begin(), ids.end(), s["receiverId"]) == ids.end())
            continue;
        selected = true;
        confirmed &= number(s, "filterAdmissionVersion") == 1 &&
                     s.value("measurementStatus", std::string()) == "trusted" &&
                     s.value("valid", Json(nullptr)) != false &&
                     fresh_time(number(s, "measurementUpdatedAt"), now, gap) &&
                     fresh_time(number(s, "measurementSampleAt", number(s, "measurementUpdatedAt")), now, gap) &&
                     s.value("receivedMeasurementAt", Json(nullptr)) ==
                         s.value("measurementUpdatedAt", Json(nullptr)) &&
                     std::isfinite(number(s, "filterUncertaintyM")) &&
                     number(s, "filterUncertaintyM") >= 0;
        latest = std::max(latest, number(s, "measurementUpdatedAt", -INFINITY));
    }
    return {versioned, versioned && selected && confirmed &&
                           u.value("measurementStatus", std::string()) == "trusted" &&
                           latest == number(u, "measurementUpdatedAt")};
}
void TargetEstimator::reset() {
    last = last_pose = recovery = recovery_anchor = nullptr;
    last_seen_at = loss_at = recovered_at = motion_at = motion_trusted_at = missing;
    last_seen_token.clear();
    raw_window.reset();
    motion_history.clear();
    motion_velocity = {0, 0};
    motion_diagnostic = {
        {"trusted", false}, {"reason", "warming_up"}, {"samples", 0}, {"spanMs", 0}};
}
Json TargetEstimator::pose(double now, const Json &t) {
    const auto a = t.value("attitude", Json::object()), v = t.value("velocity", Json::object()),
               p = t.value("position", Json::object()), e = t.value("ekf", Json::object());
    const double yaw = number(a, "yaw"), timeout = number(config, "pose_timeout_ms");
    if (calibration.value("execution_mode", std::string()) == "relative_imu") {
        if (!std::isfinite(yaw) || !fresh_time(number(a, "updatedAt"), now, timeout))
            return nullptr;
        last_pose = {{"yaw", yaw}, {"vx", 0},
                     {"vy", 0},    {"position", {{"x", 0}, {"y", 0}}},
                     {"at", now},  {"relativeOnly", true}};
        return last_pose;
    }
    const double vx = number(v, "vx"), vy = number(v, "vy");
    if (!std::isfinite(yaw) || !std::isfinite(vx) || !std::isfinite(vy) ||
        !e.value("healthy", false) || !fresh_time(number(a, "updatedAt"), now, timeout) ||
        !fresh_time(number(p, "updatedAt"), now, timeout) ||
        !fresh_time(number(e, "updatedAt"), now, timeout))
        return nullptr;
    const double dt = last_pose.is_object() ? (now - number(last_pose, "at")) / 1000 : 0;
    if (dt < 0 || dt * 1000 > timeout)
        return nullptr;
    const Vec position =
        last_pose.is_object()
            ? vector_value(last_pose["position"]) +
                  Vec{vx + number(last_pose, "vx"), vy + number(last_pose, "vy")} * (.5 * dt)
            : Vec{0, 0};
    last_pose = {
        {"yaw", yaw}, {"vx", vx}, {"vy", vy}, {"position", vector_json(position)}, {"at", now}};
    return last_pose;
}
Json TargetEstimator::sample(const Json &u, const Json &pose, double now) const {
    const double at = number(u, "measurementSampleAt", number(u, "measurementUpdatedAt")), range = number(u, "distanceM"),
                 azimuth = number(u, "azimuthDeg");
    if (!pose.is_object() || !u.value("online", false) || !u.value("fresh", false) ||
        !std::isfinite(range) || range <= 0 || !std::isfinite(azimuth) ||
        !fresh_time(at, now, number(config, "measurement_gap_ms")))
        return nullptr;
    const double bearing = wrap(azimuth * number(calibration, "bearing_sign", -1) *
                                    number(calibration, "bearing_scale", .8148) +
                                number(calibration, "bearing_offset_deg", -4.816));
    const Vec body = {range * std::cos(bearing * radians), range * std::sin(bearing * radians)},
              relative = rotate_vector(body, number(u, "measurementHeadingDeg", number(pose, "yaw")) * radians);
    const auto position = vector_value(pose["position"]) -
                          Vec{number(pose, "vx"), number(pose, "vy")} * ((now - at) / 1000) +
                          relative;
    return {{"at", at},
            {"token", measurement_key(u)},
            {"id", target_identity(u)},
            {"measurementUncertaintyM", sensor_uncertainty(u)},
            {"position", vector_json(position)}};
}
Vec TargetEstimator::projected(const Json &sample, double now) const {
    return vector_value(sample["position"]) +
           vector_value(sample["velocity"]) * std::max(0.0, (now - number(sample, "at")) / 1000);
}
Vec TargetEstimator::observe_motion(const Json &sample) {
    const auto n = [&](const char *k) { return number(config, k); };
    const double at = number(sample, "at"),
                 minimum_span = std::max(n("measurement_gap_ms"), n("recovery_blend_ms")),
                 dt = std::isfinite(motion_at) ? std::max(0.0, (at - motion_at) / 1000) : 0;
    motion_at = at;
    if (!motion_history.empty()) {
        const auto &previous = motion_history.back();
        if (at - number(previous, "at") > n("measurement_gap_ms") ||
            (vector_value(sample["position"]) - vector_value(previous["position"])).norm() >
                n("innovation_gate_m") +
                    n("max_target_speed_mps") * (at - number(previous, "at")) / 1000) {
            motion_history.clear();
            motion_velocity = {0, 0};
            motion_trusted_at = missing;
        }
    }
    std::erase_if(motion_history,
                  [&](const Json &p) { return at - number(p, "at") > 2 * minimum_span; });
    motion_history.push_back(sample);
    const auto &points = motion_history;
    const double span = number(points.back(), "at") - number(points.front(), "at"),
                 minimum_samples = 2 * n("min_samples");
    const auto median = [](std::vector<double> values) {
        std::sort(values.begin(), values.end());
        const auto n = values.size();
        return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
    };
    std::string reason = "velocity_warming_up";
    std::vector<Json> inliers;
    if (points.size() >= minimum_samples && span >= minimum_span) {
        std::vector<double> sx, sy;
        for (size_t i = 0; i + 1 < points.size(); ++i)
            for (size_t j = i + 1; j < points.size(); ++j) {
                const double gap = (number(points[j], "at") - number(points[i], "at")) / 1000;
                if (gap * 1000 < minimum_span * .75)
                    continue;
                const auto slope =
                    (vector_value(points[j]["position"]) - vector_value(points[i]["position"])) /
                    gap;
                sx.push_back(slope.x);
                sy.push_back(slope.y);
            }
        if (!sx.empty()) {
            const Vec velocity = {median(sx), median(sy)};
            std::vector<double> px, py;
            for (const auto &p : points) {
                const auto projected =
                    vector_value(p["position"]) - velocity * ((number(p, "at") - at) / 1000);
                px.push_back(projected.x);
                py.push_back(projected.y);
            }
            const Vec position = {median(px), median(py)};
            for (const auto &p : points)
                if ((vector_value(p["position"]) - position -
                     velocity * ((number(p, "at") - at) / 1000))
                        .norm() <= 2 * n("measurement_noise_m"))
                    inliers.push_back(p);
            if (inliers.size() >= minimum_samples && number(inliers.back(), "at") == at &&
                number(inliers.back(), "at") - number(inliers.front(), "at") >= minimum_span &&
                velocity.norm() <= n("max_target_speed_mps")) {
                const auto delta = velocity - motion_velocity;
                const double magnitude = delta.norm(),
                             limit = n("acceleration_bound_mps2") *
                                     std::min(dt, n("measurement_gap_ms") / 1000);
                motion_velocity =
                    motion_velocity + delta * (magnitude > limit ? limit / magnitude : 1);
                motion_trusted_at = at;
                reason = "velocity_confirmed";
            } else
                reason = "velocity_inconsistent";
        }
    }
    const bool trusted = fresh_time(motion_trusted_at, at, n("measurement_gap_ms"));
    motion_diagnostic = {{"trusted", trusted},
                         {"reason", reason},
                         {"samples", points.size()},
                         {"inliers", inliers.size()},
                         {"spanMs", span}};
    return trusted ? motion_velocity : Vec{0, 0};
}
void TargetEstimator::accept(const Json &sample, const Json &previous, double recovered_samples) {
    const auto prior = previous.is_object() ? previous : last;
    const double dt = prior.is_object() ? (number(sample, "at") - number(prior, "at")) / 1000 : 0;
    const auto velocity = observe_motion(sample);
    last = sample;
    last.update({{"velocity", vector_json(velocity)},
                 {"velocityTrusted", motion_diagnostic["trusted"]},
                 {"samples",
                  std::isfinite(recovered_samples) ? recovered_samples
                  : prior.is_object() && dt > 0 && dt * 1000 <= number(config, "measurement_gap_ms")
                      ? number(prior, "samples") + 1
                      : 1}});
}
Json TargetEstimator::update(double now, const Json &t) {
    const auto n = [&](const char *key) { return number(config, key); };
    const auto terminal = [&](const std::string &reason) {
        return Json{{"terminal", true},
                    {"reason", reason},
                    {"source", "unavailable"},
                    {"evaluatedAt", now}};
    };
    if (std::isfinite(loss_at) && now - loss_at >= n("wait_timeout_ms"))
        return terminal("prediction_wait_timeout");
    const auto current_pose = pose(now, t), u = t.value("uwb", Json::object());
    auto current = sample(u, current_pose, now);
    const auto [versioned, confirmed] = admission_contract(u, now, n("measurement_gap_ms"));
    std::vector<Json> raw_sensors;
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::isfinite(number(s, "receivedMeasurementAt")) ||
            std::isfinite(number(s, "rawRejectedAt")))
            raw_sensors.push_back(s);
    const bool raw_recovery = !versioned && std::isfinite(loss_at) && raw_sensors.size() == 1 &&
                              current_pose.is_object() && last.is_object();
    if (current.is_object() && last.is_object() && !last["id"].is_null() &&
        !current["id"].is_null() && last["id"] != current["id"])
        return terminal("target_identity_changed");
    if (!versioned && raw_sensors.size() == 1 && current_pose.is_object() && last.is_object())
        raw_window.observe(now, t, *this, recovery_anchor.is_object() ? recovery_anchor : last,
                           raw_recovery);
    const bool new_sample = !raw_recovery && (!versioned || confirmed) && current.is_object() &&
                            current["token"] != last_seen_token &&
                            (!std::isfinite(last_seen_at) || number(current, "at") > last_seen_at);
    Json rejection = nullptr;
    if (new_sample) {
        last_seen_at = number(current, "at");
        last_seen_token = current["token"];
        if (last.is_object() && !last["id"].is_null() && !current["id"].is_null() &&
            last["id"] != current["id"])
            return terminal("target_identity_changed");
        const auto reference = recovery.is_object() ? recovery : last;
        const double dt = reference.is_object()
                              ? (number(current, "at") - number(reference, "at")) / 1000
                              : 0,
                     gap = dt * 1000;
        const auto expected = reference.is_object() ? projected(reference, number(current, "at"))
                                                    : vector_value(current["position"]);
        const double allowed = n("innovation_gate_m") +
                               n("velocity_uncertainty_mps") * std::max(0.0, dt) +
                               .5 * n("acceleration_bound_mps2") * squared(std::max(0.0, dt));
        if (confirmed) {
            const bool recovering = std::isfinite(loss_at);
            current["admissionVersion"] = 1;
            accept(current);
            loss_at = missing;
            recovery = recovery_anchor = nullptr;
            if (recovering)
                recovered_at = now;
        } else if (reference.is_object() &&
                   (gap <= 0 || (vector_value(current["position"]) - expected).norm() > allowed ||
                    (gap <= n("measurement_gap_ms") &&
                     (vector_value(current["position"]) - vector_value(reference["position"]))
                             .norm() >
                         n("measurement_noise_m") * 2 + n("max_target_speed_mps") * dt))) {
            rejection = "measurement_jump";
            recovery = nullptr;
        } else if (std::isfinite(loss_at)) {
            const auto previous = recovery;
            const double samples = previous.is_object() && gap <= n("measurement_gap_ms")
                                       ? number(previous, "samples") + 1
                                       : 1;
            Vec velocity =
                previous.is_object() && dt > 0
                    ? (vector_value(current["position"]) - vector_value(previous["position"])) / dt
                : last.is_object() ? vector_value(last["velocity"])
                                   : Vec{0, 0};
            if (velocity.norm() > n("max_target_speed_mps"))
                velocity = velocity * (n("max_target_speed_mps") / velocity.norm());
            recovery = current;
            recovery.update({{"velocity", vector_json(velocity)}, {"samples", samples}});
            if (samples >= n("min_samples")) {
                accept(recovery, last, samples);
                recovery = recovery_anchor = nullptr;
                loss_at = missing;
                recovered_at = now;
            }
        } else
            accept(current);
    }
    const bool observation_fresh = (!versioned || confirmed) && current.is_object() &&
                                   rejection.is_null() && last.is_object() &&
                                   current["token"] == last["token"] &&
                                   fresh_time(number(last, "at"), now, n("measurement_gap_ms"));
    if (!observation_fresh && !std::isfinite(loss_at)) {
        loss_at = now;
        recovery_anchor = last;
    }
    if (std::isfinite(loss_at) && now - loss_at >= n("wait_timeout_ms"))
        return terminal("prediction_wait_timeout");
    if (!last.is_object() || !current_pose.is_object())
        return {{"usable", false},
                {"source", "unavailable"},
                {"evaluatedAt", now},
                {"reason", current_pose.is_object() ? "prediction_warming_up"
                                                    : "prediction_pose_unavailable"},
                {"lastMeasuredAt", last.is_object() ? last["at"] : Json(nullptr)}};
    const double age = now - number(last, "at"),
                 measurement_uncertainty =
                     std::max({n("measurement_noise_m"), number(last, "measurementUncertaintyM", 0),
                               sensor_uncertainty(u)});
    const double uncertainty = measurement_uncertainty +
                               n("velocity_uncertainty_mps") * age / 1000 +
                               .5 * n("acceleration_bound_mps2") * squared(age / 1000);
    const auto target = projected(last, now),
               body = rotate_vector(target - vector_value(current_pose["position"]),
                                    -number(current_pose, "yaw") * radians),
               velocity = vector_value(last["velocity"]);
    const bool recovering = observation_fresh && std::isfinite(recovered_at) &&
                            now - recovered_at < n("recovery_blend_ms"),
               relative = current_pose.value("relativeOnly", false);
    const std::string source =
        observation_fresh ? recovering ? "recovering" : "measured" : "predicted";
    const bool measurement_confirmed =
        observation_fresh &&
        (raw_sensors.empty() || u.value("measurementStatus", std::string()) == "trusted") &&
        std::isfinite(measurement_uncertainty) && measurement_uncertainty >= 0;
    const bool usable =
        rejection.is_null() &&
        (measurement_confirmed ? age >= 0 && age <= n("measurement_gap_ms")
                               : !observation_fresh && age >= 0 && age <= n("horizon_ms") &&
                                     uncertainty <= n("max_uncertainty_m") &&
                                     number(last, "samples") >= n("min_samples"));
    return {{"usable", usable},
            {"source", source},
            {"evaluatedAt", now},
            {"lastMeasuredAt", last["at"]},
            {"predictionAgeMs", age},
            {"uncertaintyM", uncertainty},
            {"measurementUncertaintyM", measurement_uncertainty},
            {"measurementConfirmed", measurement_confirmed},
            {"admissionVersion", last.value("admissionVersion", 0)},
            {"positionAdmission", versioned
                                      ? measurement_confirmed ? "filter_admitted" : "await_filter"
                                      : "legacy_window"},
            {"inputMeasurementStatus", u.value("measurementStatus", std::string("unknown"))},
            {"inputRejectionReason", u.value("rejectionReason", Json(nullptr))},
            {"relativeBodyM", vector_json(body)},
            {"targetPositionNedM", relative ? Json(nullptr) : vector_json(target)},
            {"targetVelocityNedMps", relative ? Json(nullptr) : vector_json(velocity)},
            {"relativeVelocityHeadingMps", relative ? vector_json(velocity) : Json(nullptr)},
            {"referenceFrame", relative ? "yaw_stabilized_relative" : "local_ned"},
            {"relativeClosingSpeedMps",
             relative ? Json(std::max(0.0, -(target.x * velocity.x + target.y * velocity.y) /
                                               std::max(.01, target.norm())))
                      : Json(nullptr)},
            {"boatSpeedMps",
             relative ? Json(nullptr)
                      : Json(std::hypot(number(current_pose, "vx"), number(current_pose, "vy")))},
            {"samples", last["samples"]},
            {"velocityEstimate", motion_diagnostic},
            {"recoverySamples", recovery.is_object() ? recovery["samples"] : Json(0)},
            {"independentRecovery", raw_window.diagnostic},
            {"reacquisitionFit", raw_window.diagnostic.value("reason", std::string()) == "confirmed"
                                     ? raw_window.fit_result
                                     : Json(nullptr)},
            {"reacquiring", std::isfinite(loss_at) && std::isfinite(last_seen_at) &&
                                last_seen_at > number(last, "at")},
            {"recoveryScale",
             recovering ? clamp((now - recovered_at) / n("recovery_blend_ms"), 0, 1) : 1},
            {"reason",
             rejection.is_string() ? rejection : Json(!usable ? "prediction_limit" : source)}};
}
} // namespace manta
