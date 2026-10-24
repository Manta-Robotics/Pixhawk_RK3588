#include "manta/recovery.hpp"

namespace manta {
namespace {
bool fresh(double at, double now, double age) {
    return std::isfinite(at) && at > 0 && at <= now && now - at <= age;
}
std::string sensor_id(const Json &s) {
    const auto str = [](const Json &v) {
        return v.is_null() ? std::string() : v.is_string() ? v.get<std::string>() : v.dump();
    };
    return str(s.value("receiverId", Json(nullptr))) + ":" + str(s.value("tagId", Json(nullptr)));
}
Vec displacement(const Json &pos, const Json &anchor) {
    return {(number(pos, "lat") - number(anchor, "lat")) * radians * 6378137,
            wrap(number(pos, "lon") - number(anchor, "lon")) * radians * 6378137 *
                std::cos(number(anchor, "lat") * radians)};
}
} // namespace
void DriftHold::reset() {
    anchor = last_tracking = certificate = target_id = nullptr;
    bearings.clear();
    creep_speed = 0;
    creep_at = last_progress = missing;
    far_pursuit = false;
    nmpc.reset();
}
void DriftHold::track(double now, const Json &t, const Json &result) {
    nmpc.observe(now, t, result);
    anchor = nullptr;
    far_pursuit = false;
    const auto a = t.value("attitude", Json::object()), v = t.value("velocity", Json::object()),
               pos = t.value("position", Json::object()), p = c.value("prediction", Json::object());
    if (result.value("safe", false) && std::isfinite(number(result, "bearingDeg")) &&
        std::isfinite(number(a, "yaw")))
        last_tracking = {{"at", now},
                         {"heading", wrap(number(a, "yaw") + number(result, "bearingDeg"))},
                         {"range", result.value("distanceM", Json(nullptr))}};
    const double psi = number(a, "yaw", 0) * radians,
                 forward = std::isfinite(number(v, "vx")) && std::isfinite(number(v, "vy"))
                               ? number(v, "vx") * std::cos(psi) + number(v, "vy") * std::sin(psi)
                               : 0;
    creep_speed =
        clamp(forward, 0, std::min(number(p, "max_speed_mps", .35), number(c, "max_speed_mps", 2)));
    creep_at = last_progress = now;
    const auto sensors = t.value("uwb", Json::object()).value("sensors", Json::array());
    if (!sensors.empty())
        target_id = sensor_id(sensors[0]);
    const auto e = result.value("targetEstimate", Json(nullptr));
    if (e.is_object() && e.value("measurementConfirmed", false) &&
        std::isfinite(number(result, "distanceM")) && std::isfinite(number(e, "uncertaintyM")) &&
        number(certificate, "measurementAt") != number(e, "lastMeasuredAt") &&
        std::isfinite(number(pos, "lat")) && std::isfinite(number(pos, "lon")))
        certificate = {{"at", now},
                       {"measurementAt", e["lastMeasuredAt"]},
                       {"range", result["distanceM"]},
                       {"uncertainty", e["uncertaintyM"]},
                       {"lat", pos["lat"]},
                       {"lon", pos["lon"]}};
}
Json DriftHold::observe_bearing(double now, const Json &t) {
    const auto p = c.value("prediction", Json::object()), u = t.value("uwb", Json::object()),
               a = t.value("attitude", Json::object());
    const auto no = [](const std::string &reason) {
        return Json{{"trusted", false}, {"reason", reason}, {"hits", 0}};
    };
    std::vector<Json> sensors;
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::isfinite(number(s, "receivedMeasurementAt")) ||
            std::isfinite(number(s, "rawRejectedAt")))
            sensors.push_back(s);
    if (sensors.size() != 1)
        return no("one_raw_receiver_required");
    const auto &s = sensors[0];
    const double at =
        std::max(number(s, "receivedMeasurementAt", 0), number(s, "rawRejectedAt", 0));
    if (quarantined(s)) {
        bearings.clear();
        return no("occlusion_quarantine");
    }
    if (!fresh(at, now, number(p, "measurement_gap_ms", 450)) || !s.value("valid", true) ||
        at != number(s, "receivedMeasurementAt"))
        return no("raw_invalid_or_stale");
    const double az =
        number(s, "rawAzimuthDeg",
               u.value("measurementStatus", std::string()) == "trusted" ? number(u, "azimuthDeg")
                                                                        : missing);
    if (!std::isfinite(az) || std::abs(az) > 180)
        return no("angle_invalid");
    const double bearing = wrap((az + number(s, "mountYawDeg", 0)) * number(c, "bearing_sign", -1) *
                                    number(c, "bearing_scale", .8148) +
                                number(c, "bearing_offset_deg", -4.816));
    if (std::abs(bearing) > 45)
        return no("outside_fov");
    const double heading =
        wrap(number(a, "yaw") + bearing - number(a, "yawRateDegS", 0) * (now - at) / 1000);
    const auto id = sensor_id(s);
    if (!target_id.is_null() && target_id != id) {
        bearings.clear();
        certificate = nullptr;
        return no("identity_changed");
    }
    std::erase_if(bearings, [&](const Json &q) { return now - number(q, "at") > 700; });
    if (!bearings.empty() && bearings.back()["id"] != id) {
        bearings.clear();
        certificate = nullptr;
        return no("identity_changed");
    }
    if (bearings.empty() || at > number(bearings.back(), "at")) {
        bearings.push_back({{"at", at},
                            {"id", id},
                            {"heading", heading},
                            {"range", s.value("rawDistanceM", Json(nullptr))}});
        if (bearings.size() > 7)
            bearings.erase(bearings.begin());
    }
    if (bearings.size() < 3 || number(bearings.back(), "at") != at)
        return no("bearing_warming_up");
    const size_t required = std::max(3.0, std::ceil(bearings.size() * .6));
    const double origin = number(bearings[0], "heading");
    struct Model {
        double rate, heading, intercept;
        size_t hits;
    };
    const auto fit_heading = [&](const std::vector<Json> &points) -> std::optional<Model> {
        double mt = 0, mh = 0;
        for (const auto &q : points) {
            mt += (number(q, "at") - at) / 1000;
            mh += origin + wrap(number(q, "heading") - origin);
        }
        mt /= points.size();
        mh /= points.size();
        double den = 0, num = 0;
        for (const auto &q : points) {
            const double dt = (number(q, "at") - at) / 1000 - mt;
            den += dt * dt;
            num += dt * (origin + wrap(number(q, "heading") - origin) - mh);
        }
        if (den == 0)
            return {};
        const double rate = num / den, intercept = mh - rate * mt;
        return Model{rate, wrap(intercept), intercept, points.size()};
    };
    const auto error = [&](const Model &m, const Json &q) {
        return std::abs(
            wrap(number(q, "heading") - m.intercept - m.rate * (number(q, "at") - at) / 1000));
    };
    std::vector<Model> models;
    for (size_t i = 0; i + 2 < bearings.size(); ++i)
        for (size_t j = i + 1; j + 1 < bearings.size(); ++j)
            for (size_t k = j + 1; k < bearings.size(); ++k) {
                const auto initial = fit_heading({bearings[i], bearings[j], bearings[k]});
                if (!initial || std::abs(initial->rate) > 30)
                    continue;
                std::vector<Json> inliers;
                for (const auto &q : bearings)
                    if (error(*initial, q) <= 3)
                        inliers.push_back(q);
                if (inliers.size() < required || number(inliers.back(), "at") != at ||
                    at - number(inliers[0], "at") < 180)
                    continue;
                const auto model = fit_heading(inliers);
                if (!model || std::abs(model->rate) > 30 ||
                    std::any_of(inliers.begin(), inliers.end(),
                                [&](const Json &q) { return error(*model, q) > 3; }))
                    continue;
                models.push_back(*model);
            }
    std::stable_sort(models.begin(), models.end(), [&](const Model &x, const Model &y) {
        return x.hits != y.hits ? x.hits > y.hits
                                : error(x, bearings.back()) < error(y, bearings.back());
    });
    if (models.empty())
        return no("bearing_inconsistent");
    const auto &model = models[0];
    target_id = id;
    return {{"trusted", true},
            {"reason", "raw_heading_confirmed"},
            {"at", at},
            {"bearingDeg", bearing},
            {"headingDeg", wrap(model.heading + model.rate * (now - at) / 1000)},
            {"rateDegS", model.rate},
            {"hits", model.hits}};
}
void DriftHold::refresh_range(double now, const Json &t) {
    const auto u = t.value("uwb", Json::object()), sensors = u.value("sensors", Json::array()),
               p = c.value("prediction", Json::object());
    if (sensors.empty())
        return;
    const auto &s = sensors[0];
    const double at = number(u, "measurementUpdatedAt"), range = number(u, "distanceM"),
                 uncertainty = number(s, "filterUncertaintyM");
    if (quarantined(s) || u.value("measurementStatus", std::string()) != "trusted" ||
        s.value("measurementStatus", std::string()) != "trusted" || !u.value("fresh", false) ||
        !fresh(at, now, number(p, "measurement_gap_ms", 450)) ||
        number(s, "receivedMeasurementAt") != at || !std::isfinite(range) ||
        !std::isfinite(uncertainty) || uncertainty < 0 ||
        number(certificate, "measurementAt") == at)
        return;
    std::vector<Point> points;
    for (const auto &q : bearings) {
        const double r = number(q, "range");
        if (std::isfinite(r) && r >= .15 && r <= 20 && at - number(q, "at") <= 500)
            points.push_back({number(q, "at"), {r, 0}, 0});
    }
    if (points.size() < 3 || points.back().at != at || at - points.front().at < 180)
        return;
    const auto model = fit(points, at);
    if (!model)
        return;
    const double slope = model->velocity.x, predicted = model->position.x,
                 gate = 2 * number(p, "measurement_noise_m", .12);
    const auto v = t.value("velocity", Json::object());
    const double own = std::hypot(number(v, "vx"), number(v, "vy"));
    if (std::abs(slope) > number(p, "max_target_speed_mps", 3) + own ||
        std::any_of(points.begin(), points.end(),
                    [&](const Point &q) {
                        return std::abs(q.z.x - predicted - slope * (q.at - at) / 1000) > gate;
                    }) ||
        std::abs(range - predicted) > gate)
        return;
    const double bounded = std::min(range, points.back().z.x);
    if (certificate.is_object() &&
        std::abs(bounded - number(certificate, "range")) >
            number(p, "innovation_gate_m", .6) + (number(p, "max_target_speed_mps", 3) + own) *
                                                     (now - number(certificate, "at")) / 1000)
        return;
    const auto pos = t.value("position", Json::object());
    certificate = {{"at", now},
                   {"measurementAt", at},
                   {"range", bounded},
                   {"uncertainty", std::max(number(p, "measurement_noise_m", .12), uncertainty)},
                   {"lat", pos.value("lat", Json(nullptr))},
                   {"lon", pos.value("lon", Json(nullptr))}};
}
Json DriftHold::accepted_prediction(double now, const Json &result, double forward,
                                    double own_speed) const {
    const auto e = result.value("targetEstimate", Json(nullptr)),
               p = c.value("prediction", Json::object());
    if (!certificate.is_object() || !e.is_object())
        return nullptr;
    const auto body = e.value("relativeBodyM", Json::object());
    if (number(e, "admissionVersion") != 1 || e.value("source", std::string()) != "predicted" ||
        !fresh(number(e, "evaluatedAt"), now, 100) ||
        number(e, "lastMeasuredAt") != number(certificate, "measurementAt") ||
        !fresh(number(e, "lastMeasuredAt"), now,
               std::min(number(p, "horizon_ms", 1000), number(p, "wait_timeout_ms", 5000))) ||
        !std::isfinite(number(body, "x")) || !std::isfinite(number(body, "y")) ||
        !std::isfinite(number(e, "uncertaintyM")) ||
        number(e, "uncertaintyM") < number(certificate, "uncertainty"))
        return nullptr;
    const auto point = vector_value(body);
    const double range = point.norm(), bearing = point.angle(), closing = std::max(0.0, forward),
                 stopping = .6 * closing +
                            closing * closing / (2 * number(p, "braking_deceleration_mps2", .3)),
                 uncertainty = number(e, "uncertaintyM") +
                               (number(p, "max_target_speed_mps", 3) + own_speed) *
                                   (now - number(e, "evaluatedAt")) / 1000,
                 lower = range - uncertainty,
                 clearance =
                     lower -
                     std::max(number(c, "safety_distance_m", .3), number(p, "clearance_m", 1.2)) -
                     stopping;
    if (!std::isfinite(clearance) || clearance <= 0 || std::abs(bearing) >= 35)
        return nullptr;
    return {{"bearingDeg", bearing},
            {"rangeLowerBoundM", lower},
            {"clearanceM", clearance},
            {"stoppingDistanceM", stopping},
            {"uncertaintyM", uncertainty},
            {"measurementAt", e["lastMeasuredAt"]},
            {"ageMs", now - number(e, "lastMeasuredAt")}};
}
Json DriftHold::update(double now, const Json &t, const Json &result) {
    const auto p = c.value("prediction", Json::object()), a = t.value("attitude", Json::object()),
               v = t.value("velocity", Json::object()), pos = t.value("position", Json::object()),
               ekf = t.value("ekf", Json::object()), acro = t.value("acroControl", Json::object());
    const auto zero = [&](const std::string &reason) {
        creep_speed = 0;
        creep_at = now;
        last_progress = missing;
        far_pursuit = false;
        return Json{
            {"active", false}, {"reason", reason}, {"forwardSpeedMps", 0}, {"yawRateDegS", 0}};
    };
    if (!c.value("waiting_drift_hold", false) || !result.value("pausePropulsion", false) ||
        result.value("status", std::string()) != "waiting_target")
        return zero("not_waiting");
    const double age_limit = number(c, "navigation_timeout_ms", 700);
    std::string mode = t.value("flightMode", std::string());
    for (auto &ch : mode)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    if (!t.value("armed", false) || mode != "ACRO" ||
        number(t.value("gps", Json::object()), "fixType", 0) < 3 || !ekf.value("healthy", false) ||
        !fresh(number(ekf, "updatedAt"), now, age_limit) ||
        !fresh(number(pos, "updatedAt"), now, age_limit) || !std::isfinite(number(pos, "lat")) ||
        !std::isfinite(number(pos, "lon")) || !fresh(number(a, "updatedAt"), now, age_limit) ||
        !std::isfinite(number(a, "yaw")) || !std::isfinite(number(v, "vx")) ||
        !std::isfinite(number(v, "vy")) || !acro.value("ready", false) ||
        !fresh(number(acro, "updatedAt"), now, age_limit))
        return zero("hold_navigation_unavailable");
    const auto u = t.value("uwb", Json::object());
    if (u.value("fresh", false) && number(u, "distanceM") > 0 &&
        number(u, "distanceM") <= number(c, "safety_distance_m", .3))
        return zero("safety_distance");
    if (std::isfinite(number(last_tracking, "range")) &&
        number(last_tracking, "range") <=
            number(c, "safety_distance_m", .3) + number(p, "measurement_noise_m", .12))
        return zero("last_clearance_too_small");
    if (!anchor.is_object())
        anchor = {{"lat", pos["lat"]},
                  {"lon", pos["lon"]},
                  {"heading", last_tracking.is_object() && now - number(last_tracking, "at") <=
                                                               number(p, "horizon_ms", 1000)
                                  ? number(last_tracking, "heading")
                                  : number(a, "yaw")},
                  {"enteredAt", now}};
    if (now - number(anchor, "enteredAt") >= number(p, "wait_timeout_ms", 5000))
        return zero("hold_timeout");
    const auto moved = displacement(pos, anchor);
    const double yaw = number(a, "yaw") * radians,
                 forward_displacement = moved.x * std::cos(yaw) + moved.y * std::sin(yaw),
                 cross = -moved.x * std::sin(yaw) + moved.y * std::cos(yaw),
                 forward = number(v, "vx") * std::cos(yaw) + number(v, "vy") * std::sin(yaw),
                 maximum = std::max(
                     0.0, std::min(number(p, "max_speed_mps", .35), number(c, "max_speed_mps", 2))),
                 reverse = std::max(0.0, std::min(maximum, number(c, "max_reverse_speed_mps", .2))),
                 bias = std::min({.03, maximum, reverse});
    double speed = clamp(-.6 * forward_displacement - .25 * forward, -reverse, maximum);
    if (std::abs(speed) < bias)
        speed = -bias;
    if (forward_displacement > .08)
        speed = std::min(speed, -bias);
    double yaw_limit =
        std::min(number(c, "max_yaw_rate_deg_s", 14),
                 number(c, "max_yaw_rate_deg_s", 14) * number(p, "max_steering_deg", 6) /
                     std::max(1.0, number(c, "relative_steering_limit_deg", 18)));
    const auto direction = observe_bearing(now, t);
    const bool live = direction.value("trusted", false);
    if (live) {
        anchor["heading"] = direction["headingDeg"];
        yaw_limit = number(c, "max_yaw_rate_deg_s", 14);
    }
    refresh_range(now, t);
    double lower = missing, clearance = missing;
    if (certificate.is_object()) {
        lower = number(certificate, "range") - number(certificate, "uncertainty") -
                displacement(pos, certificate).norm() -
                number(p, "max_target_speed_mps", 3) *
                    std::max(0.0, now - number(certificate, "at")) / 1000;
        clearance = lower -
                    std::max(number(c, "safety_distance_m", .3), number(p, "clearance_m", 1.2)) -
                    .6 * std::max(0.0, forward);
    }
    bool near = false;
    for (const auto &s : u.value("sensors", Json::array())) {
        double at = number(s, "rawDistanceUpdatedAt", 0);
        if (at == 0)
            at = number(s, "receivedMeasurementAt");
        near |= fresh(at, now, number(p, "measurement_gap_ms", 450)) &&
                std::isfinite(number(s, "rawDistanceM")) && number(s, "rawDistanceM") >= 0 &&
                number(s, "rawDistanceM") <= number(p, "clearance_m", 1.2);
    }
    const auto m = c.value("reject_nmpc", Json::object());
    if (m.value("enabled", false)) {
        const Json evidence = m.value("clearance_policy", std::string()) == "adaptive"
                                  ? nmpc.range_evidence(now, t)
                                  : Json(nullptr),
                   plan = nmpc.solve(now, t, direction, evidence.is_null() ? Json(near) : evidence);
        const double error = wrap(number(anchor, "heading") - number(a, "yaw"));
        double turn =
            clamp(number(c, "traditional_heading_gain", .85) * error -
                      number(c, "traditional_yaw_rate_damping", .6) * number(a, "yawRateDegS", 0),
                  -yaw_limit, yaw_limit);
        const bool feasible = plan.value("feasible", false);
        if (feasible) {
            speed = number(plan, "forwardSpeedMps");
            turn = number(plan, "yawRateDegS");
            anchor["lat"] = pos["lat"];
            anchor["lon"] = pos["lon"];
        } else {
            if (std::isfinite(number(evidence, "rangeM")))
                speed = std::min(0.0, speed);
            nmpc.command = {{"at", now}, {"speed", speed}, {"yaw", turn}};
        }
        far_pursuit = false;
        creep_speed = std::max(0.0, speed);
        creep_at = now;
        return {{"active", true},
                {"reason", feasible ? "reject_nmpc" : "gps_drift_hold"},
                {"forwardSpeedMps", speed},
                {"yawRateDegS", turn},
                {"forwardDisplacementM", forward_displacement},
                {"crossDisplacementM", cross},
                {"forwardVelocityMps", forward},
                {"anchorHeadingDeg", anchor["heading"]},
                {"headingErrorDeg", error},
                {"elapsedMs", now - number(anchor, "enteredAt")},
                {"positionHoldIsLongitudinal", !feasible},
                {"liveBearingConfirmed", live},
                {"bearingConfidence", direction},
                {"bearingProgress", false},
                {"predictedProgress", false},
                {"farProgress", false},
                {"nmpcProgress", feasible},
                {"nmpc", plan}};
    }
    const auto prediction =
        accepted_prediction(now, result, forward, std::hypot(number(v, "vx"), number(v, "vy")));
    const double infinity = std::numeric_limits<double>::infinity(),
                 certificate_lower =
                     certificate.is_object() && fresh(number(certificate, "measurementAt"), now,
                                                      number(p, "wait_timeout_ms", 5000))
                         ? lower - number(p, "max_target_speed_mps", 3) *
                                       std::max(0.0, number(certificate, "at") -
                                                         number(certificate, "measurementAt")) /
                                       1000
                         : missing,
                 far_lower =
                     std::max(std::isfinite(certificate_lower) ? certificate_lower : -infinity,
                              number(prediction, "rangeLowerBoundM", -infinity)),
                 brake = number(p, "braking_deceleration_mps2", .3),
                 closing = std::max(0.0, forward),
                 stopping = .6 * closing + closing * closing / (2 * brake),
                 far_clearance =
                     far_lower -
                     std::max(number(c, "safety_distance_m", .3), number(p, "clearance_m", 1.2)) -
                     stopping;
    const double bearing = number(direction, "bearingDeg"),
                 outward = live ? std::max(0.0, ((bearing > 0) - (bearing < 0)) *
                                                    (number(direction, "rateDegS") -
                                                     number(a, "yawRateDegS", 0)))
                                : 0,
                 projected = live ? std::abs(bearing) + .5 * outward : 180,
                 far_aligned = clamp((35 - projected) / 15, 0, 1);
    far_pursuit = c.value("far_pursuit_enabled", false) && live && !near && far_aligned > 0 &&
                  far_lower >= (far_pursuit ? 2.5 : 3) && far_clearance > 0;
    const double far_limit = std::min(
                     {number(c, "max_speed_mps", 2),
                      c.value("commissioning_mode", false) ? number(c, "commissioning_speed_mps", 2)
                                                           : 2,
                      maximum + (1 - maximum) * clamp((far_lower - 2.5) / 2.5, 0, 1)}),
                 live_aligned = live ? clamp((35 - std::abs(bearing)) / 15, 0, 1) : 0,
                 predicted_aligned =
                     prediction.is_object()
                         ? clamp(
                               (35 - std::abs(live ? bearing : number(prediction, "bearingDeg"))) /
                                   15,
                               0, 1)
                         : 0;
    const bool bearing_progress = !far_pursuit && live && live_aligned > 0 && !near &&
                                  std::isfinite(clearance) && clearance > 0,
               predicted_progress = !far_pursuit && !bearing_progress && !near &&
                                    prediction.is_object() && predicted_aligned > 0,
               progress = far_pursuit || bearing_progress || predicted_progress;
    if (predicted_progress && !live)
        anchor["heading"] = wrap(number(a, "yaw") + number(prediction, "bearingDeg"));
    const double dt = std::isfinite(creep_at) ? clamp((now - creep_at) / 1000, 0, .25) : .1;
    creep_at = now;
    if (progress) {
        const double available = predicted_progress ? number(prediction, "clearanceM") : clearance,
                     aligned = predicted_progress ? predicted_aligned : live_aligned,
                     cap =
                         far_pursuit
                             ? std::min(
                                   far_limit * far_aligned,
                                   std::sqrt(
                                       squared(.6 * brake) +
                                       2 * brake *
                                           std::max(0.0,
                                                    far_lower -
                                                        std::max(number(c, "safety_distance_m", .3),
                                                                 number(p, "clearance_m", 1.2)))) -
                                       .6 * brake)
                             : std::min(maximum * aligned, std::sqrt(2 * brake * available));
        creep_speed = std::min(cap, creep_speed + brake * dt);
        speed = creep_speed;
        last_progress = now;
        anchor["lat"] = pos["lat"];
        anchor["lon"] = pos["lon"];
    } else if (near || !std::isfinite(clearance) || clearance <= 0 ||
               !std::isfinite(last_progress) ||
               now - last_progress > number(p, "measurement_gap_ms", 450))
        creep_speed = 0;
    const double error = wrap(number(anchor, "heading") - number(a, "yaw")),
                 turn = clamp(number(c, "traditional_heading_gain", .85) * error -
                                  number(c, "traditional_yaw_rate_damping", .6) *
                                      number(a, "yawRateDegS", 0),
                              -yaw_limit, yaw_limit);
    return {{"active", true},
            {"reason", far_pursuit          ? "far_bearing_pursuit"
                       : predicted_progress ? "accepted_prediction_progress"
                       : bearing_progress   ? "bearing_progress"
                                            : "gps_drift_hold"},
            {"forwardSpeedMps", speed},
            {"yawRateDegS", turn},
            {"forwardDisplacementM", forward_displacement},
            {"crossDisplacementM", cross},
            {"forwardVelocityMps", forward},
            {"anchorHeadingDeg", anchor["heading"]},
            {"headingErrorDeg", error},
            {"elapsedMs", now - number(anchor, "enteredAt")},
            {"positionHoldIsLongitudinal", !progress},
            {"liveBearingConfirmed", live},
            {"bearingSamples", bearings.size()},
            {"yawLimitDegS", yaw_limit},
            {"bearingProgress", bearing_progress},
            {"bearingConfidence", direction},
            {"rangeLowerBoundM", nullable(lower)},
            {"creepClearanceM", nullable(clearance)},
            {"rangeCertificateAt", certificate.is_object() ? certificate["at"] : Json(nullptr)},
            {"rangeCertificateMeasurementAt",
             certificate.is_object() ? certificate["measurementAt"] : Json(nullptr)},
            {"predictedProgress", predicted_progress},
            {"acceptedPrediction", prediction},
            {"farProgress", far_pursuit},
            {"farRangeLowerBoundM", nullable(far_lower)},
            {"farClearanceM", nullable(far_clearance)},
            {"farBoundSource",
             prediction.is_object() &&
                     number(prediction, "rangeLowerBoundM") >
                         (std::isfinite(certificate_lower) ? certificate_lower : -infinity)
                 ? "accepted_prediction"
                 : "accepted_certificate"},
            {"farRangeCertificateAgeMs", certificate.is_object()
                                             ? Json(now - number(certificate, "measurementAt"))
                                             : Json(nullptr)},
            {"farStoppingDistanceM", stopping},
            {"farSpeedLimitMps", far_limit},
            {"farProjectedBearingDeg", projected}};
}
} // namespace manta
