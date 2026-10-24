#include "manta/prediction.hpp"

namespace manta {
bool quarantined(const Json &sensor) {
    const auto d = sensor.value("occlusionDiagnostics", Json(nullptr));
    if (!d.is_object())
        return false;
    const auto state = d.value("state", std::string());
    return state == "suspect" || state == "quarantined" || state == "recovering";
}
void RawRecovery::reset() {
    window.clear();
    token.clear();
    at = missing;
    fit_result = nullptr;
    diagnostic = {{"reason", "idle"}, {"hits", 0}, {"window", 0}};
}
bool RawRecovery::observe(double now, const Json &t, TargetEstimator &estimator, const Json &anchor,
                          bool allow_handoff) {
    const auto n = [&](const char *key) { return number(config, key); };
    const auto u = t.value("uwb", Json::object());
    const auto report = [&](const std::string &reason, size_t hits = 0,
                            const Json &extra = Json::object()) {
        diagnostic = {{"reason", reason}, {"hits", hits}, {"window", window.size()}};
        diagnostic.update(extra);
        return false;
    };
    std::vector<Json> sensors;
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::isfinite(number(s, "receivedMeasurementAt")) ||
            std::isfinite(number(s, "rawRejectedAt")))
            sensors.push_back(s);
    if (!anchor.is_object() || !estimator.last_pose.is_object() || sensors.size() != 1)
        return report("raw_receiver_or_pose_unavailable");
    const auto &s = sensors[0];
    const double sample_at =
        std::max(number(s, "receivedMeasurementAt", 0), number(s, "rawRejectedAt", 0));
    if (quarantined(s)) {
        reset();
        return report("occlusion_quarantine");
    }
    if (!fresh_time(sample_at, now, n("measurement_gap_ms")))
        return report("raw_stale");
    const double range = number(s, "rawDistanceM"), azimuth = number(s, "rawAzimuthDeg");
    const bool far = std::isfinite(range) && range > 5;
    std::erase_if(window,
                  [&](const Json &p) { return sample_at - number(p, "at") > (far ? 900 : 500); });
    if (std::isfinite(at) && sample_at < at)
        return false;
    const bool same = sample_at == at;
    const std::string sample_token = std::isfinite(number(s, "sequenceId"))
                                         ? s["receiverId"].dump() + ":" + s["sequenceId"].dump()
                                         : "time:" + Json(sample_at).dump();
    if (!same && sample_token == token)
        return false;
    if (same && (!allow_handoff || u.value("measurementStatus", std::string()) != "trusted" ||
                 s.value("measurementStatus", std::string()) != "trusted" ||
                 number(u, "measurementUpdatedAt") != sample_at))
        return false;
    if (!same) {
        at = sample_at;
        token = sample_token;
    }
    const bool valid = sample_at == number(s, "receivedMeasurementAt") &&
                       s.value("valid", Json(nullptr)) != false && std::isfinite(range) &&
                       range >= .15 && range <= 20 && std::isfinite(azimuth) &&
                       std::abs(azimuth) <= 180;
    auto raw = u;
    raw.update({{"online", true},
                {"fresh", true},
                {"measurementUpdatedAt", sample_at},
                {"activeReceiverIds", Json::array({s["receiverId"]})},
                {"sensors", Json::array({s})},
                {"distanceM", nullable(range)},
                {"azimuthDeg", nullable(azimuth + number(s, "mountYawDeg", 0))}});
    Json sample = nullptr;
    if (valid) {
        if (same) {
            for (const auto &p : window)
                if (number(p, "at") == sample_at &&
                    p.value("position", Json(nullptr)).is_object()) {
                    sample = p;
                    break;
                }
        } else
            sample = estimator.sample(raw, estimator.last_pose, now);
    }
    if (!same) {
        window.push_back(sample.is_object() ? sample
                                            : Json{{"at", sample_at}, {"position", nullptr}});
        const size_t maximum = far ? 9 : 5;
        if (window.size() > maximum)
            window.erase(window.begin(), window.end() - maximum);
    }
    if (!sample.is_object())
        return report("basic_invalid");
    if (!anchor["id"].is_null() && !sample["id"].is_null() && anchor["id"] != sample["id"]) {
        window.clear();
        return report("identity_mismatch");
    }
    std::vector<Json> reachable;
    for (const auto &p : window)
        if (p["position"].is_object() && number(p, "at") >= number(anchor, "at") - 500 &&
            (vector_value(p["position"]) - vector_value(anchor["position"])).norm() <=
                n("innovation_gate_m") + n("max_target_speed_mps") *
                                             std::abs(number(p, "at") - number(anchor, "at")) /
                                             1000)
            reachable.push_back(p);
    const double minimum = std::max(n("min_samples"), number(config, "recovery_min_samples", 3));
    if (reachable.size() < minimum)
        return report("insufficient_hits", reachable.size());
    const double radial = 2 * n("measurement_noise_m"),
                 transverse = far ? std::max(radial, std::min(n("innovation_gate_m"),
                                                              range * std::sin(2 * radians)))
                                  : radial;
    const auto delta =
        vector_value(sample["position"]) - vector_value(estimator.last_pose["position"]);
    const double length = std::max(.15, delta.norm()), ux = delta.x / length, uy = delta.y / length;
    struct Model {
        Vec position, velocity;
        double residual = 0, allowance = 0;
        std::vector<Json> points;
    };
    const auto error = [&](const Model &model, const Json &p) {
        const auto difference = vector_value(p["position"]) - model.position -
                                model.velocity * ((number(p, "at") - sample_at) / 1000);
        return std::hypot((difference.x * ux + difference.y * uy) / radial,
                          (-difference.x * uy + difference.y * ux) / model.allowance);
    };
    const auto fit_model = [&](const std::vector<Json> &points,
                               double allowance) -> std::optional<Model> {
        std::vector<Point> values;
        for (const auto &p : points)
            values.push_back({number(p, "at"), vector_value(p["position"])});
        const auto fitted = fit(values, sample_at);
        if (!fitted)
            return std::nullopt;
        Model model{fitted->position, fitted->velocity, 0, allowance, points};
        for (const auto &p : points)
            model.residual = std::max(model.residual, error(model, p));
        return model;
    };
    std::vector<Model> hypotheses;
    for (size_t i = 0; i + 2 < reachable.size(); ++i)
        for (size_t j = i + 1; j + 1 < reachable.size(); ++j)
            for (size_t k = j + 1; k < reachable.size(); ++k) {
                std::vector<double> allowances = {radial};
                if (far)
                    allowances.push_back(transverse);
                for (double allowance : allowances) {
                    const double required = allowance > radial ? std::max(5.0, minimum) : minimum;
                    const auto initial =
                        fit_model({reachable[i], reachable[j], reachable[k]}, allowance);
                    if (!initial)
                        continue;
                    std::vector<Json> points;
                    for (const auto &p : reachable)
                        if (error(*initial, p) <= 1)
                            points.push_back(p);
                    if (points.size() < required || number(points.back(), "at") != sample_at ||
                        sample_at - number(points.front(), "at") < (required - 1) * 90)
                        continue;
                    bool gap = false;
                    for (size_t q = 1; q < points.size(); ++q)
                        if (number(points[q], "at") - number(points[q - 1], "at") >
                            n("measurement_gap_ms"))
                            gap = true;
                    if (gap)
                        continue;
                    const auto model = fit_model(points, allowance);
                    if (!model || model->velocity.norm() > n("max_target_speed_mps") ||
                        model->residual > 1)
                        continue;
                    hypotheses.push_back(*model);
                }
            }
    std::stable_sort(hypotheses.begin(), hypotheses.end(), [](const Model &a, const Model &b) {
        return a.points.size() != b.points.size() ? a.points.size() > b.points.size()
                                                  : a.residual < b.residual;
    });
    if (hypotheses.empty())
        return report("inconsistent_window");
    Model best = hypotheses[0];
    size_t hits = best.points.size();
    if (!allow_handoff)
        return report("candidate_ready", hits);
    if (sample_at <= number(anchor, "at"))
        return report("awaiting_new_measurement", hits);
    if (u.value("measurementStatus", std::string()) != "trusted" ||
        s.value("measurementStatus", std::string()) != "trusted" ||
        number(u, "measurementUpdatedAt") != sample_at)
        return report("awaiting_measurement_handoff", hits);
    const double posterior_uncertainty = number(s, "filterUncertaintyM");
    if (!std::isfinite(posterior_uncertainty) || posterior_uncertainty < 0)
        return report("measurement_handoff_invalid_uncertainty", hits);
    auto filtered = estimator.sample(u, estimator.last_pose, now);
    std::string handoff = "strict_raw_fit";
    double handoff_radial = radial, handoff_transverse = best.allowance;
    if (filtered.is_object()) {
        const auto strict = std::find_if(hypotheses.begin(), hypotheses.end(), [&](const Model &h) {
            auto p = filtered;
            p["at"] = sample_at;
            return error(h, p) <= 1;
        });
        if (strict != hypotheses.end())
            best = *strict;
        else {
            const double bounded = std::min(n("innovation_gate_m"), posterior_uncertainty);
            const auto compatible =
                std::find_if(hypotheses.begin(), hypotheses.end(), [&](const Model &h) {
                    if (h.points.size() < std::max(5.0, minimum) ||
                        sample_at - number(h.points.front(), "at") < 360)
                        return false;
                    const auto d = vector_value(filtered["position"]) - h.position;
                    return std::hypot((d.x * ux + d.y * uy) / std::max(radial, bounded),
                                      (-d.x * uy + d.y * ux) / std::max(h.allowance, bounded)) <= 1;
                });
            if (compatible != hypotheses.end()) {
                best = *compatible;
                handoff = "bounded_posterior_overlap";
                handoff_radial = std::max(radial, bounded);
                handoff_transverse = std::max(best.allowance, bounded);
            }
        }
        hits = best.points.size();
        if (handoff == "strict_raw_fit")
            handoff_transverse = best.allowance;
    }
    auto latest = filtered;
    if (latest.is_object())
        latest["at"] = sample_at;
    const double handoff_error = filtered.is_object() ? error(best, latest) : missing;
    const auto difference = filtered.is_object()
                                ? vector_value(filtered["position"]) - best.position
                                : Vec{missing, missing};
    Json details = {{"handoffNormalizedError", nullable(handoff_error)},
                    {"handoffRadialErrorM", nullable(difference.x * ux + difference.y * uy)},
                    {"handoffTransverseErrorM", nullable(-difference.x * uy + difference.y * ux)},
                    {"posteriorUncertaintyM", posterior_uncertainty},
                    {"radialToleranceM", radial},
                    {"transverseToleranceM", best.allowance},
                    {"handoffMode", handoff},
                    {"handoffRetrySameObservation", same},
                    {"handoffRadialToleranceM", handoff_radial},
                    {"handoffTransverseToleranceM", handoff_transverse}};
    if (!filtered.is_object() || (handoff == "strict_raw_fit" && handoff_error > 1))
        return report("measurement_handoff_disagrees", hits, details);
    if (handoff == "bounded_posterior_overlap")
        filtered["measurementUncertaintyM"] =
            std::max({number(filtered, "measurementUncertaintyM"), posterior_uncertainty,
                      difference.norm() + std::hypot(radial, best.allowance)});
    estimator.accept(filtered, estimator.last, hits);
    estimator.last_seen_at = number(filtered, "at");
    estimator.last_seen_token = filtered["token"];
    estimator.recovery = estimator.recovery_anchor = nullptr;
    estimator.loss_at = missing;
    estimator.recovered_at = now;
    fit_result = {{"source", "independent_raw_window"},
                  {"samples", hits},
                  {"window", window.size()},
                  {"normalizedResidual", best.residual},
                  {"radialToleranceM", radial},
                  {"transverseToleranceM", best.allowance},
                  {"finalPosteriorUncertaintyM", posterior_uncertainty},
                  {"acceptedUncertaintyM", filtered["measurementUncertaintyM"]},
                  {"rawFitVelocityNedMps", vector_json(best.velocity)},
                  {"velocityNedMps", estimator.last["velocity"]},
                  {"velocityTrusted", estimator.last["velocityTrusted"]}};
    fit_result.update(details);
    diagnostic = {{"reason", "confirmed"}, {"hits", hits}, {"window", window.size()}};
    return true;
}
} // namespace manta
