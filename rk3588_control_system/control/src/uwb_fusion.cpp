#include "manta/uwb_fusion.hpp"
#include <cctype>
#include <regex>
#include <stdexcept>

namespace manta {
static void history(std::vector<double> &values, double value) {
    values.push_back(value);
    if (values.size() > 20)
        values.erase(values.begin());
}
static double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}
UwbFusion::UwbFusion(const Json &o)
    : enabled(o.value("enabled_receiver_ids", std::vector<int>{1, 2})),
      gated(o.value("require_gated_measurements", false)),
      freshness(std::max(100., number(o, "freshness_ms", 1500))),
      fov(clamp(number(o, "receiver_fov_half_deg", 70), 10, 89)),
      disagreement(clamp(number(o, "overlap_disagreement_deg", 30), 5, 90)),
      position_disagreement(std::max(.1, number(o, "overlap_position_disagreement_m", .45))),
      minimum(std::max(.01, number(o, "minimum_distance_m", .15))),
      maximum(std::max(minimum + .1, number(o, "maximum_distance_m", 20))),
      confidence(std::max(1., number(o, "switch_confidence_ratio", 1.35))),
      confirm_ms(std::max(0., number(o, "switch_confirm_ms", 180))),
      blend_ms(std::max(0., number(o, "switch_blend_ms", 180))) {
    const auto mounts = o.value("mount_yaw_deg", std::vector<double>{0, 90});
    const auto xs = o.value("receiver_position_x_m", std::vector<double>{0, 0}),
               ys = o.value("receiver_position_y_m", std::vector<double>{0, 0});
    if (mounts.size() != 2 || xs.size() != 2 || ys.size() != 2 || enabled.empty())
        throw std::invalid_argument("invalid dual UWB geometry");
    std::sort(enabled.begin(), enabled.end());
    enabled.erase(std::unique(enabled.begin(), enabled.end()), enabled.end());
    for (int id : enabled)
        if (id < 1 || id > 2)
            throw std::invalid_argument("invalid UWB receiver ID");
    for (int i = 0; i < 2; ++i) {
        auto &d = receivers[i].data;
        d = {{"receiverId", i + 1},
             {"mountYawDeg", wrap(mounts[i])},
             {"positionXM", xs[i]},
             {"positionYM", ys[i]},
             {"measurementStatus", gated ? "unavailable" : "legacy"}};
        for (const auto *key : {"sourceSystem",
                                "sourceComponent",
                                "rawDistanceM",
                                "rawDistanceUpdatedAt",
                                "rawAzimuthDeg",
                                "rawElevationDeg",
                                "distanceM",
                                "azimuthDeg",
                                "elevationDeg",
                                "sentCount",
                                "maxAvailable",
                                "sequenceId",
                                "anchorId",
                                "tagId",
                                "tagStatus",
                                "valid",
                                "updatedAt",
                                "measurementUpdatedAt",
                                "receivedMeasurementAt",
                                "rawRejectedAt",
                                "filterReason",
                                "filterNis",
                                "filterInnovationM",
                                "filterUncertaintyM",
                                "unsmoothedAzimuthDeg",
                                "angularFilterMode",
                                "smoothingDisplacementM",
                                "filterRecoveryDiagnostics",
                                "occlusionDiagnostics",
                                "invalidReason"})
            d[key] = nullptr;
        for (const auto *key :
             {"goodCount", "rxBytes", "heartbeatCount", "checksumErrors", "lengthErrors",
              "filterRecoveryCount", "filterAdmissionVersion", "rejectedCount"})
            d[key] = 0;
    }
}
std::optional<std::pair<int, std::string>> UwbFusion::parse_name(std::string name) {
    const auto first = name.find_first_not_of(" \t\r\n"), last = name.find_last_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    name = name.substr(first, last - first + 1);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::toupper(c); });
    static const std::regex pattern(
        "^UWB([12])?_(DIST|AZ|EL|GOOD|VALID|SEQ|ANCH|TAG|STAT|RX|HB|CRC|LEN|SENT|AVL)([12])?$");
    std::smatch m;
    if (!std::regex_match(name, m, pattern) || (m[1].matched && m[3].matched))
        return {};
    return std::pair{m[1].matched   ? std::stoi(m[1])
                     : m[3].matched ? std::stoi(m[3])
                                    : 0,
                     m[2].str()};
}
int UwbFusion::legacy_receiver(const Json &source) {
    if (source.is_null())
        return 1;
    const std::pair<int, int> key{source.at(0).get<int>(), source.at(1).get<int>()};
    if (sources.contains(key))
        return sources[key];
    if (sources.size() >= 2)
        return 1;
    const int id = int(sources.size()) + 1;
    sources[key] = id;
    receivers[id - 1].data["sourceSystem"] = key.first;
    receivers[id - 1].data["sourceComponent"] = key.second;
    return id;
}
std::optional<std::pair<int, std::string>> UwbFusion::update(const std::string &name, double value,
                                                             double now, const Json &source) {
    const auto parsed = parse_name(name);
    if (!parsed || !std::isfinite(value))
        return {};
    const int id = parsed->first ? parsed->first : legacy_receiver(source);
    const auto &field = parsed->second;
    auto &r = receivers[id - 1];
    auto &d = r.data;
    if (!source.is_null()) {
        d["sourceSystem"] = source.at(0);
        d["sourceComponent"] = source.at(1);
    }
    if (field == "DIST") {
        r.distance_at = now;
        d["rawDistanceUpdatedAt"] = now;
    }
    static const std::map<std::string, std::string> metadata = {
        {"GOOD", "goodCount"},     {"SENT", "sentCount"},  {"AVL", "maxAvailable"},
        {"SEQ", "sequenceId"},     {"ANCH", "anchorId"},   {"TAG", "tagId"},
        {"STAT", "tagStatus"},     {"RX", "rxBytes"},      {"HB", "heartbeatCount"},
        {"CRC", "checksumErrors"}, {"LEN", "lengthErrors"}};
    if (metadata.contains(field)) {
        d[metadata.at(field)] = std::max(0., std::trunc(value));
        return std::pair{id, field};
    }
    if (field == "VALID") {
        d["valid"] = value > 0;
        if (value <= 0)
            r.pending_distance = r.pending_azimuth = false;
        d["updatedAt"] = now;
        return std::pair{id, field};
    }
    if ((field == "DIST" && (value < minimum || value > maximum)) ||
        (gated &&
         ((field == "AZ" && std::abs(value) > 180) || (field == "EL" && std::abs(value) > 90)))) {
        d[field == "DIST" ? "rawDistanceM"
          : field == "AZ" ? "rawAzimuthDeg"
                          : "rawElevationDeg"] = value;
        if (field == "DIST")
            d["valid"] = false;
        d["invalidReason"] = field == "DIST" ? "distance_out_of_range"
                             : field == "AZ" ? "azimuth_out_of_range"
                                             : "elevation_out_of_range";
        r.pending_distance = r.pending_azimuth = false;
        d["rejectedCount"] = number(d, "rejectedCount", 0) + 1;
        d["updatedAt"] = now;
        if (field != "EL")
            d["rawRejectedAt"] = now;
        return std::pair{id, std::string("REJECTED")};
    }
    if (field == "DIST") {
        d["invalidReason"] = nullptr;
        d["rawDistanceM"] = value;
        r.pending_distance = true;
        if (!gated)
            history(r.distances, value);
    } else if (field == "AZ") {
        d["rawAzimuthDeg"] = wrap(value);
        r.pending_azimuth = true;
        if (!gated)
            history(r.azimuths, wrap(value));
    } else if (field == "EL")
        d["rawElevationDeg"] = value;
    d["updatedAt"] = now;
    if (field == "DIST")
        r.distance_at = now;
    else if (field == "AZ")
        r.azimuth_at = now;
    if (r.pending_distance && r.pending_azimuth) {
        if (std::abs(r.distance_at - r.azimuth_at) <= 100) {
            d["receivedMeasurementAt"] = std::max(r.distance_at, r.azimuth_at);
            if (!gated)
                d["measurementUpdatedAt"] = d["receivedMeasurementAt"];
            r.pending_distance = r.pending_azimuth = false;
        } else if (r.distance_at < r.azimuth_at)
            r.pending_distance = false;
        else
            r.pending_azimuth = false;
    }
    return std::pair{id, field};
}
void UwbFusion::set_filtered(int id, const std::string &field, double value) {
    auto &d = receivers.at(id - 1).data;
    if (field == "DIST")
        d["distanceM"] = clamp(value, minimum, maximum);
    else if (field == "AZ")
        d["azimuthDeg"] = wrap(value);
    else if (field == "EL")
        d["elevationDeg"] = value;
}
void UwbFusion::apply(int id, const Json &result) {
    if (result.value("duplicate", false))
        return;
    auto &r = receivers.at(id - 1);
    auto &d = r.data;
    static const std::map<std::string, std::string> fields = {
        {"status", "measurementStatus"},
        {"reason", "filterReason"},
        {"nis", "filterNis"},
        {"innovation_m", "filterInnovationM"},
        {"uncertainty_m", "filterUncertaintyM"},
        {"unsmoothed_azimuth_deg", "unsmoothedAzimuthDeg"},
        {"angular_filter_mode", "angularFilterMode"},
        {"smoothing_displacement_m", "smoothingDisplacementM"},
        {"recovery_diagnostics", "filterRecoveryDiagnostics"},
        {"occlusion_diagnostics", "occlusionDiagnostics"}};
    for (const auto &[from, to] : fields)
        d[to] = result.value(from, Json(nullptr));
    d["filterRecoveryCount"] = result.value("recovery_count", 0);
    d["filterAdmissionVersion"] = result.value("admission_version", 0);
    d["rejectedCount"] =
        std::max(number(d, "rejectedCount", 0), number(result, "rejected_count", 0));
    if (result.value("accepted", false)) {
        set_filtered(id, "DIST", number(result, "distance_m"));
        set_filtered(id, "AZ", number(result, "azimuth_deg"));
        d["measurementUpdatedAt"] = result.at("accepted_at");
        d["invalidReason"] = nullptr;
        history(r.distances, number(d, "rawDistanceM"));
        history(r.azimuths, number(d, "rawAzimuthDeg"));
    } else
        d["invalidReason"] = result.at("reason");
}
bool UwbFusion::complete(const Receiver &r) const {
    const auto &d = r.data;
    return d["valid"] != false && !d["distanceM"].is_null() && !d["azimuthDeg"].is_null() &&
           (!gated || d["measurementStatus"] == "trusted");
}
bool UwbFusion::fresh(const Receiver &r, double now) const {
    const double at = number(r.data, gated ? "measurementUpdatedAt" : "updatedAt");
    return complete(r) && std::isfinite(at) && now >= at && now - at <= freshness;
}
double UwbFusion::bearing(const Receiver &r) const {
    return wrap(number(r.data, "azimuthDeg") + number(r.data, "mountYawDeg"));
}
Vec UwbFusion::position(const Receiver &r) const {
    const double a = bearing(r) * radians, d = number(r.data, "distanceM");
    return {number(r.data, "positionXM") + d * std::cos(a),
            number(r.data, "positionYM") + d * std::sin(a)};
}
double UwbFusion::coverage(const Receiver &r) const {
    const double angle = number(r.data, gated ? "rawAzimuthDeg" : "azimuthDeg");
    if (!std::isfinite(angle) || std::abs(wrap(angle)) > fov)
        return 0;
    return std::max(.05, squared(std::cos(std::abs(wrap(angle)) / fov * pi / 2)));
}
double UwbFusion::stability(const Receiver &r) const {
    if (r.distances.size() < 6 || r.azimuths.size() < 6)
        return 1;
    const double middle = median(r.distances);
    std::vector<double> deviations;
    for (double d : r.distances)
        deviations.push_back(std::abs(d - middle));
    const double distance_score = 1 / (1 + squared(8 * median(deviations) / std::max(.5, middle)));
    Vec average;
    for (double a : r.azimuths)
        average = average + Vec{std::cos(a * radians), std::sin(a * radians)};
    deviations.clear();
    for (double a : r.azimuths)
        deviations.push_back(std::abs(wrap(a - average.angle())));
    return clamp(distance_score / (1 + squared(median(deviations) / 12)), .01, 1);
}
Json UwbFusion::telemetry(const Receiver &r, double now) const {
    Json d = r.data;
    d["bodyAzimuthDeg"] = nullable(bearing(r));
    d["fresh"] = fresh(r, now);
    return d;
}
Json UwbFusion::fused(double now) {
    std::vector<std::pair<int, double>> candidates;
    Json sensors = Json::array();
    for (int id : enabled) {
        const auto &r = receivers[id - 1];
        const double weight = coverage(r) * stability(r);
        if (fresh(r, now) && weight > 0)
            candidates.push_back({id, weight});
        auto sensor = telemetry(r, now);
        const double angle = number(r.data, gated ? "rawAzimuthDeg" : "azimuthDeg");
        sensor["withinFov"] =
            std::isfinite(angle) ? Json(std::abs(wrap(angle)) <= fov) : Json(nullptr);
        sensor["fovHalfDeg"] = fov;
        sensor["fusionExclusionReason"] = !std::isfinite(angle) ? Json("no_complete_measurement")
                                          : sensor["withinFov"] == false
                                              ? Json("outside_receiver_fov")
                                          : !complete(r)   ? r.data["filterReason"]
                                          : !fresh(r, now) ? Json("stale_measurement")
                                                           : Json(nullptr);
        sensor["stabilityWeight"] = stability(r);
        sensor["fusionWeight"] = weight;
        sensors.push_back(sensor);
    }
    Json out = {{"online", false},
                {"fresh", false},
                {"distance_m", nullptr},
                {"azimuth_deg", nullptr},
                {"elevation_deg", nullptr},
                {"good_count", 0},
                {"updated_at", nullptr},
                {"receiver_count", 0},
                {"observing_receiver_count", 0},
                {"active_receiver_ids", Json::array()},
                {"fusion_mode", "no_valid_receiver"},
                {"bearing_disagreement_deg", nullptr},
                {"position_disagreement_m", nullptr},
                {"relative_x_m", nullptr},
                {"relative_y_m", nullptr},
                {"sensors", sensors}};
    if (candidates.empty())
        return out;
    const size_t observing = candidates.size();
    std::string mode = "single_receiver";
    if (candidates.size() == 1) {
        if (selected && selected != candidates[0].first) {
            blend_from = last_output;
            blend_started = now;
        }
        selected = candidates[0].first;
        challenger = 0;
        challenger_since = missing;
    } else {
        const auto &a = receivers[candidates[0].first - 1];
        const auto &b = receivers[candidates[1].first - 1];
        const double delta = std::abs(wrap(bearing(a) - bearing(b))),
                     distance = (position(a) - position(b)).norm();
        out["bearing_disagreement_deg"] = delta;
        out["position_disagreement_m"] = distance;
        if (delta > disagreement ||
            distance >
                std::max(position_disagreement,
                         .3 * std::min(number(a.data, "distanceM"), number(b.data, "distanceM")))) {
            const auto best =
                *std::max_element(candidates.begin(), candidates.end(), [&](auto x, auto y) {
                    return x.second == y.second
                               ? number(receivers[x.first - 1].data, "updatedAt", 0) <
                                     number(receivers[y.first - 1].data, "updatedAt", 0)
                               : x.second < y.second;
                });
            auto prior = std::find_if(candidates.begin(), candidates.end(),
                                      [&](auto p) { return p.first == selected; });
            auto chosen = best;
            if (prior == candidates.end()) {
                selected = best.first;
                challenger = 0;
                challenger_since = missing;
            } else if (best.first == prior->first || best.second < prior->second * confidence) {
                challenger = 0;
                challenger_since = missing;
                chosen = *prior;
            } else if (challenger != best.first) {
                challenger = best.first;
                challenger_since = now;
                chosen = *prior;
            } else if (!std::isfinite(challenger_since) || now - challenger_since < confirm_ms)
                chosen = *prior;
            else {
                blend_from = last_output;
                blend_started = now;
                selected = best.first;
                challenger = 0;
                challenger_since = missing;
            }
            candidates = {chosen};
            mode = "hysteretic_receiver_select";
        } else {
            selected = challenger = 0;
            challenger_since = missing;
            mode = "overlap_weighted_fusion";
        }
    }
    Vec p;
    double sum = 0, elevation_sum = 0, elevation_weight = 0, good = 0, updated = 0;
    Json active = Json::array();
    for (const auto &[id, w] : candidates) {
        const auto &r = receivers[id - 1];
        p = p + position(r) * w;
        sum += w;
        active.push_back(id);
        good += number(r.data, "goodCount", 0);
        updated = std::max(updated, number(r.data, "updatedAt", 0));
        const double e = number(r.data, "elevationDeg");
        if (std::isfinite(e)) {
            elevation_sum += w * e;
            elevation_weight += w;
        }
    }
    p = p / sum;
    if (blend_from && std::isfinite(blend_started) && blend_ms > 0) {
        const double blend = clamp((now - blend_started) / blend_ms, 0, 1);
        p = *blend_from + (p - *blend_from) * blend;
        if (blend >= 1) {
            blend_from.reset();
            blend_started = missing;
        }
    }
    last_output = p;
    out.update(Json{{"online", true},
                    {"fresh", true},
                    {"distance_m", p.norm()},
                    {"azimuth_deg", wrap(p.angle())},
                    {"elevation_deg",
                     elevation_weight > 0 ? Json(elevation_sum / elevation_weight) : Json(nullptr)},
                    {"good_count", good},
                    {"updated_at", updated},
                    {"receiver_count", candidates.size()},
                    {"observing_receiver_count", observing},
                    {"active_receiver_ids", active},
                    {"fusion_mode", mode},
                    {"relative_x_m", p.x},
                    {"relative_y_m", p.y}});
    return out;
}
} // namespace manta
