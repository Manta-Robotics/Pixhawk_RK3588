#include "manta/uwb_pipeline.hpp"

namespace manta {
static Json fusion_options(const Json &config) {
    auto options = config.value("uwb_dual", Json::object());
    const auto filter = config.value("uwb_measurement_filter", Json::object());
    options["require_gated_measurements"] = filter.value("enabled", true);
    options["trust_checks_enabled"] = filter.value("trust_checks_enabled", true);
    if (options.contains("position_x_m"))
        options["receiver_position_x_m"] = options["position_x_m"];
    if (options.contains("position_y_m"))
        options["receiver_position_y_m"] = options["position_y_m"];
    return options;
}
UwbPipeline::UwbPipeline(const Json &config) : fusion(fusion_options(config)) {
    const auto follow = config.value("uwb_follow", Json::object());
    stop_distance = std::max(.1, number(follow, "safety_distance_m", .45));
    resume_distance = std::max(stop_distance + .05,
                              number(follow, "hard_stop_resume_distance_m", stop_distance + .1));
    const auto f = config.value("uwb_measurement_filter", Json::object());
    const auto b = config.value("gimbal", Json::object()).value("beacon", Json::object());
    bearing_sign = number(b, "yaw_source_sign", -1);
    bearing_scale = number(b, "yaw_source_scale", .8148);
    bearing_offset = number(b, "yaw_offset_deg", -4.816);
    fusion.heading_scale = bearing_sign * bearing_scale;
    for (int i = 0; i < 2; ++i)
        filters.emplace_back(Json{{"config", f},
                                  {"minimum_distance_m", fusion.minimum},
                                  {"maximum_distance_m", fusion.maximum},
                                  {"bearing_sign", number(b, "yaw_source_sign", -1)},
                                  {"bearing_scale", number(b, "yaw_source_scale", .8148)},
                                  {"bearing_offset_deg", number(b, "yaw_offset_deg", -4.816)},
                                  {"mount_yaw_deg", fusion.receivers[i].data["mountYawDeg"]}});
}
bool UwbPipeline::update(const std::string &name, double value, double now, const Json &source,
                         const Json &sequence, double heading, double heading_at,
                         const Json &wire_continuity,
                         const std::function<Json(uint32_t, double)> &align) {
    measurements = Json::array();
    const auto parsed = UwbFusion::parse_name(name);
    if (!parsed)
        return false;
    const int id = parsed->first ? parsed->first : fusion.legacy_receiver(source);
    auto &r = fusion.receivers.at(id - 1);
    auto &d = r.data;
    auto &filter = filters.at(id - 1);
    const auto revision = [&]() {
        return Json::array({d["receivedMeasurementAt"], d["measurementUpdatedAt"],
                            d["rejectedCount"], d["valid"], d["measurementStatus"]});
    };
    const auto before = revision();
    const auto prior_key = r.frame_key;
    const bool partial = r.pending_distance || r.pending_azimuth || r.pending_elevation;
    double prior_received = -INFINITY;
    for (double at : {r.pending_distance ? r.distance_at : missing,
                      r.pending_azimuth ? r.azimuth_at : missing,
                      r.pending_elevation ? r.elevation_at : missing})
        if (std::isfinite(at)) prior_received = std::max(prior_received, at);
    const Json prior_raw = {{"rawDistanceM", r.pending_distance ? d["rawDistanceM"] : Json(nullptr)},
                           {"rawAzimuthDeg", r.pending_azimuth ? d["rawAzimuthDeg"] : Json(nullptr)},
                           {"rawElevationDeg", r.pending_elevation ? d["rawElevationDeg"] : Json(nullptr)}};
    const auto result = fusion.update(name, value, now, source, sequence, wire_continuity);
    if (partial && !prior_key.is_null() && prior_key != r.frame_key) {
        auto record = prior_raw;
        record.update({{"receiverId", id}, {"frameKey", prior_key},
                       {"receivedAt", nullable(prior_received)}, {"finalizedAt", now},
                       {"complete", false},
                       {"status", "incomplete"}, {"reason", "superseded_partial_frame"}});
        measurements.push_back(record);
    }
    if (!(std::isfinite(heading_at) && now >= heading_at && now - heading_at <= 700))
        heading = missing;
    if (!result) {
        return before != revision();
    }
    Json timing = {{"filterTimeMs", now}, {"sampleAt", now}, {"receivedAt", now},
                   {"headingDeg", nullable(heading)}, {"headingMethod", "receive_time_fallback"},
                   {"sourceTimeKind", "receive_time"}};
    if (align && !sequence.is_null()) {
        const auto stamp = r.measurement_completed && !r.frame_key.is_null()
            ? r.frame_key.at(1).get<uint32_t>() : sequence.get<uint32_t>();
        timing = align(stamp, now);
        heading = number(timing, "headingDeg");
    }
    const double filter_now = number(timing, "filterTimeMs", now);
    Json filtered = nullptr;
    if (fusion.gated) {
        const auto &field = result->second;
        if (field == "REJECTED")
            filtered = filter.reject(filter_now, d["invalidReason"], heading);
        else if (field == "VALID" && d["valid"] == false)
            filtered = filter.reject(filter_now, "radio_invalid", heading);
        else if (r.measurement_completed) {
            if (!d["rawElevationDeg"].is_null())
                fusion.set_filtered(id, "EL", number(d, "rawElevationDeg"));
            filtered =
                fusion.trust_checks && std::abs(number(d, "rawAzimuthDeg")) > fusion.fov
                    ? filter.reject(filter_now, "outside_receiver_fov", heading)
                    : filter.update(
                          {{"distance_m", d["rawDistanceM"]},
                           {"azimuth_deg", d["rawAzimuthDeg"]},
                           {"now_ms", filter_now},
                           {"heading_deg", nullable(heading)},
                           {"valid", d["valid"] != false},
                           {"sequence_id", sequence.is_null() ? d["sequenceId"] : r.frame_key},
                           {"tag_id", d["tagId"]}});
        } else if (field == "EL" && std::abs(value) <= 90)
            fusion.set_filtered(id, "EL", value);
        if (filtered.is_object()) {
            auto admitted = filtered;
            // Preserve the v1 receipt/admission contract; source time is an
            // additional explicit clock, never a fabricated receive timestamp.
            if (admitted.value("accepted", false)) admitted["accepted_at"] = now;
            fusion.apply(id, admitted);
        }
    } else {
        const auto &field = result->second;
        const int index = field == "DIST" ? 0 : field == "AZ" ? 1 : field == "EL" ? 2 : -1;
        if (index >= 0) {
            auto &s = scalars[id - 1][index];
            if (!std::isfinite(s.estimate))
                s.estimate = index == 1 ? wrap(value) : value;
            else {
                const double uncertainty = s.uncertainty + (index == 0 ? .04 : 3),
                             gain = uncertainty / (uncertainty + (index == 0 ? .09 : 6));
                s.estimate += gain * (index == 1 ? wrap(value - s.estimate) : value - s.estimate);
                if (index == 1)
                    s.estimate = wrap(s.estimate);
                s.uncertainty = (1 - gain) * uncertainty;
            }
            fusion.set_filtered(id, field, s.estimate);
        }
    }
    if (r.measurement_completed && align) {
        d["sampleTiming"] = timing;
        if (!fusion.gated || (filtered.is_object() && filtered.value("accepted", false))) {
            d["measurementSampleAt"] = timing["sampleAt"];
            d["measurementHeadingDeg"] = timing["headingDeg"];
            d["acceptedFrameKey"] = r.frame_key;
        }
    }
    if (r.measurement_completed || result->second == "REJECTED" ||
        (result->second == "VALID" && d["valid"] == false)) {
        auto record = timing;
        const bool accepted = r.measurement_completed &&
            (!fusion.gated || (filtered.is_object() && filtered.value("accepted", false)));
        record.update({{"receiverId", id}, {"frameKey", r.frame_key}, {"source", source},
                       {"status", accepted ? "accepted" : "rejected"},
                       {"reason", accepted ? Json(nullptr) : d["invalidReason"]},
                       {"field", parsed->second}, {"fieldValue", nullable(value)},
                       {"tagId", d["tagId"]}, {"complete", r.measurement_completed},
                       {"protocol", r.strict_sources[{source.is_array() ? source.at(0).get<int>() : 0,
                                                       source.is_array() ? source.at(1).get<int>() : 0}]
                                        ? "shared_timestamp_v2" : "legacy_compatible"}});
        for (const auto *key : {"rawDistanceM", "rawAzimuthDeg", "rawElevationDeg"})
            record[key] = r.measurement_completed ? d[key] : Json(nullptr);
        if (!r.measurement_completed && parsed->second == "DIST") record["rawDistanceM"] = nullable(value);
        record["filteredDistanceM"] = accepted ? d["distanceM"] : Json(nullptr);
        record["filteredAzimuthDeg"] = accepted ? d["azimuthDeg"] : Json(nullptr);
        record["unsmoothedAzimuthDeg"] = accepted ? d["unsmoothedAzimuthDeg"] : Json(nullptr);
        record["rawBearingDeg"] = nullable(wrap((number(record, "rawAzimuthDeg") +
            number(d, "mountYawDeg")) * bearing_sign * bearing_scale + bearing_offset));
        record["filteredBearingDeg"] = accepted ? Json(wrap((number(d, "azimuthDeg") +
            number(d, "mountYawDeg")) * bearing_sign * bearing_scale + bearing_offset)) : Json(nullptr);
        for (const auto *key : {"filterNis", "filterInnovationM", "filterUncertaintyM", "angularFilterMode"})
            record[key] = d[key];
        record["trustChecksEnabled"] = fusion.trust_checks;
        record["trustDiagnostics"] = d["trustDiagnostics"];
        record["withinReceiverFov"] = std::isfinite(number(d, "rawAzimuthDeg"))
            ? Json(std::abs(number(d, "rawAzimuthDeg")) <= fusion.fov) : Json(nullptr);
        record["distanceCorrectionM"] = accepted ? Json(number(d, "distanceM") - number(d, "rawDistanceM")) : Json(nullptr);
        record["azimuthCorrectionDeg"] = accepted ? Json(wrap(number(d, "azimuthDeg") - number(d, "rawAzimuthDeg"))) : Json(nullptr);
        measurements.push_back(record);
    }
    const bool safety_changed = update_clearance(id, result->second, value, now);
    return before != revision() || r.measurement_completed || safety_changed;
}
void UwbPipeline::acknowledge_clearance(double now) {
    for (auto &s : clearance)
        if (s.stopped && !std::isfinite(s.acknowledged_at))
            s.acknowledged_at = now;
}
void UwbPipeline::expire_partial(double now, bool disconnect) {
    for (int id = 1; id <= 2; ++id) {
        auto &r = fusion.receivers[id - 1];
        if (!(r.pending_distance || r.pending_azimuth || r.pending_elevation)) continue;
        double first = INFINITY, last = -INFINITY;
        for (double at : {r.pending_distance ? r.distance_at : missing,
                          r.pending_azimuth ? r.azimuth_at : missing,
                          r.pending_elevation ? r.elevation_at : missing})
            if (std::isfinite(at)) { first = std::min(first, at); last = std::max(last, at); }
        if (!disconnect && now - first <= 100) continue;
        measurements.push_back({{"receiverId", id}, {"frameKey", r.frame_key},
            {"receivedAt", nullable(last)}, {"finalizedAt", now},
            {"status", "incomplete"}, {"reason", disconnect ? "session_closed" : "field_timeout"},
            {"rawDistanceM", r.pending_distance ? r.data["rawDistanceM"] : Json(nullptr)},
            {"rawAzimuthDeg", r.pending_azimuth ? r.data["rawAzimuthDeg"] : Json(nullptr)},
            {"rawElevationDeg", r.pending_elevation ? r.data["rawElevationDeg"] : Json(nullptr)}});
        r.pending_distance = r.pending_azimuth = r.pending_elevation = false;
        r.frame_rejected = true;
    }
}
bool UwbPipeline::update_clearance(int id, const std::string &field, double value, double now) {
    if (std::find(fusion.enabled.begin(), fusion.enabled.end(), id) == fusion.enabled.end())
        return false;
    auto &s = clearance.at(id - 1);
    const auto &r = fusion.receivers.at(id - 1);
    const auto &d = r.data;
    // Use admitted DIST packets immediately, even if EL/AZ are lost. Replayed
    // packets have already been rejected by fusion and never enter this path.
    const bool distance = field == "DIST" ||
        (field == "REJECTED" && d["invalidReason"] == "distance_out_of_range");
    if (distance && std::isfinite(value) && value >= 0 && value <= stop_distance) {
        s.distance = s.stopped ? std::min(s.distance, value) : value;
        s.stopped = true;
        s.observed_at = now;
        s.acknowledged_at = s.clear_since = s.clear_at = missing;
        s.clear_hits = 0;
        return true;
    }
    if (!s.stopped)
        return false;
    if (field == "REJECTED" || (field == "VALID" && d["valid"] == false)) {
        s.clear_since = s.clear_at = missing;
        s.clear_hits = 0;
    }
    if (!r.measurement_completed)
        return false;
    const bool safe = d["valid"] != false && (!fusion.gated || d["measurementStatus"] == "trusted") &&
        number(d, "rawDistanceM") >= resume_distance && number(d, "distanceM") >= resume_distance;
    if (!safe || !std::isfinite(s.acknowledged_at) || now <= s.acknowledged_at) {
        s.clear_since = s.clear_at = missing;
        s.clear_hits = 0;
        return false;
    }
    if (!std::isfinite(s.clear_at) || now - s.clear_at > 150 || now < s.clear_at) {
        s.clear_since = now;
        s.clear_hits = 0;
    }
    if (!std::isfinite(s.clear_at) || now > s.clear_at)
        ++s.clear_hits;
    s.clear_at = now;
    if (now - s.clear_since >= 500 && s.clear_hits >= 3) {
        s = Clearance{};
        return true;
    }
    return false;
}
Json UwbPipeline::payload(double now) {
    fusion.reference_heading = reference_heading;
    if (fusion.gated)
        for (int i = 0; i < 2; ++i)
            fusion.receivers[i].data["filterUncertaintyM"] =
                filters[i].result(now)["uncertainty_m"];
    const auto f = fusion.fused(now);
    const bool fresh = f["fresh"];
    Json p = {{"online", fresh},
              {"fresh", fresh},
              {"rawDistanceM", nullptr},
              {"rawAzimuthDeg", nullptr},
              {"rawElevationDeg", nullptr},
              {"measurementUpdatedAt", nullptr},
              {"source", fusion.enabled == std::vector<int>{1} ? "TELEM1_NAMED_VALUE_FLOAT"
                         : fusion.enabled == std::vector<int>{2}
                             ? "TELEM2_NAMED_VALUE_FLOAT"
                             : "TELEM1_TELEM2_NAMED_VALUE_FLOAT"},
              {"trustChecksEnabled", fusion.trust_checks},
              {"filter", fusion.gated ? (fusion.trust_checks ? "raw_validation_innovation_gate_cv_kalman_then_fusion"
                                                             : "complete_frame_cv_kalman_then_fusion")
                                      : "fast_per_receiver_kalman_then_body_xy_fusion"},
              {"measurementStatus", fresh ? "trusted" : "unavailable"},
              {"rejectionReason", nullptr},
              {"measurementNis", nullptr}};
    const std::vector<std::pair<std::string, std::string>> fields = {
        {"distance_m", "distanceM"},
        {"azimuth_deg", "azimuthDeg"},
        {"elevation_deg", "elevationDeg"},
        {"good_count", "goodCount"},
        {"updated_at", "updatedAt"},
        {"receiver_count", "receiverCount"},
        {"observing_receiver_count", "observingReceiverCount"},
        {"active_receiver_ids", "activeReceiverIds"},
        {"fusion_mode", "fusionMode"},
        {"bearing_disagreement_deg", "bearingDisagreementDeg"},
        {"position_disagreement_m", "positionDisagreementM"},
        {"relative_x_m", "relativeXM"},
        {"relative_y_m", "relativeYM"},
        {"sensors", "sensors"}};
    for (const auto &[from, to] : fields)
        p[to] = f[from];
    double newest = -1, accepted_at = 0, nis = missing;
    for (const auto &sensor : f["sensors"]) {
        const bool active =
            std::find(f["active_receiver_ids"].begin(), f["active_receiver_ids"].end(),
                      sensor["receiverId"]) != f["active_receiver_ids"].end();
        if (active)
            accepted_at = std::max(accepted_at, number(sensor, "measurementUpdatedAt", 0));
        if (!fresh && sensor["measurementStatus"] == "suspect") {
            p["measurementStatus"] = "suspect";
            if (p["rejectionReason"].is_null())
                p["rejectionReason"] = sensor["filterReason"];
        }
        const double n = number(sensor, "filterNis");
        if (std::isfinite(n) && (!fresh || active))
            nis = std::isfinite(nis) ? std::max(nis, n) : n;
        const double at = number(sensor, "receivedMeasurementAt", number(sensor, "updatedAt", 0));
        if (!sensor["rawDistanceM"].is_null() && !sensor["rawAzimuthDeg"].is_null() &&
            at > newest) {
            newest = at;
            for (const auto *key : {"rawDistanceM", "rawAzimuthDeg", "rawElevationDeg"})
                p[key] = sensor[key];
        }
    }
    p["measurementUpdatedAt"] = accepted_at > 0 ? Json(accepted_at) : Json(nullptr);
    p["measurementNis"] = nullable(nis);
    double sample_at = INFINITY;
    bool aligned = !f["active_receiver_ids"].empty();
    for (const auto &id : f["active_receiver_ids"]) {
        const auto &d = fusion.receivers[id.get<int>() - 1].data;
        aligned &= std::isfinite(number(d, "measurementSampleAt"));
        sample_at = std::min(sample_at, number(d, "measurementSampleAt", INFINITY));
    }
    if (aligned) {
        p["measurementSampleAt"] = sample_at;
        p["measurementHeadingDeg"] = nullable(reference_heading);
    }
    p["clearanceStop"] = false;
    p["clearanceDistanceM"] = p["clearanceObservedAt"] = nullptr;
    for (const auto &s : clearance)
        if (s.stopped) {
            p["clearanceStop"] = true;
            p["clearanceDistanceM"] = std::min(number(p, "clearanceDistanceM", s.distance), s.distance);
            p["clearanceObservedAt"] = std::max(number(p, "clearanceObservedAt", s.observed_at), s.observed_at);
        }
    return p;
}
} // namespace manta
