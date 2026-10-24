#include "manta/uwb_pipeline.hpp"

namespace manta {
static Json fusion_options(const Json &config) {
    auto options = config.value("uwb_dual", Json::object());
    const auto filter = config.value("uwb_measurement_filter", Json::object());
    options["require_gated_measurements"] = filter.value("enabled", true);
    if (options.contains("position_x_m"))
        options["receiver_position_x_m"] = options["position_x_m"];
    if (options.contains("position_y_m"))
        options["receiver_position_y_m"] = options["position_y_m"];
    return options;
}
UwbPipeline::UwbPipeline(const Json &config) : fusion(fusion_options(config)) {
    const auto f = config.value("uwb_measurement_filter", Json::object());
    const auto b = config.value("gimbal", Json::object()).value("beacon", Json::object());
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
                         const Json &sequence, double heading, double heading_at) {
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
    const auto result = fusion.update(name, value, now, source);
    if (!(std::isfinite(heading_at) && now >= heading_at && now - heading_at <= 700))
        heading = missing;
    if (!result) {
        if (fusion.gated && !std::isfinite(value) &&
            (parsed->second == "DIST" || parsed->second == "AZ" || parsed->second == "EL"))
            fusion.apply(id, filter.reject(now, "non_finite_measurement"));
    } else if (fusion.gated) {
        const auto &field = result->second;
        if (field == "REJECTED")
            fusion.apply(id, filter.reject(now, d["invalidReason"], heading));
        else if (field == "VALID" && d["valid"] == false)
            fusion.apply(id, filter.reject(now, "radio_invalid", heading));
        else if ((field == "DIST" || field == "AZ") && number(d, "receivedMeasurementAt") == now &&
                 !r.pending_distance && !r.pending_azimuth) {
            const auto accepted =
                std::abs(number(d, "rawAzimuthDeg")) > fusion.fov
                    ? filter.reject(now, "outside_receiver_fov", heading)
                    : filter.update(
                          {{"distance_m", d["rawDistanceM"]},
                           {"azimuth_deg", d["rawAzimuthDeg"]},
                           {"now_ms", now},
                           {"heading_deg", nullable(heading)},
                           {"valid", d["valid"] != false},
                           {"sequence_id", sequence.is_null() ? d["sequenceId"] : sequence},
                           {"tag_id", d["tagId"]}});
            fusion.apply(id, accepted);
        } else if (field == "EL" && std::abs(value) <= 90)
            fusion.set_filtered(id, "EL", value);
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
    return before != revision();
}
Json UwbPipeline::payload(double now) {
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
              {"filter", fusion.gated ? "raw_validation_innovation_gate_cv_kalman_then_fusion"
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
    return p;
}
} // namespace manta
