#pragma once
#include "uwb_filter.hpp"
#include <map>

namespace manta {
struct Receiver {
    Json data;
    bool pending_distance = false, pending_azimuth = false, pending_elevation = false;
    std::map<std::pair<int, int>, uint32_t> newest_samples;
    std::map<std::pair<int, int>, bool> strict_sources;
    int legacy_step = 0;
    Json wire_continuity = nullptr;
    uint32_t wire_stamp = 0;
    Json frame_key = nullptr;
    bool frame_rejected = false, frame_completed = false, measurement_completed = false;
    double distance_at = missing, azimuth_at = missing, elevation_at = missing;
    std::vector<double> distances, azimuths;
};
class UwbFusion {
  public:
    std::array<Receiver, 2> receivers;
    std::vector<int> enabled;
    bool gated, trust_checks;
    double reference_heading = missing, heading_scale = 1;
    double freshness, fov, disagreement, position_disagreement, minimum, maximum, confidence,
        confirm_ms, blend_ms;
    explicit UwbFusion(const Json &options = Json::object());
    static std::optional<std::pair<int, std::string>> parse_name(std::string name);
    int legacy_receiver(const Json &source);
    std::optional<std::pair<int, std::string>> update(const std::string &name, double value,
                                                      double now, const Json &source = nullptr,
                                                      const Json &sample_id = nullptr,
                                                      const Json &wire_continuity = nullptr);
    void apply(int id, const Json &result);
    void set_filtered(int id, const std::string &field, double value);
    Json fused(double now);

  private:
    std::map<std::pair<int, int>, int> sources;
    int selected = 0, challenger = 0;
    double challenger_since = missing, blend_started = missing;
    std::optional<Vec> blend_from, last_output;
    bool complete(const Receiver &r) const;
    bool fresh(const Receiver &r, double now) const;
    double coverage(const Receiver &r) const;
    double stability(const Receiver &r) const;
    Vec position(const Receiver &r) const;
    double bearing(const Receiver &r) const;
    Json telemetry(const Receiver &r, double now) const;
};
} // namespace manta
