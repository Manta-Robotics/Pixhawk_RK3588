#pragma once
#include "uwb_fusion.hpp"
#include <functional>

namespace manta {
class UwbPipeline {
  public:
    explicit UwbPipeline(const Json &config);
    bool update(const std::string &name, double value, double now, const Json &source,
                const Json &sequence = nullptr, double heading = missing,
                double heading_at = missing, const Json &wire_continuity = nullptr,
                const std::function<Json(uint32_t, double)> &align = {});
    Json payload(double now);
    Json measurements = Json::array();
    double reference_heading = missing;
    void acknowledge_clearance(double now);
    void expire_partial(double now, bool disconnect = false);
    UwbFusion fusion;

  private:
    bool raw_control = false;
    std::vector<UwbFilter> filters;
    // Translate host "now" to each filter's fixed source timeline for prediction.
    std::array<double, 2> filter_clock_offset{};
    struct Scalar {
        double estimate = missing, uncertainty = 1;
    };
    std::array<std::array<Scalar, 3>, 2> scalars;
    struct Clearance {
        bool stopped = false;
        double distance = missing, observed_at = missing, acknowledged_at = missing;
        double clear_since = missing, clear_at = missing;
        int clear_hits = 0;
    };
    std::array<Clearance, 2> clearance;
    double stop_distance, resume_distance;
    double bearing_sign, bearing_scale, bearing_offset;
    bool update_clearance(int id, const std::string &field, double value, double now);
};
} // namespace manta
