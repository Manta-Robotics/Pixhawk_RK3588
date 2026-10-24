#pragma once
#include "uwb_fusion.hpp"

namespace manta {
class UwbPipeline {
  public:
    explicit UwbPipeline(const Json &config);
    bool update(const std::string &name, double value, double now, const Json &source,
                const Json &sequence = nullptr, double heading = missing,
                double heading_at = missing);
    Json payload(double now);
    UwbFusion fusion;

  private:
    std::vector<UwbFilter> filters;
    struct Scalar {
        double estimate = missing, uncertainty = 1;
    };
    std::array<std::array<Scalar, 3>, 2> scalars;
};
} // namespace manta
