#pragma once
#include "common.hpp"
#include <deque>

namespace manta {
// Keep two clocks: a session-fixed source timeline for filter/pose spacing,
// and a refreshed host mapping for freshness. Host/FCU oscillator skew (including
// VM clock corrections) must not accumulate into fictitious transport delay.
// Refreshing the mapping never changes timestamps of already admitted samples.
class UwbTiming {
  public:
    void reset() { poses.clear(); ready = false; }
    void synchronize(uint32_t boot, double received, double rtt) {
        const double midpoint = received - rtt / 2;
        if (!ready) {
            anchor = midpoint;
            ready = true;
        } else {
            anchor += int32_t(boot - origin);
        }
        origin = boot;
        // tc1 lies inside this authenticated round trip. Use its send bound,
        // not packet receipt, so real queued data cannot be made artificially
        // younger. A slower reply conservatively increases age by at most RTT.
        age_anchor = received - rtt;
        uncertainty = rtt + 1; // Includes source milliseconds quantization.
    }
    double mapped(uint32_t stamp) const {
        return ready ? anchor + int32_t(stamp - origin) : missing;
    }
    void attitude(uint32_t stamp, double yaw, double rate) {
        const double at = mapped(stamp);
        if (!std::isfinite(at)) return;
        const auto it = std::lower_bound(poses.begin(), poses.end(), at,
            [](const Pose &p, double t) { return p.at < t; });
        if (it != poses.end() && it->at == at) return;
        poses.insert(it, {at, yaw, rate});
        while (poses.size() > 256 || (!poses.empty() && poses.back().at - poses.front().at > 2000))
            poses.pop_front();
    }
    Json alignment(uint32_t stamp, double received) const {
        const double at = mapped(stamp);
        const double sample = ready ? age_anchor + int32_t(stamp - origin) : missing;
        Json out = {{"sourceTimeBootMs", stamp}, {"sourceTimeKind", "lua_processing"},
                    {"filterTimeMs", nullable(at)}, {"sampleAt", nullable(std::min(sample, received))},
                    {"receivedAt", received}, {"clockUncertaintyMs", uncertainty},
                    {"clockLeadMs", nullable(std::max(0., sample - received))},
                    {"clockOffsetCorrectionMs", nullable(age_anchor - anchor)},
                    {"transportDelayMs", nullable(std::max(0., received - sample))},
                    {"headingDeg", nullptr}, {"headingMethod", "unavailable"},
                    {"headingSpanMs", nullptr}};
        if (!std::isfinite(at) || poses.empty()) return out;
        const auto next = std::lower_bound(poses.begin(), poses.end(), at,
            [](const Pose &p, double t) { return p.at < t; });
        if (next != poses.end() && next->at == at) {
            out.update({{"headingDeg", next->yaw}, {"headingMethod", "exact"}, {"headingSpanMs", 0}});
        } else if (next != poses.begin()) {
            const auto &prior = *std::prev(next);
            if (next != poses.end() && next->at - prior.at <= 150) {
                const double fraction = (at - prior.at) / (next->at - prior.at);
                out.update({{"headingDeg", wrap(prior.yaw + fraction * wrap(next->yaw - prior.yaw))},
                            {"headingMethod", "interpolated"}, {"headingSpanMs", next->at - prior.at}});
            } else if (at - prior.at <= 100) {
                out.update({{"headingDeg", wrap(prior.yaw + prior.rate * (at - prior.at) / 1000)},
                            {"headingMethod", "yaw_rate_extrapolated"}, {"headingSpanMs", at - prior.at}});
            }
        }
        return out;
    }
  private:
    struct Pose { double at, yaw, rate; };
    bool ready = false;
    uint32_t origin = 0;
    double anchor = missing, age_anchor = missing, uncertainty = missing;
    std::deque<Pose> poses;
};
} // namespace manta
