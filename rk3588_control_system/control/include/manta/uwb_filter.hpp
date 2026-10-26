#pragma once
#include <memory>
#include "common.hpp"

namespace manta {
// Native equivalents of the Python admission filter. Clocks are supplied by
// the caller so replay and the production loop execute the same algorithms.
class AngularFilter {
  public:
    double angle = missing, at = missing, rate = 0, stable_since = missing;
    std::string mode = "acquiring";
    double stationary_tau, moving_tau;
    std::vector<Point> points;
    AngularFilter(double stationary = .4, double moving = .01)
        : stationary_tau(stationary), moving_tau(moving) {}
    void reset();
    double observe(double raw, double posterior, double now, double noise);
};
class OcclusionGuard {
  public:
    double speed_limit, step_margin, angle_margin, recovery_ms;
    std::optional<Point> anchor;
    std::vector<Point> history;
    Vec velocity, reference;
    int bad_count = 0, return_hits = 0;
    double return_at = missing, return_last_at = missing;
    Json diagnostics = {{"state", "clear"}, {"reason", nullptr}};
    std::vector<Point> reacquisition;
    double candidate_last_at = missing;
    Vec candidate_last_position, candidate_velocity;
    OcclusionGuard(double speed, double distance_noise, double angle_noise, double recovery = 250);
    void reset();
    bool check(Vec position, double now);
    void interrupt_confirmation();
    void radio_loss();
    void accept(Vec position, double now);
    bool confirm_new_track(Vec position, double now);
};
struct Axis {
    double position = 0, velocity = 0, p00 = 0, p01 = 0, p11 = 0;
    Axis() = default;
    Axis(double p, double variance, double velocity_variance)
        : position(p), p00(variance), p11(velocity_variance) {}
    Axis predicted(double dt, double q) const;
    void correct(double measurement, double variance);
};
class UwbFilter {
  public:
    Json config;
    double minimum, maximum, sign, scale, offset, mount;
    std::optional<std::array<Axis, 2>> axes;
    double at = missing, last_accepted = missing, last_received = missing;
    Json last_sequence = nullptr, tag = nullptr;
    int accepted_count = 0, rejected_count = 0, recovery_count = 0;
    std::string status = "unavailable", reason = "not_initialized";
    double nis = missing, innovation = missing;
    std::vector<Point> raw_window;
    Json recovery_diagnostics = {{"reason", "warming_up"}, {"hits", 0}};
    AngularFilter angular;
    OcclusionGuard occlusion;
    explicit UwbFilter(const Json &options = Json::object());
    Json update(const Json &observation);
    Json reject(double now, const std::string &reason, double heading = missing, bool diagnose = true);
    Json result(double now, double heading = missing) const;

  private:
    std::unique_ptr<UwbFilter> diagnostic_filter;
    struct Candidate {
        int hits;
        double residual;
        Fit model;
        double p00, p11, uncertainty;
    };
    void predict(double now);
    void update_angle(Vec measurement, double now);
    std::optional<Candidate> raw_candidate(Vec measurement, double variance, double now);
};
} // namespace manta
