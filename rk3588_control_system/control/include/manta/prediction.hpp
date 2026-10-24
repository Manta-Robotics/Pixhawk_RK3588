#pragma once
#include "follow_control.hpp"
#include <memory>

namespace manta {
Json prediction_config(const Json &raw);
Json target_identity(const Json &uwb);
std::string measurement_key(const Json &uwb);
double sensor_uncertainty(const Json &uwb);
bool fresh_time(double at, double now, double timeout);
bool quarantined(const Json &sensor);
inline Vec vector_value(const Json &j) {
    return {number(j, "x"), number(j, "y")};
}
inline Json vector_json(Vec v) {
    return {{"x", v.x}, {"y", v.y}};
}
inline Vec rotate_vector(Vec p, double angle) {
    return {p.x * std::cos(angle) - p.y * std::sin(angle),
            p.x * std::sin(angle) + p.y * std::cos(angle)};
}
class TargetEstimator;
class RawRecovery {
  public:
    explicit RawRecovery(Json config) : config(std::move(config)) { reset(); }
    void reset();
    bool observe(double now, const Json &telemetry, TargetEstimator &estimator, const Json &anchor,
                 bool allow_handoff = true);
    Json diagnostic, fit_result;

  private:
    Json config;
    std::vector<Json> window;
    std::string token;
    double at = missing;
};
class TargetEstimator {
  public:
    TargetEstimator(const Json &raw, const Json &calibration)
        : config(prediction_config(raw)), calibration(calibration), raw_window(config) {
        reset();
    }
    void reset();
    Json pose(double now, const Json &telemetry);
    Json sample(const Json &uwb, const Json &pose, double now) const;
    Vec projected(const Json &sample, double now) const;
    Vec observe_motion(const Json &sample);
    void accept(const Json &sample, const Json &previous = nullptr,
                double recovered_samples = missing);
    Json update(double now, const Json &telemetry);
    Json config, calibration, last, last_pose, recovery, recovery_anchor, motion_diagnostic;
    RawRecovery raw_window;
    double last_seen_at = missing, loss_at = missing, recovered_at = missing;
    std::string last_seen_token;

  private:
    std::vector<Json> motion_history;
    Vec motion_velocity{};
    double motion_at = missing, motion_trusted_at = missing;
};
class IntermittentFollow {
  public:
    IntermittentFollow(const Json &prediction, const Json &calibration)
        : config(prediction_config(prediction)), estimator(config, calibration),
          reject_nmpc(calibration.value("reject_nmpc", Json::object()).value("enabled", false)) {
        reset();
    }
    void reset();
    Json update(FollowController &controller, double now, const Json &telemetry, bool connected,
                const std::string &mode);
    Json neutral(const std::string &reason, const Json &estimate, bool waiting);
    Json govern_acro(const Json &result, double speed_limit, double yaw_limit, double steering_span,
                     double throttle_span);
    Json govern_drive(const Json &output, const Json &result);
    Json config;
    TargetEstimator estimator;

  private:
    bool reject_nmpc, recovery_step_good = false;
    std::string control_mode = "relative_manual";
    double last_throttle = 0, last_steering = 0, last_forward = 0, raw_recovered_at = missing,
           last_now = missing, recovery_cycle_at = missing, recovery_good_ms = 0,
           recovery_step_at = missing, last_position_good_at = missing, live_command_at = missing,
           live_command_speed = 0;
    Json raw_recovery, target_id, admitted_drive;
    double recovery_scale(double now, const Json &estimate);
    Json coast(double now, const Json &estimate, const Json &telemetry,
               FollowController &controller);
};
} // namespace manta
