#pragma once
#include "common.hpp"
#include <chrono>

namespace manta {
Json normalize_follow_config(const Json &config);
Json traditional_guidance(const Json &config, double bearing, double yaw_rate, double speed = 0);
bool usable_estimate(const Json &estimate, double now, const Json &prediction);
Json zero_control(const std::string &reason, const Json &extra = Json::object());

class GpsSpeedLoop {
  public:
    explicit GpsSpeedLoop(Json config) : c(std::move(config)) { reset(); }
    void reset();
    std::string quality(double now, const Json &telemetry) const;
    Json update(double now, const Json &telemetry, double bearing, double range_rate,
                double closing, double applied, double policy_speed = missing, bool stop = false);
    void track_applied(double throttle, double dt);

  private:
    Json c;
    double good_since = missing, at = missing, integral = 0, target_radial = missing,
           measurement_at = missing, request = 0;
    bool active = false;
};

class RelativeNmpc {
  public:
    explicit RelativeNmpc(Json config) : c(std::move(config)) { reset(); }
    void reset();
    void track_applied(double throttle, double steering);
    Json solve(double now, const Json &telemetry, double range, double bearing,
               const Json &estimate, double reference_throttle, bool gps_available);

  private:
    using Controls = std::array<std::array<double, 2>, 4>;
    Json c;
    double at = missing, measurement_at = missing, model_surge = 0, applied_throttle = 0,
           applied_steering = 0;
    std::string identity;
    std::vector<Point> history;
    std::optional<Controls> plan;
    Controls project(const Controls &input, double dt) const;
    std::string observe(double now, const Json &telemetry, double range, double bearing,
                        const Json &estimate);
    std::pair<Vec, bool> relative_velocity(double yaw) const;
};

class RelativeControl {
  public:
    explicit RelativeControl(Json config) : c(std::move(config)), gps(c), nmpc(c) {}
    void reset();
    void pause();
    void track_applied(double throttle, double steering);
    Json update(double now, const Json &preflight, const Json &telemetry,
                const Json &estimate = nullptr, const Json &policy = nullptr);

  private:
    Json c;
    GpsSpeedLoop gps;
    RelativeNmpc nmpc;
    double at = missing, measurement_at = missing, distance = missing, range_rate = 0, cruise = 0,
           throttle = 0, steering = 0, bearing = missing;
};

class FollowController {
  public:
    explicit FollowController(const Json &raw)
        : config(normalize_follow_config(raw)), relative(config) {
        reset();
    }
    virtual ~FollowController() = default;
    virtual void reset();
    virtual Json preflight(double now, const Json &telemetry, bool connected,
                           const std::string &mode = "auto", const Json &estimate = nullptr) const;
    virtual Json update(double now, const Json &telemetry, bool connected,
                        const std::string &mode = "auto", const Json &estimate = nullptr,
                        const Json &motion = nullptr);
    Json config;
    RelativeControl relative;

  private:
    friend class IntermittentFollow;
    double previous_distance = missing, previous_bearing = missing, previous_at = missing,
           range_rate = 0, bearing_rate = 0, target_radial = 0, range_integral = 0;
    double filtered_bearing = missing, steering_command = 0, steering_at = missing, reverse_at = 0,
           cross_integral = 0, throttle_command = 0, throttle_at = missing;
    int last_steering_sign = 0, reverse_sign = 0;
    bool hard_stop = false, distance_recovery = false;
};
} // namespace manta
