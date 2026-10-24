#pragma once
#include "prediction.hpp"
#include <stdexcept>

namespace manta {
// Validate the control-owned part of the shared configuration without needing
// a Web/Node process. Normalization remains identical to the reference ports.
inline void validate_control_config(const Json &root) {
    const auto object = [](const Json &parent, const char *key) {
        auto value = parent.value(key, Json::object());
        if (!value.is_object())
            throw std::invalid_argument(std::string(key) + " must be an object");
        return value;
    };
    const auto bound = [](const Json &c, const char *key, double low, double high) {
        if (c.contains(key) &&
            (!std::isfinite(number(c, key)) || number(c, key) < low || number(c, key) > high))
            throw std::invalid_argument(std::string(key) + " is outside its finite control range");
    };
    const auto boolean = [](const Json &c, const char *key) {
        if (c.contains(key) && !c[key].is_boolean())
            throw std::invalid_argument(std::string(key) + " must be boolean");
    };
    const auto f = object(root, "uwb_follow"), p = prediction_config(object(f, "prediction")),
               filter = object(root, "uwb_measurement_filter");
    for (const auto *key :
         {"enabled", "auto_resume_after_uwb_timeout", "waiting_drift_hold", "far_pursuit_enabled",
          "commissioning_mode", "hard_stop_enabled", "close_approach_enabled"})
        boolean(f, key);
    boolean(p, "enabled");
    for (const auto *key : {"enabled", "adaptive_enabled", "occlusion_enabled"})
        boolean(filter, key);
    const auto execution = f.value("execution_mode", std::string("manual"));
    if (execution != "manual" && execution != "acro" && execution != "relative_imu")
        throw std::invalid_argument("unsupported follow execution_mode");
    for (const auto *key : {"max_speed_mps", "commissioning_speed_mps"})
        bound(f, key, .1, execution == "acro" ? 2 : 5);
    struct Limit {
        const char *key;
        double low, high;
    };
    for (const auto &[key, low, high] : std::vector<Limit>{{"horizon_ms", 100, 2000},
                                                           {"wait_timeout_ms", 500, 10000},
                                                           {"pose_timeout_ms", 100, 1000},
                                                           {"min_samples", 3, 10},
                                                           {"measurement_gap_ms", 50, 1000},
                                                           {"measurement_noise_m", .01, .5},
                                                           {"velocity_uncertainty_mps", .05, 3},
                                                           {"acceleration_bound_mps2", .1, 5},
                                                           {"max_uncertainty_m", .05, 1},
                                                           {"max_target_speed_mps", .1, 5},
                                                           {"innovation_gate_m", .1, 2},
                                                           {"max_speed_mps", .05, 1},
                                                           {"max_throttle_percent", 1, 25},
                                                           {"max_steering_deg", 1, 15},
                                                           {"clearance_m", .5, 5},
                                                           {"braking_deceleration_mps2", .05, 1},
                                                           {"recovery_blend_ms", 300, 3000}})
        bound(p, key, low, high);
    if (number(p, "min_samples") != std::trunc(number(p, "min_samples")) ||
        number(p, "horizon_ms") >= number(p, "wait_timeout_ms") ||
        number(p, "measurement_gap_ms") > number(f, "uwb_timeout_ms", 700) ||
        number(p, "measurement_noise_m") >= number(p, "max_uncertainty_m"))
        throw std::invalid_argument("inconsistent prediction bounds");
    for (const auto &[key, low, high] : std::vector<Limit>{{"distance_std_m", .02, 1},
                                                           {"angle_std_deg", .2, 15},
                                                           {"acceleration_std_mps2", .1, 10},
                                                           {"initial_velocity_std_mps", .1, 6},
                                                           {"nis_threshold", 4, 25},
                                                           {"absolute_gate_m", .1, 2},
                                                           {"max_relative_speed_mps", .5, 10},
                                                           {"rejection_covariance_scale", 1, 2},
                                                           {"stationary_time_constant_s", .1, 2},
                                                           {"moving_time_constant_s", .01, .3}})
        bound(filter, key, low, high);
    if (number(filter, "moving_time_constant_s", .01) >
        number(filter, "stationary_time_constant_s", .4))
        throw std::invalid_argument("inconsistent angular filter time constants");
    const auto gps = object(f, "gps_speed_closed_loop"), nmpc = object(f, "relative_nmpc"),
               reject = object(f, "reject_nmpc");
    for (const auto *child : {&gps, &nmpc, &reject})
        boolean(*child, "enabled");
    if ((gps.value("enabled", false) || nmpc.value("enabled", false)) &&
        execution != "relative_imu")
        throw std::invalid_argument("relative controller requires relative_imu execution");
    for (const auto &[key, low, high] : std::vector<Limit>{{"acquire_ms", 1000, 30000},
                                                           {"kp", 0, 100},
                                                           {"ki", 0, 100},
                                                           {"maximum_observed_speed_mps", .1, 30}})
        bound(gps, key, low, high);
    for (const auto &[key, low, high] : std::vector<Limit>{{"horizon_s", .3, 3},
                                                           {"solve_budget_ms", 1, 20},
                                                           {"surge_time_constant_s", .2, 3},
                                                           {"yaw_time_constant_s", .1, 2},
                                                           {"surge_map_exponent", .3, 1.5},
                                                           {"model_cruise_speed_mps", .1, 2},
                                                           {"model_position_error_mps", .05, 1},
                                                           {"steering_share", .1, .8}})
        bound(nmpc, key, low, high);
    bound(reject, "solve_budget_ms", 1, 25);
    for (const auto *key : {"rover_left_channel", "rover_right_channel"}) {
        bound(root, key, 1, 4);
        if (root.contains(key) && number(root, key) != std::trunc(number(root, key)))
            throw std::invalid_argument("motor channel must be integral");
    }
    if (number(root, "rover_throttle_min", -100) >= number(root, "rover_throttle_max", 100) ||
        number(root, "rover_steering_min", -45) >= number(root, "rover_steering_max", 45))
        throw std::invalid_argument("invalid Rover input ranges");
    bound(root, "rover_manual_throttle_limit_percent", 0, 100);
    bound(root, "rover_manual_steering_limit_percent", 0, 100);
}
} // namespace manta
