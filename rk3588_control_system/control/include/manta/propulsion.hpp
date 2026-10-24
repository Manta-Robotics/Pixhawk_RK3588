#pragma once
#include "common.hpp"

namespace manta {
double shape_axis(double value, double limit, double deadband = 0, double expo = 1);
class MotorLimiter {
  public:
    MotorLimiter(const Json &raw, bool traditional = false);
    Json reset(double now = 0);
    Json set_target(const Json &input);
    Json step(double now);
    Json propulsion(double now, const Json &result, const Json &telemetry, bool connected);
    Json config, target, output;
    double throttle_limit, steering_limit, throttle_span, steering_span, left = 0, right = 0,
                                                                         reverse_ready = 0;
    int last_direction = 0;

  private:
    bool traditional;
    double updated_at = 0, side_ready = 0, throttle_deadband, steering_deadband, expo;
    std::string intent_side = "both", active_side = "neutral";
};
} // namespace manta
