#include "manta/propulsion.hpp"
#include <stdexcept>

namespace manta {
namespace {
int direction(double v) {
    return (v > 0) - (v < 0);
}
double toward_motor(double v, double t, double d) {
    return v < t ? std::min(t, v + d) : std::max(t, v - d);
}
} // namespace
Json allocate_forward_drive(double throttle, double steering, double throttle_span,
                            double steering_span, double motor_limit) {
    if (!std::isfinite(throttle) || !std::isfinite(steering) ||
        !std::isfinite(throttle_span) || !std::isfinite(steering_span) ||
        !std::isfinite(motor_limit) || throttle_span <= 0 || steering_span <= 0 || motor_limit < 0)
        throw std::invalid_argument("invalid_forward_allocation");
    const double requested = clamp(throttle, 0, motor_limit),
                 turn = steering * throttle_span / steering_span;
    // First reserve the requested differential, then reduce common thrust.
    // The 0.8 share keeps both motors forward, including after saturation.
    const double limit = std::min(.8 * requested, motor_limit * .8 / 1.8),
                 difference = clamp(turn, -limit, limit),
                 common = std::min(requested, motor_limit - std::abs(difference));
    return {{"throttle", common}, {"steering", difference * steering_span / throttle_span},
            {"requestedThrottle", throttle}, {"requestedSteering", steering},
            {"leftDemandPercent", common + difference}, {"rightDemandPercent", common - difference},
            {"turnHeadroomPercent", motor_limit - common},
            {"throttleReducedForTurn", common < requested - 1e-9},
            {"steeringLimited", std::abs(difference - turn) > 1e-9}};
}
double shape_axis(double value, double limit, double deadband, double expo) {
    limit = std::max(.001, std::abs(limit));
    deadband = clamp(std::abs(deadband), 0, limit * .95);
    const double magnitude = clamp(std::abs(value), 0, limit);
    if (magnitude <= deadband)
        return 0;
    return direction(value) * limit *
           std::pow((magnitude - deadband) / (limit - deadband), std::max(1.0, expo));
}
MotorLimiter::MotorLimiter(const Json &raw, bool is_traditional)
    : throttle_limit(std::max(0.0, number(raw, "throttle_limit", 100))),
      steering_limit(std::max(0.0, number(raw, "steering_limit", 45))),
      throttle_span(std::max(1.0, number(raw, "throttle_span", 100))),
      steering_span(std::max(is_traditional ? 1.0 : 0.0, number(raw, "steering_span", 45))),
      traditional(is_traditional), throttle_deadband(number(raw, "throttle_deadband", 4)),
      steering_deadband(number(raw, "steering_deadband", 2)), expo(number(raw, "input_expo", 1.4)) {
    config = {{"throttleAccelPerS", std::max(1.0, number(raw, "throttle_accel_percent_s", 40))},
              {"throttleDecelPerS", std::max(1.0, number(raw, "throttle_decel_percent_s", 80))},
              {"reverseNeutralMs", std::max(0.0, number(raw, "reverse_neutral_ms", 300))},
              {"maximumDtS", clamp(number(raw, "maximum_dt_s", .1), .02, 1)}};
    if (traditional)
        config["steeringShare"] = .8;
    else
        config.update(
            {{"steeringRatePerS", 0},
             {"sideSwitchNeutralMs", std::max(0.0, number(raw, "side_switch_neutral_ms", 150))},
             {"powerMode",
              raw.value("power_mode", std::string()) == "throttle" ? "throttle" : "vector"},
             {"turnEnter", .12},
             {"turnExit", .06}});
    reset();
}
Json MotorLimiter::reset(double now) {
    target = output = {{"throttle", 0}, {"steering", 0}};
    left = right = 0;
    updated_at = now;
    last_direction = 0;
    reverse_ready = side_ready = 0;
    intent_side = "both";
    active_side = "neutral";
    return output;
}
Json MotorLimiter::set_target(const Json &input) {
    const double throttle = number(input, "throttle", 0), steering = number(input, "steering", 0);
    target = {
        {"throttle", traditional ? clamp(throttle, -throttle_limit, throttle_limit) : throttle},
        {"steering", traditional ? clamp(steering, -steering_limit, steering_limit) : steering}};
    if (!traditional) {
        const double horizontal =
            number(input, "rawSteering", steering) / std::max(1.0, steering_limit);
        if (throttle == 0 && steering == 0) {
            intent_side = "both";
            return target;
        }
        if (horizontal >= number(config, "turnEnter"))
            intent_side = "left";
        else if (horizontal <= -number(config, "turnEnter"))
            intent_side = "right";
        else if (std::abs(horizontal) <= number(config, "turnExit"))
            intent_side = "both";
    }
    return target;
}
Json MotorLimiter::step(double now) {
    const double dt = clamp((now - updated_at) / 1000, 0, number(config, "maximumDtS"));
    updated_at = now;
    if (dt == 0)
        return output;
    const double throttle = clamp(number(target, "throttle"), -throttle_limit, throttle_limit),
                 steering = clamp(number(target, "steering"), -steering_limit, steering_limit);
    double requested_left = 0, requested_right = 0;
    bool turning = false;
    int sign = 0;
    if (traditional) {
        sign = direction(throttle);
        const double turn = clamp(steering / steering_span * throttle_span,
                                  -std::abs(throttle) * .8, std::abs(throttle) * .8);
        requested_left = throttle + turn;
        requested_right = throttle - turn;
        const double maximum = std::max(std::abs(requested_left), std::abs(requested_right)),
                     scale = maximum > throttle_limit ? throttle_limit / maximum : 1;
        requested_left *= scale;
        requested_right *= scale;
    } else {
        turning = intent_side != "both";
        const double throttle_ratio = throttle_limit > 0 ? throttle / throttle_limit : 0,
                     turn_ratio = turning && steering_limit > 0 ? steering / steering_limit : 0;
        const double base = std::min(1.0, std::hypot(throttle_ratio, turn_ratio)) * throttle_limit;
        if (base != 0) {
            const int travel = turning ? 1 : direction(throttle);
            const double difference =
                             turning && steering_span > 0
                                 ? direction(steering) *
                                       std::min(std::abs(steering / steering_span * throttle_span),
                                                base * .8)
                                 : 0,
                         scale = std::min(1.0, throttle_limit / (base + std::abs(difference)));
            requested_left = (travel * base + difference) * scale;
            requested_right = (travel * base - difference) * scale;
        }
        sign = direction(requested_left != 0 ? requested_left : requested_right);
    }
    const bool was_moving = left != 0 || right != 0,
               reversing = sign != 0 && last_direction != 0 && sign != last_direction;
    if (!traditional && reversing && turning && was_moving) {
        left = right = 0;
        reverse_ready = now + number(config, "reverseNeutralMs");
    }
    const bool moving = left != 0 || right != 0;
    if (!traditional && reversing && was_moving && !moving)
        reverse_ready = now + number(config, "reverseNeutralMs");
    const bool hold = reversing && (moving || now < reverse_ready);
    const auto update_motor = [&](double current, double requested) {
        const double target = hold || now < side_ready ? 0 : requested;
        const bool slowing = (!traditional && target == 0) || std::abs(target) < std::abs(current);
        return toward_motor(current, target,
                            number(config, slowing ? "throttleDecelPerS" : "throttleAccelPerS") *
                                dt);
    };
    left = update_motor(left, requested_left);
    right = update_motor(right, requested_right);
    const bool now_moving = left != 0 || right != 0;
    if (was_moving && !now_moving)
        reverse_ready = now + number(config, "reverseNeutralMs");
    if (now_moving && !hold)
        last_direction = sign;
    output = {{"throttle", (left + right) / 2},
              {"steering", (left - right) / 2 * steering_span / throttle_span}};
    return output;
}
Json MotorLimiter::propulsion(double now, const Json &result, const Json &t, bool connected) {
    std::string reason;
    auto mode = t.value("flightMode", std::string());
    for (auto &ch : mode)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    const auto pilot = t.value("pilotSteering", Json::object());
    const double pilot_type = number(pilot, "type"), throttle = number(result, "throttlePercent"),
                 steering = number(result, "steeringDegrees");
    if (!connected)
        reason = "pixhawk_offline";
    else if (mode != "MANUAL")
        reason = "manual_required";
    else if (!t.value("armed", false))
        reason = "armed_required";
    else if (!pilot.value("ready", false) || !std::isfinite(pilot_type) ||
             pilot_type != std::trunc(pilot_type) || pilot_type < 0 || pilot_type > 3)
        reason = "pilot_steering_not_ready";
    else if (!result.value("safe", false))
        reason = result.value("reason", std::string("follow_unsafe"));
    if ((!std::isfinite(throttle) || !std::isfinite(steering)) && reason.empty())
        reason = "invalid_propulsion_target";
    if (!reason.empty() || result.value("reason", std::string()) == "holding_distance") {
        auto out = reset(now);
        out.update({{"safe", reason.empty()}, {"reason", reason.empty() ? "neutral" : reason}});
        return out;
    }
    if (traditional) {
        if (throttle == 0) {
            const int previous = last_direction;
            const double ready =
                left != 0 || right != 0 ? now + number(config, "reverseNeutralMs") : reverse_ready;
            auto out = reset(now);
            last_direction = previous;
            reverse_ready = ready;
            out.update({{"safe", true}, {"reason", "neutral"}});
            return out;
        }
        set_target({{"throttle", throttle}, {"steering", throttle < 0 ? 0 : steering}});
        auto out = step(now);
        out.update({{"safe", true}, {"reason", "tracking"}});
        return out;
    }
    const double bounded_throttle = clamp(throttle, -throttle_limit, throttle_limit),
                 bounded_steering = clamp(steering, -steering_limit, steering_limit);
    set_target({{"throttle", shape_axis(bounded_throttle, throttle_limit, throttle_deadband, expo)},
                {"steering", shape_axis(bounded_steering, steering_limit, steering_deadband, expo)},
                {"rawSteering", bounded_steering}});
    auto out = step(now);
    out.update({{"safe", true}, {"reason", "tracking"}, {"propulsion", "dual_thrust"}});
    return out;
}
} // namespace manta
