#include "manta/rover_mapping.hpp"
#include <stdexcept>

namespace manta {
static const std::array<std::string, 5> rc_fields = {"MIN", "MAX", "TRIM", "DZ", "REVERSED"};
static bool channel_valid(double value) {
    return std::isfinite(value) && value == std::trunc(value) && value >= 1 && value <= 16;
}
std::set<std::string> required_parameters(const Json &v) {
    std::set<std::string> names = {"PILOT_STEER_TYPE", "SPEED_MAX",      "CRUISE_SPEED",
                                   "CRUISE_THROTTLE",  "ACRO_TURN_RATE", "RCMAP_ROLL",
                                   "RCMAP_THROTTLE"};
    for (const auto *key : {"RCMAP_ROLL", "RCMAP_THROTTLE"})
        if (channel_valid(number(v, key)))
            for (const auto &field : rc_fields)
                names.insert("RC" + std::to_string(int(number(v, key))) + "_" + field);
    return names;
}
void validate_parameters(const Json &v, double limit) {
    const auto fail = [](const char *reason) { throw std::invalid_argument(reason); };
    for (const auto &key : required_parameters(v))
        if (!v.contains(key))
            fail("acro_parameters_missing");
    for (const auto &[key, value] : v.items())
        if (!value.is_number() || !std::isfinite(value.get<double>()))
            fail("acro_parameters_invalid");
    const double pilot = number(v, "PILOT_STEER_TYPE");
    if (pilot != 0 && pilot != 1)
        fail("acro_pilot_type_unsupported");
    if (!(number(v, "SPEED_MAX") > 0 && number(v, "SPEED_MAX") <= limit &&
          number(v, "CRUISE_THROTTLE") >= 5 && number(v, "CRUISE_THROTTLE") <= 100))
        fail("acro_speed_limit_not_configured");
    if (!(number(v, "CRUISE_SPEED") > 0 && number(v, "ACRO_TURN_RATE") > 0))
        fail("acro_parameters_invalid");
    for (const auto *key : {"RCMAP_ROLL", "RCMAP_THROTTLE"}) {
        const auto channel = number(v, key);
        if (!channel_valid(channel))
            fail("acro_rcmap_invalid");
        const auto prefix = "RC" + std::to_string(int(channel)) + "_";
        const double low = number(v, prefix + "MIN"), high = number(v, prefix + "MAX"),
                     trim = number(v, prefix + "TRIM"), dz = number(v, prefix + "DZ"),
                     reverse = number(v, prefix + "REVERSED");
        if (!(800 <= low && low < trim && trim < high && high <= 2200 && 0 <= dz &&
              dz < std::min(trim - low, high - trim) && (reverse == 0 || reverse == 1)))
            fail("acro_rc_calibration_invalid");
    }
}
double decode_axis(int axis, const Json &v, int channel) {
    const auto prefix = "RC" + std::to_string(channel) + "_";
    const double low = number(v, prefix + "MIN"), high = number(v, prefix + "MAX"),
                 trim = number(v, prefix + "TRIM"), dz = number(v, prefix + "DZ");
    const int pwm = int(low + (high - low) * (axis + 1000) / 2000);
    const double result = pwm > trim + dz   ? (pwm - trim - dz) / (high - trim - dz)
                          : pwm < trim - dz ? (pwm - trim + dz) / (trim - dz - low)
                                            : 0;
    return number(v, prefix + "REVERSED") != 0 ? -result : result;
}
int encode_axis(double value, const Json &v, int channel) {
    const auto prefix = "RC" + std::to_string(channel) + "_";
    const double low = number(v, prefix + "MIN"), high = number(v, prefix + "MAX"),
                 trim = number(v, prefix + "TRIM"), dz = number(v, prefix + "DZ");
    const double physical = number(v, prefix + "REVERSED") != 0 ? -value : value;
    const double pwm = physical == 0  ? trim
                       : physical > 0 ? trim + dz + physical * (high - trim - dz)
                                      : trim - dz + physical * (trim - dz - low);
    const double ideal = (pwm - low) * 2000 / (high - low) - 1000;
    std::optional<int> best;
    double error = std::numeric_limits<double>::infinity();
    for (int delta = -3; delta <= 4; ++delta) {
        const int axis = int(clamp(std::floor(ideal) + delta, -1000, 1000));
        const double decoded = decode_axis(axis, v, channel), e = std::abs(decoded - value);
        if (std::abs(decoded) <= std::abs(value) + 1e-9 && decoded * value >= 0 && e < error) {
            best = axis;
            error = e;
        }
    }
    if (!best)
        throw std::invalid_argument("acro_rc_quantization_failed");
    return *best;
}
std::pair<int, int> acro_axes(double speed, double yaw, const Json &v, double limit,
                              double yaw_limit) {
    validate_parameters(v, limit);
    if (!std::isfinite(speed) || !std::isfinite(yaw))
        throw std::invalid_argument("acro_target_invalid");
    const double throttle = clamp(clamp(speed, -limit, limit) / number(v, "SPEED_MAX"), -1, 1);
    double steering = clamp(clamp(yaw, -yaw_limit, yaw_limit) / number(v, "ACRO_TURN_RATE"), -1, 1);
    if (throttle < 0)
        steering = -steering;
    return {encode_axis(steering, v, int(number(v, "RCMAP_ROLL"))),
            encode_axis(throttle, v, int(number(v, "RCMAP_THROTTLE")))};
}
std::pair<int, int> manual_axes(double throttle, double steering, const Json &c) {
    if (!std::isfinite(throttle) || !std::isfinite(steering))
        throw std::invalid_argument("non_finite_drive");
    const double tmin = number(c, "rover_throttle_min", -100),
                 tmax = number(c, "rover_throttle_max", 100),
                 smin = number(c, "rover_steering_min", -45),
                 smax = number(c, "rover_steering_max", 45);
    if (tmax <= tmin || smax <= smin)
        throw std::invalid_argument("invalid_rover_range");
    return {int(clamp(
                std::nearbyint(-1000 + (clamp(steering, smin, smax) - smin) * 2000 / (smax - smin)),
                -1000, 1000)),
            int(clamp(
                std::nearbyint(-1000 + (clamp(throttle, tmin, tmax) - tmin) * 2000 / (tmax - tmin)),
                -1000, 1000))};
}
std::pair<int, int> motor_target_axes(int steering, int throttle, int pilot) {
    if (steering == 0 && throttle == 0)
        return {0, 0};
    if (pilot < 0 || pilot > 3)
        throw std::invalid_argument("pilot_steer_type_missing");
    if ((pilot == 0 || pilot == 2) && throttle < 0)
        steering = -steering;
    if (pilot == 1)
        return {int(clamp(throttle + steering, -1000, 1000)),
                int(clamp(throttle - steering, -1000, 1000))};
    return {steering, throttle};
}
} // namespace manta
