#pragma once
#include "common.hpp"
#include <set>

namespace manta {
std::set<std::string> required_parameters(const Json &values);
void validate_parameters(const Json &values, double speed_limit = 2);
double decode_axis(int axis, const Json &values, int channel);
int encode_axis(double value, const Json &values, int channel);
std::pair<int, int> acro_axes(double speed, double yaw, const Json &values, double speed_limit = 2,
                              double yaw_limit = 14);
std::pair<int, int> manual_axes(double throttle, double steering, const Json &config);
std::pair<int, int> motor_target_axes(int steering, int throttle, int pilot_type);
} // namespace manta
