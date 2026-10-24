#include "manta/mavlink_core.hpp"
#include <cctype>
#include <cstring>

namespace manta {
namespace {
const std::map<int, std::string> modes = {
    {0, "MANUAL"}, {1, "ACRO"},       {3, "STEERING"}, {4, "HOLD"},         {5, "LOITER"},
    {6, "FOLLOW"}, {7, "SIMPLE"},     {8, "DOCK"},     {9, "CIRCLE"},       {10, "AUTO"},
    {11, "RTL"},   {12, "SMART_RTL"}, {15, "GUIDED"},  {16, "INITIALISING"}};
const std::array<std::string, 7> poses = {"",         "LEVEL",  "LEFT", "RIGHT",
                                          "NOSEDOWN", "NOSEUP", "BACK"};
const std::array<std::string, 7> pose_hints = {
    "",
    "Place the rover level and keep it still, then confirm the pose in the UI.",
    "Place the rover with the left side down, keep it still, then confirm the pose in the UI.",
    "Place the rover with the right side down, keep it still, then confirm the pose in the UI.",
    "Place the rover nose down and vertical, then confirm the pose in the UI.",
    "Place the rover nose up and vertical, then confirm the pose in the UI.",
    "Place the rover tail down and vertical, then confirm the pose in the UI."};
std::string result_name(int value) {
    static const std::array<std::string, 10> names = {
        "ACCEPTED",         "TEMPORARILY_REJECTED",
        "DENIED",           "UNSUPPORTED",
        "FAILED",           "IN_PROGRESS",
        "CANCELLED",        "COMMAND_LONG_ONLY",
        "COMMAND_INT_ONLY", "COMMAND_UNSUPPORTED_MAV_FRAME"};
    return value >= 0 && value < int(names.size()) ? "MAV_RESULT_" + names[value]
                                                   : std::to_string(value);
}
std::string fixed_string(const char *value, size_t count) {
    return std::string(value, strnlen(value, count));
}
bool fresh(double now, double at, double limit) {
    return std::isfinite(at) && now >= at && now - at <= limit;
}
} // namespace

MavlinkCore::MavlinkCore(Json c, Wire w, Event e, std::function<void()> clear)
    : config(std::move(c)), uwb(config), wire(std::move(w)), event(std::move(e)),
      clear_pending(std::move(clear)) {
    state = {
        {"position", {{"lat", 0}, {"lon", 0}, {"alt", 0}, {"source", ""}, {"updatedAt", nullptr}}},
        {"attitude",
         {{"roll", 0}, {"pitch", 0}, {"yaw", 0}, {"yawRateDegS", 0}, {"updatedAt", nullptr}}},
        {"velocity", {{"vx", 0}, {"vy", 0}, {"vz", 0}}},
        {"ekf",
         {{"healthy", false},
          {"flags", 0},
          {"velocityVariance", nullptr},
          {"positionHorizontalVariance", nullptr},
          {"compassVariance", nullptr},
          {"updatedAt", nullptr},
          {"source", "EKF_STATUS_REPORT"}}},
        {"battery", {{"voltage", 0}, {"current", 0}, {"percentage", 100}}},
        {"servoOutputs", {{"ch1", 0}, {"ch2", 0}, {"ch3", 0}, {"ch4", 0}}},
        {"temperature", Json::object()},
        {"gps",
         {{"satellites", 0},
          {"hdop", 999},
          {"fixType", 0},
          {"latitude", 0},
          {"longitude", 0},
          {"altitude", 0},
          {"horizontalAccuracy", nullptr},
          {"verticalAccuracy", nullptr},
          {"groundSpeed", nullptr},
          {"course", nullptr},
          {"updatedAt", nullptr}}},
        {"imuCalibration",
         {{"active", false},
          {"mode", ""},
          {"status", "IDLE"},
          {"step", ""},
          {"stepCode", nullptr},
          {"instructions", ""},
          {"progress", nullptr},
          {"lastAckCommand", nullptr},
          {"lastAckResult", ""},
          {"updatedAt", nullptr}}},
        {"flightMode", "STABILIZE"},
        {"systemStatus", "STANDBY"},
        {"armed", false},
        {"pilotSteering", {{"type", nullptr}, {"ready", false}}},
        {"hardware", {{"motors", Json::object()}}}};
    for (const auto &side : {"left", "right"})
        state["hardware"]["motors"][side] = {
            {"channel", config.value(std::string("rover_") + side + "_channel",
                                     std::string(side) == "left" ? 1 : 3)},
            {"outputPwm", 0},
            {"outputUpdatedAt", nullptr},
            {"feedbackUpdatedAt", nullptr},
            {"feedbackSource", "none"}};
}
void MavlinkCore::emit(const std::string &type, const Json &payload) {
    event({{"type", type}, {"payload", payload}, {"timestamp", wall / 1000}});
}
void MavlinkCore::log(const std::string &level, const std::string &text, double interval) {
    const auto key = level + ":" + text;
    if (log_times.contains(key) && mono - log_times[key] < interval)
        return;
    // FCU text must not grow daemon memory without bound.
    if (log_times.size() >= 512)
        log_times.clear();
    log_times[key] = mono;
    emit("log", {{"level", level}, {"message", text}});
}
void MavlinkCore::send(mavlink_message_t &message) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto count = mavlink_msg_to_send_buffer(bytes, &message);
    wire(std::span<const uint8_t>(bytes, count));
}
void MavlinkCore::long_command(uint16_t command, const std::array<float, 7> &p) {
    mavlink_command_long_t value{};
    value.target_system = target_system;
    value.target_component = target_component;
    value.command = command;
    value.param1 = p[0];
    value.param2 = p[1];
    value.param3 = p[2];
    value.param4 = p[3];
    value.param5 = p[4];
    value.param6 = p[5];
    value.param7 = p[6];
    mavlink_message_t message{};
    mavlink_msg_command_long_encode_status(255, 0, &tx_status, &message, &value);
    send(message);
}
void MavlinkCore::manual(int steering, int throttle) {
    mavlink_manual_control_t value{};
    value.target = target_system;
    value.x = 32767;
    value.r = 32767;
    value.y = steering;
    value.z = throttle;
    mavlink_message_t message{};
    mavlink_msg_manual_control_encode_status(255, 0, &tx_status, &message, &value);
    send(message);
}
void MavlinkCore::neutral() {
    if (clear_pending)
        clear_pending();
    if (target_system) {
        if (motion_command == "ROVER_VELOCITY_NED" || motion_command == "ROVER_BODY_VELOCITY") {
            mavlink_set_position_target_local_ned_t v{};
            v.target_system = target_system;
            v.target_component = target_component;
            const bool body = motion_command == "ROVER_BODY_VELOCITY";
            v.coordinate_frame = body ? MAV_FRAME_BODY_NED : MAV_FRAME_LOCAL_NED;
            v.type_mask = body ? 1511 : 3559;
            mavlink_message_t message{};
            mavlink_msg_set_position_target_local_ned_encode_status(255, 0, &tx_status, &message,
                                                                    &v);
            send(message);
        }
        manual(0, 0);
    }
    motion_active = false;
    motion_command.clear();
}
void MavlinkCore::disconnect(double at, double clock) {
    wall = at;
    mono = clock;
    neutral();
    if (online)
        emit("connection", {{"connected", false}});
    online = false;
    state["armed"] = false;
    state["pilotSteering"] = {{"type", nullptr}, {"ready", false}};
    parameters = Json::object();
    param_times.clear();
    param_requests.clear();
    parser_message = {};
    parser_status = {};
    target_system = target_component = 0;
    interval_waiting = false;
}
void MavlinkCore::receive(std::span<const uint8_t> bytes, double at, double clock) {
    wall = at;
    mono = clock;
    mavlink_message_t message{};
    mavlink_status_t status{};
    for (auto byte : bytes)
        if (mavlink_frame_char_buffer(&parser_message, &parser_status, byte, &message, &status) ==
            MAVLINK_FRAMING_OK)
            parse(message);
}
void MavlinkCore::configure_intervals() {
    intervals = {{30, 20}, {36, 20}, {33, 10}, {24, 5}, {1, 5}, {193, 5}, {105, 10}};
    const std::array<int, 8> ids = {11030, 11031, 11032, 11040, 11041, 11042, 11043, 11044};
    std::set<int> groups;
    for (const auto &side : {"left", "right"})
        groups.insert((state["hardware"]["motors"][side]["channel"].get<int>() - 1) / 4);
    for (auto group : groups)
        if (group >= 0 && group < int(ids.size()))
            intervals.emplace_back(ids[group], 5);
    interval_index = 0;
    interval_attempts = 0;
    interval_waiting = false;
    interval_sent = -1e12;
}
void MavlinkCore::tick(double at, double clock) {
    wall = at;
    mono = clock;
    if (online &&
        mono - heartbeat_at > std::max(3000.0, number(config, "heartbeat_interval", 1) * 3500)) {
        log("ERROR", "Pixhawk heartbeat lost; control link closed");
        disconnect(at, clock);
    }
    if (motion_active && mono - command_at > number(config, "rover_command_timeout_ms", 500)) {
        neutral();
        log("WARNING", "Control command lease expired; output held neutral");
    }
    if (mono - heartbeat_sent >= std::max(200.0, number(config, "heartbeat_interval", 1) * 1000)) {
        mavlink_heartbeat_t value{};
        value.type = MAV_TYPE_GCS;
        value.autopilot = MAV_AUTOPILOT_INVALID;
        value.mavlink_version = 3;
        mavlink_message_t message{};
        mavlink_msg_heartbeat_encode_status(255, 0, &tx_status, &message, &value);
        send(message);
        heartbeat_sent = mono;
    }
    if (online) {
        // ACK 511 has no message ID. Keep one interval request in flight, while
        // receiving telemetry and serving commands throughout startup.
        if (interval_waiting && mono - interval_sent >= 800) {
            interval_waiting = false;
            if (interval_attempts >= 2) {
                ++interval_index;
                interval_attempts = 0;
            }
        }
        if (!interval_waiting && interval_index < intervals.size()) {
            const auto [id, rate] = intervals[interval_index];
            long_command(MAV_CMD_SET_MESSAGE_INTERVAL,
                         {float(id), float(std::nearbyint(1e6 / rate)), 0, 0, 0, 0, 0});
            interval_sent = mono;
            interval_waiting = true;
            ++interval_attempts;
        }
        if (mono - param_sent >= 80) {
            std::string candidate;
            double oldest = 1e30;
            for (const auto &name : required_parameters(parameters)) {
                const double seen = param_times.contains(name) ? param_times[name] : -1e12;
                const double asked = param_requests.contains(name) ? param_requests[name] : -1e12;
                if (mono - seen >= 10000 && mono - asked >= 1000 && asked < oldest) {
                    candidate = name;
                    oldest = asked;
                }
            }
            if (!candidate.empty()) {
                mavlink_param_request_read_t value{};
                value.target_system = target_system;
                value.target_component = target_component;
                value.param_index = -1;
                std::memcpy(value.param_id, candidate.data(),
                            std::min(candidate.size(), sizeof(value.param_id)));
                mavlink_message_t message{};
                mavlink_msg_param_request_read_encode_status(255, 0, &tx_status, &message, &value);
                send(message);
                param_sent = mono;
                param_requests[candidate] = mono;
            }
        }
    }
    if (mono - telemetry_sent >= 1000 / clamp(number(config, "telemetry_rate", 10), 1, 200)) {
        emit("telemetry", telemetry(at, clock));
        telemetry_sent = mono;
    }
}
Json MavlinkCore::acro_status() {
    Json reason = nullptr;
    try {
        validate_parameters(parameters);
        for (const auto &name : required_parameters(parameters))
            if (!param_times.contains(name) || mono - param_times[name] > 30000)
                reason = "acro_parameters_stale";
    } catch (const std::exception &error) {
        reason = error.what();
    }
    const bool velocity = number(state["gps"], "fixType", 0) >= 2 &&
                          state["ekf"].value("healthy", false) &&
                          fresh(wall, number(state["position"], "updatedAt"), 700) &&
                          fresh(wall, number(state["ekf"], "updatedAt"), 700);
    return {{"ready", reason.is_null()},
            {"reason", reason},
            {"updatedAt", wall},
            {"velocityAvailable", velocity},
            {"speedLimitMps", 2},
            {"fcuSpeedMaxMps", parameters.value("SPEED_MAX", Json(nullptr))},
            {"fcuTurnRateDegS", parameters.value("ACRO_TURN_RATE", Json(nullptr))}};
}
Json MavlinkCore::telemetry(double at, double clock) {
    wall = at;
    mono = clock;
    state["uwb"] = uwb.payload(wall);
    state["acroControl"] = acro_status();
    state["hardware"]["pixhawk"] = {
        {"online", online},
        {"source", "heartbeat"},
        {"lastSeenAt", heartbeat_wall > 0 ? Json(heartbeat_wall) : Json(nullptr)}};
    for (auto &motor : state["hardware"]["motors"]) {
        motor["outputPwm"] =
            state["servoOutputs"].value("ch" + std::to_string(motor["channel"].get<int>()), 0);
        motor["outputUpdatedAt"] = nullable(servo_at);
    }
    return state;
}
void MavlinkCore::calibration(const Json &changes) {
    state["imuCalibration"].update(changes);
    state["imuCalibration"]["updatedAt"] = wall / 1000;
}
void MavlinkCore::pose(int code) {
    if (code == 16777215 || code == 16777216) {
        const bool success = code == 16777215;
        calibration({{"active", false},
                     {"mode", "ACCEL"},
                     {"status", success ? "SUCCESS" : "FAILED"},
                     {"step", success ? "SUCCESS" : "FAILED"},
                     {"stepCode", code},
                     {"instructions", success ? "6-point IMU calibration completed. Check the "
                                                "attitude and accelerometer readings."
                                              : "6-point IMU calibration failed. Restart it and "
                                                "keep the rover still at every step."},
                     {"lastAckCommand", 42429},
                     {"lastAckResult", success ? "SUCCESS" : "FAILED"}});
        if (success)
            state["imuCalibration"]["progress"] = 100;
    } else if (code >= 1 && code <= 6)
        calibration({{"active", true},
                     {"mode", "ACCEL"},
                     {"status", "AWAITING_POSITION"},
                     {"step", poses[code]},
                     {"stepCode", code},
                     {"instructions", pose_hints[code]},
                     {"lastAckCommand", 42429},
                     {"lastAckResult", ""}});
}
void MavlinkCore::calibration_ack(int command, int result, int progress) {
    if (command != 241 && command != 42429)
        return;
    auto &cal = state["imuCalibration"];
    const auto name = result_name(result);
    const bool accepted = result == MAV_RESULT_ACCEPTED || result == MAV_RESULT_IN_PROGRESS;
    Json changes = {{"lastAckCommand", command}, {"lastAckResult", name}};
    if (progress >= 0 && progress <= 100)
        changes["progress"] = progress;
    if (!accepted) {
        changes.update(
            {{"active", false},
             {"status", result == 6 ? "CANCELLED" : "FAILED"},
             {"instructions", std::string(command == 241 ? "IMU calibration did not start: "
                                                         : "Pose confirmation failed: ") +
                                  name}});
    } else if (command == 241 && cal["mode"] == "LEVEL") {
        changes.update(
            {{"active", result != MAV_RESULT_ACCEPTED},
             {"status", result == MAV_RESULT_ACCEPTED ? "SUCCESS" : "IN_PROGRESS"},
             {"instructions",
              result == MAV_RESULT_ACCEPTED
                  ? "Level calibration completed. Keep the rover still for a few more seconds."
                  : "Level calibration in progress. Keep the rover level and still."}});
    } else {
        const auto step = cal.value("step", std::string());
        changes.update(
            {{"active", true},
             {"status", "IN_PROGRESS"},
             {"instructions",
              command == 241
                  ? "6-point IMU calibration is in progress. Waiting for the FCU pose request."
              : step.empty() ? "Pose confirmation sent. Waiting for the next FCU step."
                             : step + " confirmed. Waiting for the next FCU step."}});
    }
    calibration(changes);
    log("INFO", "IMU calibration ACK: command=" + std::to_string(command) + ", result=" + name,
        200);
}
void MavlinkCore::battery(double voltage, double current, double percentage) {
    auto &b = state["battery"];
    if (std::isfinite(voltage) && voltage > 0)
        b["voltage"] = voltage;
    if (std::isfinite(current) && current >= 0)
        b["current"] = current;
    if (std::isfinite(percentage) && percentage >= 0)
        b["percentage"] = percentage;
    if (number(b, "percentage") <= 20)
        log(number(b, "percentage") <= 10 ? "CRITICAL" : "WARNING",
            "Battery low: " + std::to_string(int(number(b, "percentage"))) + "%", 5000);
}
template <class T> void MavlinkCore::esc(const T &v, int base) {
    for (const auto &side : {"left", "right"}) {
        Json &motor = state["hardware"]["motors"][side];
        const int index = motor["channel"].get<int>() - base;
        if (index < 0 || index >= 4)
            continue;
        if (v.temperature[index] > 0)
            state["temperature"][std::string(side) == "left" ? "motorLeft" : "motorRight"] =
                v.temperature[index];
        if (v.temperature[index] || v.count[index] || v.voltage[index] || v.current[index] ||
            v.totalcurrent[index] || v.rpm[index]) {
            motor["feedbackUpdatedAt"] = wall;
            motor["feedbackSource"] = "esc_telemetry";
        }
    }
}

void MavlinkCore::parse(const mavlink_message_t &message) {
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t v{};
        mavlink_msg_heartbeat_decode(&message, &v);
        if (v.type == MAV_TYPE_GCS || v.autopilot == MAV_AUTOPILOT_INVALID)
            return;
        if (!target_system) {
            target_system = message.sysid;
            target_component = message.compid;
            configure_intervals();
        }
        if (message.sysid != target_system || message.compid != target_component)
            return;
        heartbeat_at = mono;
        heartbeat_wall = wall;
        if (!online) {
            online = true;
            emit("connection", {{"connected", true}});
        }
        const auto mode = modes.contains(v.custom_mode)
                              ? modes.at(v.custom_mode)
                              : "Mode(" + std::to_string(v.custom_mode) + ")";
        if (state["flightMode"] != mode)
            log("INFO", "Flight mode: " + mode, 0);
        const bool armed = (v.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        if (state["armed"] != armed)
            log(armed ? "COMMAND" : "WARNING", armed ? "Vehicle armed" : "Vehicle disarmed", 0);
        state["flightMode"] = mode;
        state["armed"] = armed;
        static const std::array<std::string, 9> names = {
            "UNINIT",   "BOOT",      "CALIBRATING", "STANDBY",           "ACTIVE",
            "CRITICAL", "EMERGENCY", "POWEROFF",    "FLIGHT_TERMINATION"};
        state["systemStatus"] = v.system_status < names.size()
                                    ? "MAV_STATE_" + names[v.system_status]
                                    : std::to_string(v.system_status);
        return;
    }
    if (!online)
        return;
    // UWB receivers may use their own component/system IDs. Only the selected
    // FCU is allowed to update flight state, command ACKs and RC parameters.
    if (message.msgid != MAVLINK_MSG_ID_NAMED_VALUE_FLOAT &&
        (message.sysid != target_system || message.compid != target_component))
        return;
    switch (message.msgid) {
    case MAVLINK_MSG_ID_PARAM_VALUE: {
        mavlink_param_value_t v{};
        mavlink_msg_param_value_decode(&message, &v);
        const auto name = fixed_string(v.param_id, 16);
        if (std::isfinite(v.param_value) && required_parameters(parameters).contains(name)) {
            parameters[name] = v.param_value;
            param_times[name] = mono;
        }
        if (name == "PILOT_STEER_TYPE") {
            const bool valid = std::isfinite(v.param_value) &&
                               v.param_value == std::trunc(v.param_value) && v.param_value >= 0 &&
                               v.param_value <= 3;
            state["pilotSteering"] = {{"type", valid ? Json(int(v.param_value)) : Json(nullptr)},
                                      {"ready", valid}};
        }
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t v{};
        mavlink_msg_attitude_decode(&message, &v);
        if (!std::isfinite(v.roll) || !std::isfinite(v.pitch) || !std::isfinite(v.yaw) ||
            !std::isfinite(v.yawspeed))
            break;
        state["attitude"] = {{"roll", v.roll / radians},
                             {"pitch", v.pitch / radians},
                             {"yaw", v.yaw / radians},
                             {"yawRateDegS", v.yawspeed / radians},
                             {"updatedAt", wall}};
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t v{};
        mavlink_msg_global_position_int_decode(&message, &v);
        state["position"] = {{"lat", v.lat / 1e7},
                             {"lon", v.lon / 1e7},
                             {"alt", v.relative_alt / 1000.0},
                             {"source", "GLOBAL_POSITION_INT"},
                             {"updatedAt", wall}};
        state["velocity"] = {{"vx", v.vx / 100.0}, {"vy", v.vy / 100.0}, {"vz", v.vz / 100.0}};
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t v{};
        mavlink_msg_gps_raw_int_decode(&message, &v);
        auto &g = state["gps"];
        const bool fix = v.fix_type >= 2;
        g["satellites"] = v.satellites_visible;
        g["fixType"] = v.fix_type;
        if (v.eph && v.eph != 65535)
            g["hdop"] = v.eph / 100.0;
        if (fix && (std::abs(v.lat / 1e7) > 1e-6 || std::abs(v.lon / 1e7) > 1e-6)) {
            g["latitude"] = v.lat / 1e7;
            g["longitude"] = v.lon / 1e7;
            g["altitude"] = v.alt / 1000.0;
        }
        g["horizontalAccuracy"] =
            fix && v.h_acc > 0 && v.h_acc < 1000000 ? Json(v.h_acc / 1000.0) : Json(nullptr);
        g["verticalAccuracy"] =
            fix && v.v_acc > 0 && v.v_acc < 1000000 ? Json(v.v_acc / 1000.0) : Json(nullptr);
        g["groundSpeed"] = fix && v.vel != 65535 ? Json(v.vel / 100.0) : Json(nullptr);
        g["course"] = fix && v.cog != 65535 ? Json(v.cog / 100.0) : Json(nullptr);
        g["updatedAt"] = wall;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t v{};
        mavlink_msg_ekf_status_report_decode(&message, &v);
        state["ekf"] = {{"flags", int(v.flags)},
                        {"healthy", (v.flags & 18) == 18 && (v.flags & (128 | 32768)) == 0},
                        {"velocityVariance", nullable(v.velocity_variance)},
                        {"positionHorizontalVariance", nullable(v.pos_horiz_variance)},
                        {"compassVariance", nullable(v.compass_variance)},
                        {"source", "EKF_STATUS_REPORT"},
                        {"updatedAt", wall}};
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t v{};
        mavlink_msg_sys_status_decode(&message, &v);
        battery(v.voltage_battery / 1000.0, v.current_battery / 100.0, v.battery_remaining);
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t v{};
        mavlink_msg_battery_status_decode(&message, &v);
        double voltage = 0;
        for (size_t i = 0; i < 10; ++i) {
            const auto cell = v.voltages[i];
            if (cell && cell != 65535)
                voltage += cell / 1000.0;
        }
        battery(voltage, v.current_battery / 100.0, v.battery_remaining);
        break;
    }
    case MAVLINK_MSG_ID_HIGHRES_IMU: {
        mavlink_highres_imu_t v{};
        mavlink_msg_highres_imu_decode(&message, &v);
        if (std::isfinite(v.temperature))
            state["temperature"]["flightController"] = v.temperature;
        break;
    }
#define PRESSURE(ID, TYPE, FN)                                                                     \
    case ID: {                                                                                     \
        TYPE v{};                                                                                  \
        FN(&message, &v);                                                                          \
        state["temperature"]["flightController"] = v.temperature / 100.0;                          \
        break;                                                                                     \
    }
        PRESSURE(MAVLINK_MSG_ID_SCALED_PRESSURE, mavlink_scaled_pressure_t,
                 mavlink_msg_scaled_pressure_decode)
        PRESSURE(MAVLINK_MSG_ID_SCALED_PRESSURE2, mavlink_scaled_pressure2_t,
                 mavlink_msg_scaled_pressure2_decode)
        PRESSURE(MAVLINK_MSG_ID_SCALED_PRESSURE3, mavlink_scaled_pressure3_t,
                 mavlink_msg_scaled_pressure3_decode)
#undef PRESSURE
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t v{};
        mavlink_msg_servo_output_raw_decode(&message, &v);
        state["servoOutputs"] = {{"ch1", int(v.servo1_raw)},
                                 {"ch2", int(v.servo2_raw)},
                                 {"ch3", int(v.servo3_raw)},
                                 {"ch4", int(v.servo4_raw)}};
        servo_at = wall;
        break;
    }
#define ESC(ID, TYPE, FN, BASE)                                                                    \
    case ID: {                                                                                     \
        TYPE v{};                                                                                  \
        FN(&message, &v);                                                                          \
        esc(v, BASE);                                                                              \
        break;                                                                                     \
    }
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, mavlink_esc_telemetry_1_to_4_t,
            mavlink_msg_esc_telemetry_1_to_4_decode, 1)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8, mavlink_esc_telemetry_5_to_8_t,
            mavlink_msg_esc_telemetry_5_to_8_decode, 5)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12, mavlink_esc_telemetry_9_to_12_t,
            mavlink_msg_esc_telemetry_9_to_12_decode, 9)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16, mavlink_esc_telemetry_13_to_16_t,
            mavlink_msg_esc_telemetry_13_to_16_decode, 13)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20, mavlink_esc_telemetry_17_to_20_t,
            mavlink_msg_esc_telemetry_17_to_20_decode, 17)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, mavlink_esc_telemetry_21_to_24_t,
            mavlink_msg_esc_telemetry_21_to_24_decode, 21)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28, mavlink_esc_telemetry_25_to_28_t,
            mavlink_msg_esc_telemetry_25_to_28_decode, 25)
        ESC(MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32, mavlink_esc_telemetry_29_to_32_t,
            mavlink_msg_esc_telemetry_29_to_32_decode, 29)
#undef ESC
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t v{};
        mavlink_msg_command_ack_decode(&message, &v);
        if (v.command == MAV_CMD_SET_MESSAGE_INTERVAL && interval_waiting &&
            v.result != MAV_RESULT_IN_PROGRESS) {
            interval_waiting = false;
            if (v.result != MAV_RESULT_TEMPORARILY_REJECTED || interval_attempts >= 2) {
                ++interval_index;
                interval_attempts = 0;
            }
        }
        if (v.command == MAV_CMD_COMPONENT_ARM_DISARM)
            log(v.result == 0 ? "INFO" : "WARNING", "FCU ARM/DISARM ACK: " + result_name(v.result),
                200);
        calibration_ack(v.command, v.result, v.progress);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t v{};
        mavlink_msg_command_long_decode(&message, &v);
        if (v.command == 42429 && std::isfinite(v.param1))
            pose(int(std::nearbyint(v.param1)));
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t v{};
        mavlink_msg_command_int_decode(&message, &v);
        if (v.command == 42429 && std::isfinite(v.param1))
            pose(int(std::nearbyint(v.param1)));
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
        mavlink_named_value_float_t v{};
        mavlink_msg_named_value_float_decode(&message, &v);
        if (uwb.update(fixed_string(v.name, 10), v.value, wall,
                       Json::array({message.sysid, message.compid}), v.time_boot_ms,
                       number(state["attitude"], "yaw"), number(state["attitude"], "updatedAt")))
            emit("uwb", uwb.payload(wall));
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t v{};
        mavlink_msg_statustext_decode(&message, &v);
        const auto text = fixed_string(v.text, 50);
        if (text.empty())
            break;
        log(v.severity <= 2   ? "CRITICAL"
            : v.severity <= 4 ? "WARNING"
            : v.severity <= 6 ? "INFO"
                              : "DEBUG",
            "FCU: " + text);
        auto lower = text;
        for (auto &ch : lower)
            ch = char(std::tolower(static_cast<unsigned char>(ch)));
        bool relevant = false;
        for (const auto *word : {"calib", "accel", "imu", "trim", "place vehicle"})
            relevant |= lower.find(word) != std::string::npos;
        if (state["imuCalibration"].value("active", false) && relevant) {
            Json changes = {{"instructions", text}};
            if (lower.find("success") != std::string::npos ||
                lower.find("completed") != std::string::npos)
                changes.update({{"status", "SUCCESS"}, {"active", false}});
            else if (lower.find("failed") != std::string::npos ||
                     lower.find("error") != std::string::npos)
                changes.update({{"status", "FAILED"}, {"active", false}});
            calibration(changes);
        }
        break;
    }
    default:
        break;
    }
}

Json MavlinkCore::command(const Json &request, double at, double clock) {
    wall = at;
    mono = clock;
    try {
        if (!request.is_object())
            throw std::invalid_argument("invalid_command");
        const auto name = request.value("command", std::string());
        const auto p = request.value("params", Json::object());
        if (!p.is_object())
            throw std::invalid_argument("invalid_params");
        if (!online)
            throw std::invalid_argument("pixhawk_disconnected");
        for (const auto *key :
             {"throttle", "steering", "throttlePwm", "steeringPwm", "forwardSpeedMps",
              "yawRateDegS", "vx", "vy", "yawRateRadS", "positionCode"})
            if (p.contains(key) && !std::isfinite(number(p, key)))
                throw std::invalid_argument("non_finite_command_parameter");
        if (name == "CONTROL_NEUTRAL")
            neutral();
        else if (name == "ROVER_DRIVE") {
            const double throttle = p.contains("throttle") ? number(p, "throttle")
                                    : p.contains("throttlePwm")
                                        ? (number(p, "throttlePwm") - 1500) / 5
                                        : 0;
            const double steering = p.contains("steering") ? number(p, "steering")
                                    : p.contains("steeringPwm")
                                        ? (number(p, "steeringPwm") - 1500) * 45 / 500
                                        : 0;
            auto [y, z] = manual_axes(throttle, steering, config);
            if (p.value("motorTargetSemantics", false))
                std::tie(y, z) =
                    motor_target_axes(y, z, int(number(state["pilotSteering"], "type", -1)));
            manual(y, z);
            motion_active = y != 0 || z != 0;
            command_at = mono;
        } else if (name == "ROVER_ACRO") {
            const double speed = number(p, "forwardSpeedMps", 0), yaw = number(p, "yawRateDegS", 0);
            const auto status = acro_status();
            if (!status["ready"].get<bool>() || state["flightMode"] != "ACRO" ||
                ((speed != 0 || yaw != 0) && !status["velocityAvailable"].get<bool>()))
                throw std::invalid_argument("acro_mode_or_velocity_unavailable");
            auto [y, z] = acro_axes(speed, yaw, parameters);
            manual(y, z);
            motion_active = y != 0 || z != 0;
            command_at = mono;
        } else if (name == "ROVER_VELOCITY_NED" || name == "ROVER_BODY_VELOCITY") {
            const bool body = name == "ROVER_BODY_VELOCITY";
            const double x = number(p, body ? "forwardSpeedMps" : "vx", 0),
                         y = number(p, body ? "yawRateRadS" : "vy", 0);
            if (!std::isfinite(x) || !std::isfinite(y))
                throw std::invalid_argument("invalid_velocity");
            mavlink_set_position_target_local_ned_t v{};
            v.target_system = target_system;
            v.target_component = target_component;
            v.time_boot_ms = uint32_t(uint64_t(std::max(0.0, mono)));
            v.coordinate_frame = body ? MAV_FRAME_BODY_NED : MAV_FRAME_LOCAL_NED;
            v.type_mask = body ? 1511 : 3559;
            v.vx = clamp(x, -5, 5);
            if (body)
                v.yaw_rate = clamp(y, -120 * radians, 120 * radians);
            else
                v.vy = clamp(y, -5, 5);
            mavlink_message_t message{};
            mavlink_msg_set_position_target_local_ned_encode_status(255, 0, &tx_status, &message,
                                                                    &v);
            send(message);
            motion_active = x != 0 || y != 0;
            command_at = mono;
        } else if (name == "SET_MODE") {
            auto mode = p.value("mode", std::string());
            for (auto &ch : mode)
                ch = char(std::toupper(static_cast<unsigned char>(ch)));
            auto found = std::find_if(modes.begin(), modes.end(),
                                      [&](auto &item) { return item.second == mode; });
            if (found == modes.end())
                throw std::invalid_argument("unsupported_mode");
            long_command(MAV_CMD_DO_SET_MODE,
                         {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, float(found->first), 0, 0, 0, 0, 0});
        } else if (name == "ARM" || name == "DISARM") {
            if (name == "DISARM")
                neutral();
            long_command(MAV_CMD_COMPONENT_ARM_DISARM,
                         {name == "ARM" ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0});
        } else if (name == "EMERGENCY_STOP") {
            neutral();
            long_command(MAV_CMD_COMPONENT_ARM_DISARM);
        } else if (name == "IMU_CALIBRATION_START") {
            const auto type = p.value("type", std::string("ACCEL"));
            if (type != "ACCEL" && type != "LEVEL")
                throw std::invalid_argument("unsupported_calibration");
            long_command(241, {0, 0, 0, 0, type == "ACCEL" ? 1.0f : 2.0f, 0, 0});
            calibration({{"active", true},
                         {"mode", type},
                         {"status", "STARTING"},
                         {"step", ""},
                         {"stepCode", nullptr},
                         {"progress", type == "ACCEL" ? Json(0) : Json(nullptr)},
                         {"instructions",
                          type == "ACCEL"
                              ? "6-point IMU calibration started. Waiting for the FCU pose request."
                              : "Level calibration started. Keep the rover level and still."},
                         {"lastAckCommand", 241},
                         {"lastAckResult", ""}});
        } else if (name == "IMU_CALIBRATION_CONFIRM") {
            const double code = number(p, "positionCode");
            if (!std::isfinite(code) || code < 1 || code > 6 || code != std::trunc(code))
                throw std::invalid_argument("invalid_calibration_position");
            long_command(42429, {float(code), 0, 0, 0, 0, 0, 0});
            calibration({{"active", true},
                         {"mode", "ACCEL"},
                         {"status", "CONFIRMING_POSITION"},
                         {"step", poses[int(code)]},
                         {"stepCode", int(code)},
                         {"instructions",
                          poses[int(code)] + " confirmation sent. Waiting for the next FCU step."},
                         {"lastAckCommand", 42429},
                         {"lastAckResult", ""}});
        } else
            throw std::invalid_argument("unsupported_command");
        if (name.starts_with("ROVER_"))
            motion_command = name;
        return {{"ok", true}};
    } catch (const std::exception &error) {
        // Malformed or rejected motion commands may never leave an earlier
        // nonzero actuator request running until its normal lease expires.
        const auto name =
            request.is_object() ? request.value("command", std::string()) : std::string();
        if (name.starts_with("ROVER_"))
            neutral();
        return {{"ok", false}, {"error", error.what()}};
    }
}
} // namespace manta
