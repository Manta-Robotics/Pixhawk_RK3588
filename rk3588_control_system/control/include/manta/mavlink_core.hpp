#pragma once
#include "rover_mapping.hpp"
#include "uwb_pipeline.hpp"
#include "uwb_timing.hpp"
#include <ardupilotmega/mavlink.h>
#include <functional>
#include <map>
#include <span>
#include <set>
#include <tuple>

namespace manta {
// Clock and transport are injected. The same state machine runs in the daemon
// and in the offline wire-protocol tests, without a Python runtime.
class MavlinkCore {
  public:
    using Wire = std::function<void(std::span<const uint8_t>)>;
    using Event = std::function<void(const Json &)>;
    MavlinkCore(Json config, Wire wire, Event event, std::function<void()> clear_pending = {});
    void receive(std::span<const uint8_t> bytes, double wall_ms, double monotonic_ms);
    void tick(double wall_ms, double monotonic_ms);
    Json command(const Json &request, double wall_ms, double monotonic_ms);
    Json telemetry(double wall_ms, double monotonic_ms);
    void disconnect(double wall_ms, double monotonic_ms);
    bool connected() const { return online; }
    void acknowledge_uwb_clearance(double now) { uwb.acknowledge_clearance(now); }

  private:
    Json config, state, parameters = Json::object();
    UwbPipeline uwb;
    UwbTiming uwb_timing;
    uint64_t uwb_session_id = 0, uwb_record_id = 0;
    std::string uwb_source_instance;
    uint32_t attitude_stamp = 0;
    bool attitude_stamp_ready = false;
    Wire wire;
    Event event;
    std::function<void()> clear_pending;
    mavlink_message_t parser_message{};
    mavlink_status_t parser_status{}, tx_status{};
    std::map<std::pair<int, int>, std::pair<int, uint64_t>> uwb_streams;
    bool uwb_epoch_ready = false, uwb_epoch_pending = false;
    int64_t uwb_epoch_nonce = 0;
    double uwb_epoch_sent = -1e12, uwb_epoch_at = 0, uwb_epoch_rtt = 0;
    uint32_t uwb_epoch_boot = 0, uwb_epoch_floor = 0;
    std::set<std::tuple<int, int, int>> uwb_started;
    uint8_t target_system = 0, target_component = 0;
    bool online = false, motion_active = false;
    std::string motion_command;
    double wall = 0, mono = 0, heartbeat_at = -1, heartbeat_wall = 0, heartbeat_sent = -1e12;
    double telemetry_sent = -1e12, command_at = -1e12, param_sent = -1e12;
    double servo_at = missing;
    std::map<std::string, double> param_times, param_requests, log_times;
    std::vector<std::pair<int, double>> intervals;
    size_t interval_index = 0;
    int interval_attempts = 0;
    double interval_sent = -1e12;
    bool interval_waiting = false;
    void parse(const mavlink_message_t &message);
    void request_uwb_epoch();
    void publish_uwb_measurements();
    void send(mavlink_message_t &message);
    void emit(const std::string &type, const Json &payload);
    void log(const std::string &level, const std::string &text, double interval = 2000);
    void long_command(uint16_t command, const std::array<float, 7> &values = {});
    void manual(int steering, int throttle);
    void neutral();
    void configure_intervals();
    void pose(int code);
    void calibration_ack(int command, int result, int progress);
    void calibration(const Json &changes);
    Json acro_status();
    void battery(double voltage, double current, double percentage);
    template <class T> void esc(const T &value, int base);
};
} // namespace manta
