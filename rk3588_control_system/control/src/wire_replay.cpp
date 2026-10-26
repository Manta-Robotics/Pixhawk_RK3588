#include "manta/clock_export.hpp"
#include "manta/mavlink_core.hpp"
#include <ardupilotmega/mavlink.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

int main() {
    using manta::Json;
    Json frames = Json::array(), events = Json::array();
    int64_t sync_nonce = 0;
    const auto wire = [&](std::span<const uint8_t> bytes) {
        mavlink_message_t buffer{}, message{};
        mavlink_status_t parser{}, status{};
        for (auto byte : bytes) {
            if (mavlink_frame_char_buffer(&buffer, &parser, byte, &message, &status) == MAVLINK_FRAMING_OK &&
                message.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                mavlink_timesync_t sync{};
                mavlink_msg_timesync_decode(&message, &sync);
                if (sync.tc1 == 0) sync_nonce = sync.ts1;
            }
        }
        std::ostringstream out;
        for (auto byte : bytes)
            out << std::hex << std::setfill('0') << std::setw(2) << int(byte);
        frames.push_back(out.str());
    };
    const auto event = [&](const Json &value) { events.push_back(value); };
    auto core = std::make_unique<manta::MavlinkCore>(Json::object(), wire, event);
    std::string line;
    while (std::getline(std::cin, line)) {
        frames = Json::array();
        events = Json::array();
        try {
            const auto request = Json::parse(line);
            const auto op = request.value("op", std::string());
            const auto wall = request.value("wall_ms", 1000000.0),
                       mono = request.value("mono_ms", 0.0);
            Json result = nullptr;
            if (op == "export_clock")
                result =
                    manta::export_wall_clock(request.at("packet"), wall, request.at("system_ms"));
            else if (op == "reset")
                core = std::make_unique<manta::MavlinkCore>(request.value("config", Json::object()),
                                                            wire, event);
            else if (op == "receive") {
                auto hex = request.at("hex").get<std::string>();
                if (hex.size() % 2)
                    throw std::invalid_argument("invalid_hex");
                std::vector<uint8_t> bytes;
                for (size_t i = 0; i < hex.size(); i += 2)
                    bytes.push_back(uint8_t(std::stoul(hex.substr(i, 2), nullptr, 16)));
                core->receive(bytes, wall, mono);
            } else if (op == "tick")
                core->tick(wall, mono);
            else if (op == "sync") {
                // Exercise the real decoder/nonce check, never set session state directly.
                mavlink_message_t message{};
                mavlink_status_t status{};
                status.current_tx_seq = request.value("seq", 0);
                mavlink_timesync_t sync{};
                sync.tc1 = request.value("boot_ms", int64_t(0)) * 1000000 + 1;
                sync.ts1 = request.value("nonce", sync_nonce);
                mavlink_msg_timesync_encode_status(request.value("sysid", 1),
                    request.value("compid", 1), &status, &message, &sync);
                std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
                const auto size = mavlink_msg_to_send_buffer(bytes.data(), &message);
                core->receive(std::span(bytes.data(), size), wall, mono);
            } else if (op == "ack_uwb_clearance")
                core->acknowledge_uwb_clearance(wall);
            else if (op == "command")
                result = core->command(request.at("request"), wall, mono);
            else if (op == "disconnect")
                core->disconnect(wall, mono);
            else if (op != "state")
                throw std::invalid_argument("unknown_operation");
            std::cout << Json{{"result", result},
                              {"frames", frames},
                              {"events", events},
                              {"state", core->telemetry(wall, mono)},
                              {"connected", core->connected()}}
                             .dump()
                      << '\n';
        } catch (const std::exception &error) {
            std::cout << Json{{"error", error.what()}}.dump() << '\n';
        }
        std::cout.flush();
    }
}
