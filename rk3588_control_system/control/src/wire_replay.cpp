#include "manta/mavlink_core.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

int main() {
    using manta::Json;
    Json frames = Json::array(), events = Json::array();
    const auto wire = [&](std::span<const uint8_t> bytes) {
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
            if (op == "reset")
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
