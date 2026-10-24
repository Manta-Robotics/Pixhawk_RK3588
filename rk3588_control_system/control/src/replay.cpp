#include "manta/uwb_filter.hpp"
#include "manta/uwb_fusion.hpp"
#include <iostream>
#include <memory>

// JSONL offline harness: no sockets, serial ports, or actuator output.
int main() {
    auto filter = std::make_unique<manta::UwbFilter>();
    auto fusion = std::make_unique<manta::UwbFusion>();
    std::string line;
    while (std::getline(std::cin, line)) {
        try {
            const auto request = manta::Json::parse(line);
            const auto op = request.value("op", std::string("update"));
            if (op == "fusion_reset") {
                fusion = std::make_unique<manta::UwbFusion>(
                    request.value("options", manta::Json::object()));
                std::cout << "{\"ok\":true}\n";
            } else if (op == "fusion_update") {
                const auto r = fusion->update(request.at("name"), manta::number(request, "value"),
                                              manta::number(request, "now_ms"),
                                              request.value("source", manta::Json(nullptr)));
                std::cout
                    << (r ? manta::Json::array({r->first, r->second}) : manta::Json(nullptr)).dump()
                    << '\n';
            } else if (op == "fusion_apply") {
                fusion->apply(request.at("receiver_id"), request.at("result"));
                std::cout << "null\n";
            } else if (op == "fusion_set") {
                fusion->set_filtered(request.at("receiver_id"), request.at("field"),
                                     request.at("value"));
                std::cout << "null\n";
            } else if (op == "fusion_result")
                std::cout << fusion->fused(manta::number(request, "now_ms")).dump() << '\n';
            else if (op == "reset") {
                filter = std::make_unique<manta::UwbFilter>(
                    request.value("options", manta::Json::object()));
                std::cout << "{\"ok\":true}\n";
            } else {
                if (!filter)
                    filter = std::make_unique<manta::UwbFilter>();
                if (op == "result")
                    std::cout << filter
                                     ->result(manta::number(request, "now_ms"),
                                              manta::number(request, "heading_deg"))
                                     .dump()
                              << '\n';
                else if (op == "reject")
                    std::cout << filter
                                     ->reject(manta::number(request, "now_ms"),
                                              request.at("reason"),
                                              manta::number(request, "heading_deg"))
                                     .dump()
                              << '\n';
                else if (op == "update")
                    std::cout << filter->update(request).dump() << '\n';
                else
                    throw std::invalid_argument("unknown replay operation");
            }
        } catch (const std::exception &e) {
            std::cout << manta::Json{{"error", e.what()}}.dump() << '\n';
        }
    }
}
