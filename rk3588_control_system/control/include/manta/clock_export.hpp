#pragma once
#include "common.hpp"

namespace manta {
// Keep control and leases on the steady clock. Only external timestamps are
// translated to the current wall clock, preserving every observation's age.
// Null/zero sentinels, stale data, and future data keep their original meaning.
inline Json export_wall_clock(Json packet, double control_now_ms, double wall_now_ms) {
    const double offset = wall_now_ms - control_now_ms;
    const auto translate = [&](auto &&self, Json &value, const std::string &key,
                               int depth) -> void {
        if (value.is_object()) {
            for (auto it = value.begin(); it != value.end(); ++it)
                self(self, it.value(), it.key(), depth + 1);
        } else if (value.is_array()) {
            for (auto &item : value)
                self(self, item, "", depth + 1);
        } else if (value.is_number() && value.get<double>() > 0) {
            if (key == "timestamp")
                value = value.get<double>() + (depth == 1 ? offset / 1000 : offset);
            else if (key.ends_with("At") || key == "accepted_at" || key == "updated_at")
                value = value.get<double>() + offset;
        }
    };
    translate(translate, packet, "", 0);
    return packet;
}
} // namespace manta
