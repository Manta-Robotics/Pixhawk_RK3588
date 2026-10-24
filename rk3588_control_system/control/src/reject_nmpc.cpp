#include "manta/recovery.hpp"

namespace manta {
namespace {
bool recent(double at, double now, double timeout) {
    return std::isfinite(at) && at > 0 && now >= at && now - at < timeout;
}
std::string sid(const Json &s) {
    const auto str = [](const Json &v) {
        return v.is_null() ? std::string() : v.is_string() ? v.get<std::string>() : v.dump();
    };
    return str(s.value("receiverId", Json(nullptr))) + ":" + str(s.value("tagId", Json(nullptr)));
}
double elapsed_ms(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
}
} // namespace
void RejectNmpc::reset() {
    accepted = command = plan = nullptr;
}
Json RejectNmpc::range_evidence(double now, const Json &t) const {
    const auto p = c.value("prediction", Json::object());
    double near = missing, at = missing;
    int zeros = 0, invalid = 0;
    for (const auto &s : t.value("uwb", Json::object()).value("sensors", Json::array())) {
        const double stamp = number(s, "rawDistanceUpdatedAt", number(s, "receivedMeasurementAt")),
                     range = number(s, "rawDistanceM");
        if (!recent(stamp, now, number(p, "measurement_gap_ms", 450)))
            continue;
        if (range == 0) {
            ++zeros;
            continue;
        }
        if (s.value("valid", true) == false || !std::isfinite(range) || range < .15 || range > 20) {
            ++invalid;
            continue;
        }
        if (range <= 1.2 && (!std::isfinite(near) || range < near)) {
            near = range;
            at = stamp;
        }
    }
    const double noise = std::max(.12, number(p, "measurement_noise_m", .12)),
                 age = std::isfinite(near) ? (now - at) / 1000 : 0;
    const auto v = t.value("velocity", Json::object());
    const double allowance = age * (number(p, "max_target_speed_mps", 3) +
                                    std::hypot(number(v, "vx", 0), number(v, "vy", 0)));
    return {{"kind", std::isfinite(near) ? "positive_near_range"
                     : zeros             ? "invalid_zero_unknown"
                                         : "no_near_range"},
            {"veto", std::isfinite(near) && near - noise <= number(c, "safety_distance_m", .3)},
            {"rangeM", nullable(near)},
            {"lowerBoundM", nullable(near - noise - allowance)},
            {"at", nullable(at)},
            {"noiseM", noise},
            {"ageAllowanceM", allowance},
            {"invalidZeros", zeros},
            {"invalidPackets", invalid}};
}
void RejectNmpc::observe(double now, const Json &t, const Json &r) {
    if (r.value("safe", false) && std::isfinite(number(r, "forwardSpeedMps")) &&
        std::isfinite(number(r, "yawRateDegS")))
        command = {{"at", now}, {"speed", r["forwardSpeedMps"]}, {"yaw", r["yawRateDegS"]}};
    const auto e = r.value("targetEstimate", Json(nullptr)),
               p = c.value("prediction", Json::object()),
               sensors = t.value("uwb", Json::object()).value("sensors", Json::array());
    if (!e.is_object() || sensors.empty())
        return;
    const auto &s = sensors[0];
    const auto b = e.value("relativeBodyM", Json::object()),
               pos = t.value("position", Json::object()), att = t.value("attitude", Json::object());
    if (number(e, "admissionVersion") != 1 || !e.value("measurementConfirmed", false) ||
        e.value("referenceFrame", std::string()) != "local_ned" ||
        !recent(number(e, "lastMeasuredAt"), now, number(p, "measurement_gap_ms", 450)) ||
        number(e, "evaluatedAt") != now || !std::isfinite(number(b, "x")) ||
        !std::isfinite(number(b, "y")) || !std::isfinite(number(e, "uncertaintyM")) ||
        number(e, "uncertaintyM") < 0 || !std::isfinite(number(pos, "lat")) ||
        !std::isfinite(number(pos, "lon")) || !std::isfinite(number(att, "yaw")) ||
        quarantined(s) || !s.value("valid", true) ||
        s.value("measurementStatus", std::string()) != "trusted" ||
        number(s, "receivedMeasurementAt") != number(e, "lastMeasuredAt") ||
        number(accepted, "measurementAt") == number(e, "lastMeasuredAt"))
        return;
    const auto velocity = e.value("targetVelocityNedMps", Json::object()),
               ve = e.value("velocityEstimate", Json(nullptr));
    const bool trusted = ve.is_object() && ve.value("trusted", false) &&
                         std::isfinite(number(velocity, "x")) &&
                         std::isfinite(number(velocity, "y"));
    const Vec v = trusted ? vector_value(velocity) : Vec{};
    if (v.norm() > number(p, "max_target_speed_mps", 3))
        return;
    const auto point = rotate_vector(vector_value(b), number(att, "yaw") * radians);
    accepted = {{"at", now},
                {"measurementAt", e["lastMeasuredAt"]},
                {"id", sid(s)},
                {"lat", pos["lat"]},
                {"lon", pos["lon"]},
                {"x", point.x},
                {"y", point.y},
                {"velocity", vector_json(v)},
                {"velocityTrusted", trusted},
                {"uncertainty", e["uncertaintyM"]}};
}
Json RejectNmpc::solve(double now, const Json &t, const Json &direction, const Json &near_raw) {
    const auto start = std::chrono::steady_clock::now();
    const auto p = c.value("prediction", Json::object()),
               m = c.value("reject_nmpc", Json::object()), a = accepted;
    const auto fail = [&](const std::string &reason) {
        return Json{{"feasible", false}, {"reason", reason}, {"solveMs", elapsed_ms(start)}};
    };
    if (!m.value("enabled", false))
        return fail("disabled");
    const double loss = number(p, "wait_timeout_ms", 5000);
    if (!a.is_object() || now < number(a, "at") || !recent(number(a, "measurementAt"), now, loss))
        return fail("accepted_state_expired");
    const auto sensors = t.value("uwb", Json::object()).value("sensors", Json::array());
    for (const auto &s : sensors)
        if (sid(s) != a.at("id").get<std::string>()) {
            reset();
            return fail("identity_changed");
        }
    const auto evidence = near_raw.is_object() ? near_raw : Json(nullptr);
    if (evidence.is_object() ? evidence.value("veto", false) : near_raw == true) {
        auto r = fail("near_raw_veto");
        r["rangeEvidence"] = evidence;
        return r;
    }
    const auto v = t.value("velocity", Json::object()), att = t.value("attitude", Json::object()),
               pos = t.value("position", Json::object());
    if (!std::isfinite(number(v, "vx")) || !std::isfinite(number(v, "vy")) ||
        !std::isfinite(number(att, "yaw")) || !std::isfinite(number(att, "yawRateDegS")) ||
        !std::isfinite(number(pos, "lat")) || !std::isfinite(number(pos, "lon")))
        return fail("navigation_invalid");
    const double yaw = number(att, "yaw") * radians,
                 u = number(v, "vx") * std::cos(yaw) + number(v, "vy") * std::sin(yaw);
    const Vec drift{number(v, "vx") - u * std::cos(yaw), number(v, "vy") - u * std::sin(yaw)};
    const double elapsed = (now - number(a, "at")) / 1000,
                 age = (now - number(a, "measurementAt")) / 1000;
    const Vec moved{(number(pos, "lat") - number(a, "lat")) * radians * 6378137,
                    wrap(number(pos, "lon") - number(a, "lon")) * radians * 6378137 *
                        std::cos(number(a, "lat") * radians)};
    const auto velocity = vector_value(a["velocity"]);
    const Vec target = vector_value(a) + velocity * elapsed - moved;
    const double speed_error = a.value("velocityTrusted", false)
                                   ? number(p, "velocity_uncertainty_mps", .2)
                                   : number(p, "max_target_speed_mps", 3),
                 acceleration = number(p, "acceleration_bound_mps2", .6),
                 accept_age = (number(a, "at") - number(a, "measurementAt")) / 1000;
    const auto radius = [&](double time) {
        return number(a, "uncertainty") + speed_error * (elapsed + time) +
               .5 * acceleration * (squared(age + time) - squared(accept_age)) + .10 * time +
               .10 * time * time;
    };
    const bool adaptive = m.value("clearance_policy", std::string()) == "adaptive";
    bool quarantine = false;
    for (const auto &s : sensors)
        quarantine |= quarantined(s);
    const bool live = !quarantine && direction.is_object() && direction.value("trusted", false) &&
                      recent(number(direction, "at"), now, number(p, "measurement_gap_ms", 450));
    const double initial_angle = wrap((live ? number(direction, "headingDeg") : target.angle()) -
                                      number(att, "yaw")),
                 safety = number(c, "safety_distance_m", .3),
                 reserve = .10 + .10 * clamp(std::abs(u), 0, 1) +
                           .15 * clamp(std::abs(initial_angle) / 45, 0, 1),
                 buffer =
                     adaptive ? safety + reserve : std::max(safety, number(p, "clearance_m", 1.2)),
                 brake = number(p, "braking_deceleration_mps2", .3);
    const auto stop = [&](double speed) {
        return .6 * std::max(0.0, speed) + squared(std::max(0.0, speed)) / (2 * brake);
    };
    const double initial_range = target.norm(), initial_radius = radius(0),
                 infinity = std::numeric_limits<double>::infinity(),
                 initial_lower = std::min(initial_range - initial_radius,
                                          number(evidence, "lowerBoundM", infinity));
    const Json diagnostic = {{"initialRangeLowerBoundM", initial_lower},
                             {"initialStoppingDistanceM", stop(u)},
                             {"bufferM", buffer},
                             {"initialRangeM", initial_range},
                             {"initialUncertaintyM", initial_radius},
                             {"initialBearingDeg", initial_angle},
                             {"initialForwardVelocityMps", u},
                             {"safetyDistanceM", safety},
                             {"modellingReserveM", adaptive ? reserve : buffer - safety},
                             {"clearancePolicy", adaptive ? "adaptive" : "legacy"},
                             {"rangeEvidence", evidence},
                             {"measurementAt", a["measurementAt"]},
                             {"measurementAgeMs", now - number(a, "measurementAt")}};
    if (initial_lower - buffer - stop(u) <= 0) {
        auto r = fail("no_initial_braking_clearance");
        r.update(diagnostic);
        return r;
    }
    const double maximum = std::max(0.0, std::min({1.0, number(c, "max_speed_mps", 2),
                                                   c.value("commissioning_mode", false)
                                                       ? number(c, "commissioning_speed_mps", 2)
                                                       : 2})),
                 yaw_max = std::min(14.0, number(c, "max_yaw_rate_deg_s", 14)),
                 command_dt =
                     command.is_object() ? clamp((now - number(command, "at")) / 1000, 0, .25) : .1;
    const bool recent_command = command.is_object() && recent(number(command, "at"), now,
                                                              number(p, "measurement_gap_ms", 450));
    const double previous_speed = recent_command ? number(command, "speed") : std::max(0.0, u),
                 previous_yaw =
                     recent_command ? number(command, "yaw") : number(att, "yawRateDegS"),
                 budget = clamp(number(m, "solve_budget_ms", 20), 1, 25);
    using Controls = std::array<std::array<double, 2>, 4>;
    int evaluations = 0;
    bool timed_out = false;
    const auto project = [&](Controls controls, double block_dt) {
        double speed = previous_speed, turn = previous_yaw;
        for (size_t i = 0; i < 4; ++i) {
            const double dt = i == 0 ? command_dt : block_dt;
            speed = clamp(controls[i][0], 0, std::min(maximum, std::max(0.0, speed) + brake * dt));
            turn = clamp(controls[i][1], std::max(-yaw_max, turn - 40 * dt),
                         std::min(yaw_max, turn + 40 * dt));
            controls[i] = {speed, turn};
        }
        return controls;
    };
    const auto rollout = [&](const Controls &controls, double horizon, bool record = false) {
        ++evaluations;
        const double dt = horizon / 12;
        double x = 0, y = 0, psi = yaw, speed = u, turn = number(att, "yawRateDegS"), cost = 0,
               min_clearance = infinity, max_fov = 0, prior_speed = previous_speed,
               prior_yaw = previous_yaw;
        Json path = Json::array();
        for (int k = 0; k < 4; ++k) {
            const auto [v, w] = controls[k];
            cost += 3 * squared(v - prior_speed) + .004 * squared(w - prior_yaw);
            prior_speed = v;
            prior_yaw = w;
            for (int j = 0; j < 3; ++j) {
                const double time = (k * 3 + j + 1) * dt;
                speed += clamp((v - speed) / .7, -brake, .8) * dt;
                turn += clamp((w - turn) / .35, -40, 40) * dt;
                psi += turn * radians * dt;
                x += (speed * std::cos(psi) + drift.x) * dt;
                y += (speed * std::sin(psi) + drift.y) * dt;
                const Vec relative{target.x + velocity.x * time - x,
                                   target.y + velocity.y * time - y};
                const double range = relative.norm(), uncertainty = radius(time),
                             raw_lower = std::isfinite(number(evidence, "lowerBoundM"))
                                             ? number(evidence, "lowerBoundM") +
                                                   (range - initial_range) -
                                                   (uncertainty - initial_radius)
                                             : infinity,
                             lower = std::min(range - uncertainty, raw_lower),
                             clearance = lower - buffer - stop(std::max(speed, v)),
                             heading = live ? number(direction, "headingDeg") +
                                                  number(direction, "rateDegS") * time
                                            : relative.angle(),
                             angle = wrap(heading - psi / radians),
                             angle_error =
                                 live
                                     ? 6 + 3 * time
                                     : std::asin(clamp(uncertainty / std::max(range, .001), 0, 1)) /
                                           radians,
                             fov = std::abs(angle) + angle_error;
                if (!std::isfinite(clearance) || clearance <= 0 || !std::isfinite(fov) || fov >= 45)
                    return Json{{"feasible", false}, {"cost", infinity}};
                min_clearance = std::min(min_clearance, clearance);
                max_fov = std::max(max_fov, fov);
                const double desired = std::max(buffer + .4, number(c, "following_distance_m", .5));
                cost += dt * (2 * squared(range - desired) + .012 * squared(angle) +
                              .12 * squared(turn) + .05 * v * v);
                if (record)
                    path.push_back({{"timeS", time},
                                    {"ownNorthM", x},
                                    {"ownEastM", y},
                                    {"targetNorthM", target.x + velocity.x * time},
                                    {"targetEastM", target.y + velocity.y * time},
                                    {"rangeM", range},
                                    {"uncertaintyM", uncertainty},
                                    {"speedMps", speed},
                                    {"commandSpeedMps", v},
                                    {"brakingDistanceM", stop(std::max(speed, v))},
                                    {"clearanceM", clearance},
                                    {"rangeLowerBoundM", lower},
                                    {"rawRangeLowerBoundM", nullable(raw_lower)},
                                    {"bearingBoundDeg", fov}});
            }
        }
        return Json{{"feasible", true},
                    {"cost", cost},
                    {"minClearanceM", min_clearance},
                    {"maxBearingBoundDeg", max_fov},
                    {"path", path}};
    };
    Json best = nullptr;
    for (const double horizon : {1.2, .6, .3}) {
        if (age + horizon >= loss / 1000)
            continue;
        const double heading = live ? number(direction, "headingDeg") : target.angle(),
                     aim = clamp(.85 * wrap(heading - number(att, "yaw")) -
                                     .3 * number(att, "yawRateDegS"),
                                 -yaw_max, yaw_max);
        std::vector<Controls> seeds;
        if (plan.is_object())
            seeds.push_back(plan["controls"].get<Controls>());
        for (const double speed : {previous_speed, 0.0, maximum}) {
            Controls seed;
            seed.fill({speed, aim});
            seeds.push_back(seed);
        }
        Json winner = nullptr;
        for (const auto &seed : seeds) {
            const auto controls = project(seed, horizon / 4);
            auto r = rollout(controls, horizon);
            if (r["feasible"] == true &&
                (!winner.is_object() || number(r, "cost") < number(winner, "cost"))) {
                r["controls"] = controls;
                r["horizon"] = horizon;
                winner = r;
            }
        }
        if (!winner.is_object())
            continue;
        for (const double scale : {1.0, .5, .25}) {
            for (int k = 0; k < 4; ++k)
                for (int axis = 0; axis < 2; ++axis)
                    for (const double sign : {-1.0, 1.0}) {
                        if (elapsed_ms(start) > budget) {
                            timed_out = true;
                            break;
                        }
                        auto trial = winner["controls"].get<Controls>();
                        trial[k][axis] += sign * scale * (axis == 0 ? .3 : 6);
                        const auto controls = project(trial, horizon / 4);
                        auto r = rollout(controls, horizon);
                        if (r["feasible"] == true && number(r, "cost") < number(winner, "cost")) {
                            r["controls"] = controls;
                            r["horizon"] = horizon;
                            winner = r;
                        }
                    }
            if (timed_out)
                break;
        }
        best = winner;
        break;
    }
    timed_out |= elapsed_ms(start) > budget;
    if (!best.is_object() || timed_out) {
        plan = nullptr;
        auto r = fail(timed_out ? "solver_deadline" : "no_feasible_trajectory");
        r.update(diagnostic);
        return r;
    }
    const auto controls = best["controls"].get<Controls>();
    const auto audit = rollout(controls, number(best, "horizon"), true);
    if (audit["feasible"] != true)
        return fail("solution_validation_failed");
    if (elapsed_ms(start) > budget) {
        plan = nullptr;
        return fail("solver_deadline");
    }
    plan = best;
    const auto [speed, turn] = controls[0];
    command = {{"at", now}, {"speed", speed}, {"yaw", turn}};
    Json out = {{"feasible", true},
                {"reason", "reject_nmpc"},
                {"forwardSpeedMps", speed},
                {"yawRateDegS", turn},
                {"measurementAt", a["measurementAt"]},
                {"measurementAgeMs", now - number(a, "measurementAt")},
                {"velocityTrusted", a["velocityTrusted"]},
                {"horizonS", best["horizon"]},
                {"evaluations", evaluations},
                {"solveMs", elapsed_ms(start)}};
    out.update(diagnostic);
    out.update({{"maxSpeedMps", maximum},
                {"liveBearing", live},
                {"minClearanceM", audit["minClearanceM"]},
                {"maxBearingBoundDeg", audit["maxBearingBoundDeg"]},
                {"path", audit["path"]},
                {"controls", controls},
                {"cost", best["cost"]}});
    return out;
}
} // namespace manta
