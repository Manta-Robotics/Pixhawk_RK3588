#include "manta/follow_control.hpp"

namespace manta {
void RelativeNmpc::reset() {
    at = measurement_at = missing;
    identity.clear();
    history.clear();
    model_surge = applied_throttle = applied_steering = 0;
    plan.reset();
}
void RelativeNmpc::track_applied(double throttle, double steering) {
    applied_throttle = std::isfinite(throttle) ? std::max(0.0, throttle) : 0;
    applied_steering = std::isfinite(steering) ? steering : 0;
}
RelativeNmpc::Controls RelativeNmpc::project(const Controls &input, double dt) const {
    Controls result{};
    double throttle = applied_throttle, steering = applied_steering;
    const double step = number(c["relativeNmpc"], "horizon_s") / 4,
                 share = number(c["relativeNmpc"], "steering_share");
    for (size_t i = 0; i < 4; ++i) {
        const double h = i == 0 ? dt : step;
        throttle =
            clamp(input[i][0], std::max(0.0, throttle - number(c, "throttleDecelPercentS") * h),
                  std::min(number(c, "relativeThrottleLimitPercent"),
                           throttle + number(c, "throttleAccelPercentS") * h));
        const double limit =
            std::max(0.0, std::min({number(c, "relativeSteeringLimitDeg"), throttle * .45 * share,
                                    (100 - throttle) * .45}));
        steering = clamp(clamp(input[i][1], steering - number(c, "steeringSlewDegS") * h,
                               steering + number(c, "steeringSlewDegS") * h),
                         -limit, limit);
        result[i] = {throttle, steering};
    }
    return result;
}
std::string RelativeNmpc::observe(double now, const Json &t, double range, double bearing,
                                  const Json &estimate) {
    const auto u = t.value("uwb", Json::object()),
               ids = u.value("activeReceiverIds", Json::array());
    std::vector<std::string> tags;
    bool sensors_trusted = true;
    for (const auto &s : u.value("sensors", Json::array()))
        if (std::find(ids.begin(), ids.end(), s["receiverId"]) != ids.end()) {
            tags.push_back(s["receiverId"].dump() + ":" +
                           (s.value("tagId", Json(nullptr)).is_null() ? "" : s["tagId"].dump()));
            if (number(s, "filterAdmissionVersion", 0) == 1)
                sensors_trusted &= s.value("measurementStatus", std::string()) == "trusted" &&
                                   s.value("valid", Json(nullptr)) != false &&
                                   s.value("receivedMeasurementAt", Json(nullptr)) ==
                                       s.value("measurementUpdatedAt", Json(nullptr));
        }
    std::sort(tags.begin(), tags.end());
    std::string id;
    for (const auto &tag : tags) {
        if (!id.empty())
            id += '|';
        id += tag;
    }
    if (!identity.empty() && !id.empty() && identity != id) {
        reset();
        return "target_identity_changed";
    }
    if (!id.empty())
        identity = id;
    const double sample = measurement_time(u);
    const bool admitted =
        u.value("online", false) && u.value("fresh", false) &&
        (estimate.is_object() ? estimate.value("measurementConfirmed", false)
                              : u.value("measurementStatus", std::string()) == "trusted") &&
        sensors_trusted && std::isfinite(sample) && sample <= now &&
        now - sample <= number(c, "uwbTimeoutMs");
    if (admitted && (!std::isfinite(measurement_at) || sample > measurement_at)) {
        const double yaw = number(u, "measurementHeadingDeg", number(t["attitude"], "yaw") -
                            number(t["attitude"], "yawRateDegS") * (now - sample) / 1000) *
                           radians,
                     angle = yaw + bearing * radians;
        std::erase_if(history, [&](const Point &p) { return sample - p.at > 900; });
        history.push_back({sample, {range * std::cos(angle), range * std::sin(angle)}});
        measurement_at = sample;
    }
    return "";
}
std::pair<Vec, bool> RelativeNmpc::relative_velocity(double yaw) const {
    if (history.size() < 3 || history.back().at - history.front().at < 200)
        return {{0, 0}, false};
    std::vector<double> sx, sy;
    for (size_t i = 0; i + 1 < history.size(); ++i)
        for (size_t j = i + 1; j < history.size(); ++j) {
            const double dt = (history[j].at - history[i].at) / 1000;
            if (dt < .18)
                continue;
            sx.push_back((history[j].z.x - history[i].z.x) / dt);
            sy.push_back((history[j].z.y - history[i].z.y) / dt);
        }
    if (sx.empty())
        return {{0, 0}, false};
    const auto median = [](std::vector<double> &values) {
        std::sort(values.begin(), values.end());
        const auto n = values.size();
        return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
    };
    const double x = median(sx), y = median(sy),
                 scale = std::min(1.0, number(c["prediction"], "max_target_speed_mps", 3) /
                                           std::max(.001, std::hypot(x, y)));
    return {{scale * (x * std::cos(yaw) + y * std::sin(yaw)),
             scale * (-x * std::sin(yaw) + y * std::cos(yaw))},
            true};
}
Json RelativeNmpc::solve(double now, const Json &t, double range, double bearing,
                         const Json &estimate, double reference, bool gps_available) {
    const auto started = std::chrono::steady_clock::now();
    const auto elapsed = [&]() {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
    };
    const auto &o = c["relativeNmpc"];
    const auto &p = c["prediction"];
    const std::string mode = gps_available ? "gps_imu" : "uwb_imu_relative";
    const auto fail = [&](const std::string &reason) {
        plan.reset();
        return Json{{"feasible", false},     {"reason", reason},
                    {"mode", mode},          {"solveMs", elapsed()},
                    {"path", Json::array()}, {"absoluteSpeedObservable", gps_available}};
    };
    if (!o.value("enabled", false))
        return fail("disabled");
    if (std::isfinite(at) && (now <= at || now - at > number(c, "uwbTimeoutMs"))) {
        reset();
        return fail("control_clock_gap");
    }
    const double dt = std::isfinite(at) ? clamp((now - at) / 1000, .001, .25) : .1;
    at = now;
    const auto attitude = t.value("attitude", Json::object());
    const double yaw = number(attitude, "yaw") * radians, r0 = number(attitude, "yawRateDegS"),
                 imu_at = number(attitude, "updatedAt");
    if (!std::isfinite(range) || range <= number(c, "safetyDistanceM") || !std::isfinite(bearing) ||
        !std::isfinite(yaw) || !std::isfinite(r0) || !std::isfinite(imu_at) || imu_at > now ||
        now - imu_at > number(c, "navigationTimeoutMs"))
        return fail("invalid_relative_state");
    const auto fault = observe(now, t, range, bearing, estimate);
    if (!fault.empty())
        return fail(fault);
    const double accepted =
        estimate.is_object()
            ? number(estimate, "lastMeasuredAt",
                     measurement_time(t.value("uwb", Json::object())))
            : measurement_time(t.value("uwb", Json::object()));
    const double max_age =
        estimate.is_object() && estimate.value("source", std::string()) == "predicted"
            ? number(p, "horizon_ms", 1000)
            : number(c, "uwbTimeoutMs");
    if (!std::isfinite(accepted) || accepted > now || now - accepted > max_age ||
        !std::isfinite(measurement_at))
        return fail("accepted_state_stale");
    const auto [rv, trusted] = relative_velocity(yaw);
    if (!trusted && !gps_available)
        return fail("relative_motion_warming_up");
    const double deceleration = number(p, "braking_deceleration_mps2", .3),
                 speed_limit =
                     std::min(number(c, "maxSpeedMps"), c.value("commissioningMode", true)
                                                            ? number(c, "commissioningSpeedMps")
                                                            : number(c, "maxSpeedMps"));
    const double throttle_limit = std::max(1.0, number(c, "relativeThrottleLimitPercent")),
                 cruise_speed = number(o, "model_cruise_speed_mps"),
                 surge_tau = number(o, "surge_time_constant_s"),
                 exponent = number(o, "surge_map_exponent");
    const auto surge_target = [&](double q) {
        return std::min(speed_limit,
                        std::pow(clamp(q / throttle_limit, 0, 1), exponent) * cruise_speed);
    };
    const auto acceleration = [&](double u, double q) {
        const double desired = surge_target(q);
        return clamp(
            cruise_speed / (2 * surge_tau) *
                (squared(desired / cruise_speed) - u * std::abs(u) / squared(cruise_speed)),
            -deceleration, deceleration);
    };
    model_surge += acceleration(model_surge, applied_throttle) * dt;
    if (gps_available) {
        const double vx = number(t.value("velocity", Json::object()), "vx"),
                     vy = number(t.value("velocity", Json::object()), "vy");
        if (!std::isfinite(vx) || !std::isfinite(vy))
            return fail("gps_velocity_invalid");
        model_surge = vx * std::cos(yaw) + vy * std::sin(yaw);
    }
    const double u0 = model_surge, x0 = range * std::cos(bearing * radians),
                 y0 = range * std::sin(bearing * radians);
    const Vec target = {rv.x + u0, rv.y};
    double covariance = number(p, "measurement_noise_m", .12);
    for (const auto &s : t.value("uwb", Json::object()).value("sensors", Json::array()))
        covariance = std::max(covariance, number(s, "filterUncertaintyM", 0));
    if (estimate.is_object())
        covariance = number(estimate, "uncertaintyM", covariance);
    if (!std::isfinite(covariance) || covariance < 0)
        return fail("invalid_uncertainty");
    const double horizon = number(o, "horizon_s"),
                 model_error = number(o, "model_position_error_mps"),
                 velocity_uncertainty = number(p, "velocity_uncertainty_mps", .2);
    const double horizon_error = covariance + model_error * horizon + .05 * squared(horizon),
                 approach = .35 + .5 * velocity_uncertainty;
    const double desired = std::max(number(c, "followingDistanceM"),
                                    number(c, "safetyDistanceM") + horizon_error + .6 * approach +
                                        squared(approach) / (2 * deceleration) + .3);
    const double h = horizon / 12, steering_limit = number(c, "relativeSteeringLimitDeg"),
                 yaw_limit = number(c, "maxYawRateDegS"),
                 yaw_tau = number(o, "yaw_time_constant_s"), safety = number(c, "safetyDistanceM"),
                 fov = number(c, "fieldOfViewHalfDeg"), budget = number(o, "solve_budget_ms");
    struct Candidate {
        Controls controls;
        Json path;
        double cost, clearance;
    };
    std::optional<Candidate> best;
    int evaluations = 0;
    const auto rollout = [&](const Controls &controls) -> std::optional<Candidate> {
        ++evaluations;
        double x = 0, y = 0, psi = 0, u = u0, r = r0, cost = 0, min_clearance = INFINITY,
               prior_t = applied_throttle, prior_s = applied_steering;
        Json path = Json::array();
        for (int i = 0; i < 12; ++i) {
            const auto &q = controls[i / 3];
            const double time = (i + 1) * h, turn = q[1] / steering_limit * yaw_limit;
            u += acceleration(u, q[0]) * h;
            r += clamp((turn - r) / yaw_tau, -40, 40) * h;
            psi += r * radians * h;
            x += u * std::cos(psi) * h;
            y += u * std::sin(psi) * h;
            const double px = x0 + target.x * time - x, py = y0 + target.y * time - y,
                         d = std::hypot(px, py),
                         angle = wrap(std::atan2(py, px) / radians - psi / radians);
            const double closing = std::max(
                0.0, -((target.x - u * std::cos(psi)) * px + (target.y - u * std::sin(psi)) * py) /
                         std::max(.01, d));
            const double error = covariance + model_error * time + .05 * time * time,
                         closing_upper = closing + .5 * velocity_uncertainty,
                         stopping =
                             .6 * closing_upper + squared(closing_upper) / (2 * deceleration),
                         clearance = d - error - safety - stopping;
            if (!std::isfinite(clearance) || clearance <= 0 || std::abs(angle) > fov)
                return std::nullopt;
            min_clearance = std::min(min_clearance, clearance);
            const double desired_closing = clamp(.5 * (d - desired), 0, .35);
            cost += h * ((d < desired ? 12 : 1.2) * squared(d - desired) +
                         25 * squared(angle * radians) + 4 * squared(closing - desired_closing) +
                         .3 * squared((q[0] - reference) / throttle_limit) +
                         .01 * squared(r / yaw_limit));
            if (i % 3 == 0) {
                cost += .2 * squared((q[0] - prior_t) / throttle_limit) +
                        .2 * squared((q[1] - prior_s) / steering_limit);
                prior_t = q[0];
                prior_s = q[1];
            }
            path.push_back({{"timeS", time},
                            {"relativeXM", px * std::cos(psi) + py * std::sin(psi)},
                            {"relativeYM", -px * std::sin(psi) + py * std::cos(psi)},
                            {"rangeM", d},
                            {"bearingDeg", angle},
                            {"uncertaintyM", error},
                            {"closingSpeedMps", closing},
                            {"stoppingDistanceM", stopping},
                            {"clearanceM", clearance}});
        }
        cost += 40 * squared(number(path.back(), "bearingDeg") * radians);
        return Candidate{controls, path, cost, min_clearance};
    };
    const double aim =
        clamp(.85 * bearing - .6 * r0, -yaw_limit, yaw_limit) / yaw_limit * steering_limit;
    const auto constant = [](double t, double s) {
        Controls value;
        value.fill({t, s});
        return value;
    };
    std::vector<Controls> seeds;
    if (plan) {
        auto shifted = *plan;
        for (int i = 0; i < 3; ++i)
            shifted[i] = (*plan)[i + 1];
        seeds.push_back(shifted);
    }
    seeds.push_back(constant(reference, aim));
    seeds.push_back(constant(applied_throttle, aim));
    for (double fraction : {.05, .15, .3, .5, 1.0})
        seeds.push_back(constant(throttle_limit * fraction, aim));
    seeds.push_back(constant(0, 0));
    const auto consider = [&](const Controls &seed) {
        auto candidate = rollout(project(seed, dt));
        if (candidate && (!best || candidate->cost < best->cost))
            best = std::move(candidate);
    };
    bool deadline = false;
    for (const auto &seed : seeds) {
        consider(seed);
        if (elapsed() >= budget) {
            deadline = true;
            break;
        }
    }
    if (!best)
        return fail("no_feasible_trajectory");
    for (double scale : {1.0, .5, .25}) {
        if (deadline)
            break;
        for (int k = 0; k < 4 && !deadline; ++k)
            for (int axis = 0; axis < 2 && !deadline; ++axis)
                for (int sign : {-1, 1}) {
                    if (elapsed() >= budget) {
                        deadline = true;
                        break;
                    }
                    auto q = best->controls;
                    q[k][axis] += sign * scale * (axis == 0 ? throttle_limit * .2 : 6);
                    consider(q);
                }
    }
    const double solve_ms = elapsed();
    if (solve_ms > std::max(30.0, budget * 2))
        return fail("solver_overrun");
    plan = best->controls;
    const auto q = best->controls[0];
    return {{"feasible", true},
            {"reason", "relative_nmpc"},
            {"mode", mode},
            {"throttlePercent", q[0]},
            {"steeringDegrees", q[1]},
            {"nominalSpeedMps", surge_target(q[0])},
            {"yawRateDegS", q[1] / steering_limit * yaw_limit},
            {"horizonS", horizon},
            {"solveMs", solve_ms},
            {"evaluations", evaluations},
            {"deadlineReached", deadline},
            {"path", best->path},
            {"cost", best->cost},
            {"minClearanceM", best->clearance},
            {"measurementAt", accepted},
            {"relativeMotionConfirmed", trusted},
            {"effectiveFollowingDistanceM", desired},
            {"absoluteSpeedObservable", gps_available},
            {"modelSurgeMps", u0},
            {"modelSurgeMeasured", gps_available}};
}
} // namespace manta
