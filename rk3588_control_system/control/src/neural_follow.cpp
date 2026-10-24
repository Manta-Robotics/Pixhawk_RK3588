#include "manta/neural_follow.hpp"
#include <filesystem>
#include <fstream>

namespace manta {
namespace {
Vec rotate(Vec p, double angle) {
    return {p.x * std::cos(angle) - p.y * std::sin(angle),
            p.x * std::sin(angle) + p.y * std::cos(angle)};
}
double wrap_rad(double value) {
    return std::fmod(std::fmod(value + pi, 2 * pi) + 2 * pi, 2 * pi) - pi;
}
double neural_wrap(double value) {
    return std::fmod(std::fmod(value + 180, 360) + 360, 360) - 180;
}
} // namespace
JsonActor::JsonActor(const std::string &path)
    : model_path(std::filesystem::absolute(path).string()) {
    std::ifstream stream(path);
    if (!stream)
        throw std::invalid_argument("neural_policy_file_missing");
    const auto model = Json::parse(stream);
    if (model.value("format", std::string()) != "smart_usv_rsl_rl_actor_v1")
        throw std::invalid_argument("unsupported_neural_policy_format");
    if (model.at("observationNames").size() != 20 || model.at("layers").size() != 4)
        throw std::invalid_argument("invalid_neural_policy_dimensions");
    mean = model.at("normalizer").at("mean").get<std::vector<double>>();
    stddev = model.at("normalizer").at("std").get<std::vector<double>>();
    epsilon = model.at("normalizer").at("epsilon");
    if (mean.size() != 20 || stddev.size() != 20 || !std::isfinite(epsilon) || epsilon <= 0)
        throw std::invalid_argument("invalid_neural_normalizer");
    for (size_t i = 0; i < 20; ++i)
        if (!std::isfinite(mean[i]) || !std::isfinite(stddev[i]) || stddev[i] < 0)
            throw std::invalid_argument("invalid_neural_normalizer");
    size_t width = 20;
    for (const auto &layer : model["layers"]) {
        Layer l{layer.at("weight").get<std::vector<std::vector<double>>>(),
                layer.at("bias").get<std::vector<double>>(),
                layer.value("activation", std::string()) == "elu"};
        if (l.weight.empty() || l.weight.size() != l.bias.size())
            throw std::invalid_argument("invalid_neural_policy_dimensions");
        for (const auto &row : l.weight) {
            if (row.size() != width)
                throw std::invalid_argument("invalid_neural_policy_dimensions");
            for (double v : row)
                if (!std::isfinite(v))
                    throw std::invalid_argument("invalid_neural_weight");
        }
        for (double b : l.bias)
            if (!std::isfinite(b))
                throw std::invalid_argument("invalid_neural_bias");
        width = l.bias.size();
        layers.push_back(std::move(l));
    }
    if (width != 2)
        throw std::invalid_argument("invalid_neural_action_dimensions");
    const auto &golden = model.at("golden");
    if (golden.at("observations").empty() ||
        golden["observations"].size() != golden.at("actions").size())
        throw std::invalid_argument("neural_policy_golden_vectors_missing");
    for (size_t i = 0; i < golden["observations"].size(); ++i) {
        const auto actual = infer(golden["observations"][i].get<std::vector<double>>());
        for (size_t j = 0; j < actual.size(); ++j)
            if (std::abs(actual[j] - golden["actions"][i][j].get<double>()) > 2e-5)
                throw std::invalid_argument("neural_policy_self_test_failed");
    }
}
std::vector<double> JsonActor::infer(const std::vector<double> &observation) const {
    if (observation.size() != 20)
        throw std::invalid_argument("invalid_neural_observation");
    std::vector<double> values(20);
    for (size_t i = 0; i < 20; ++i) {
        if (!std::isfinite(observation[i]))
            throw std::invalid_argument("invalid_neural_observation");
        values[i] = (observation[i] - mean[i]) / (stddev[i] + epsilon);
    }
    for (const auto &layer : layers) {
        std::vector<double> output(layer.bias.size());
        for (size_t i = 0; i < output.size(); ++i) {
            double value = layer.bias[i];
            for (size_t j = 0; j < values.size(); ++j)
                value += layer.weight[i][j] * values[j];
            output[i] = layer.elu && value < 0 ? std::expm1(value) : value;
        }
        values = std::move(output);
    }
    for (auto &value : values)
        value = clamp(value, -1, 1);
    return values;
}
NeuralFollow::NeuralFollow(const Json &raw)
    : FollowController(raw), actor(raw.at("neural_model_path")),
      maximum(clamp(number(raw, "neural_max_speed_mps", 3), .1, 3)),
      stop_distance(std::max(.1, number(raw, "neural_safety_stop_distance_m", .65))),
      resume_distance(std::max(.2, number(raw, "neural_safety_resume_distance_m", .8))),
      velocity_alpha(clamp(number(raw, "neural_target_velocity_alpha", .25), .01, 1)) {
    if (resume_distance <= stop_distance)
        throw std::invalid_argument("neural_safety_resume_must_exceed_stop");
    reset();
}
void NeuralFollow::reset() {
    FollowController::reset();
    initialized = false;
    filter = {0, 0, 0};
    uncertainty = {1, 1, 1};
    measurement_at = previous_heading = missing;
    last_relative = target_velocity = {0, 0};
    previous_speed = neural_throttle = neural_steering = 0;
    safety_latched = false;
}
Json NeuralFollow::preflight(double now, const Json &t, bool connected, const std::string &,
                             const Json &estimate) const {
    auto out = FollowController::preflight(now, t, connected, "relative_acro", estimate);
    const auto pilot = t.value("pilotSteering", Json::object());
    const double type = number(pilot, "type");
    const bool ready = pilot.value("ready", false) && std::isfinite(type) &&
                       type == std::trunc(type) && type >= 0 && type <= 3;
    if (out.value("safe", false) && !ready)
        out.update({{"safe", false}, {"reason", "pilot_steering_not_ready"}});
    const bool acro = config["executionMode"] == "acro";
    out.update({{"trackingMethod", "neural"},
                {"controlMode", acro ? "relative_acro" : "relative_manual"},
                {"requiredFlightMode", acro ? "ACRO" : "MANUAL"},
                {"commandFrame", "BODY_RELATIVE"},
                {"propulsion", acro ? "acro_velocity" : "dual_thrust"}});
    return out;
}
void NeuralFollow::ingest(const Json &t, const Json &pf) {
    const auto u = t.value("uwb", Json::object());
    const double at =
        number(u, u.contains("measurementUpdatedAt") ? "measurementUpdatedAt" : "updatedAt");
    if (!std::isfinite(at) || at == measurement_at)
        return;
    const std::array<double, 3> raw = {number(pf, "distanceM"), number(pf, "azimuthDeg"),
                                       number(u, "elevationDeg", 0)};
    if (!initialized) {
        filter = raw;
        initialized = true;
    } else {
        for (size_t i = 0; i < 3; ++i) {
            const double predicted = uncertainty[i] + (i == 0 ? .01 : .8),
                         gain = predicted / (predicted + (i == 0 ? .16 : 9));
            double residual = raw[i] - filter[i];
            if (i > 0)
                residual = neural_wrap(residual);
            filter[i] += gain * residual;
            if (i > 0)
                filter[i] = neural_wrap(filter[i]);
            uncertainty[i] = (1 - gain) * predicted;
        }
    }
    const double bearing = neural_wrap(filter[1] * number(config, "bearingSign") *
                                           number(config, "bearingScale") +
                                       number(config, "bearingOffsetDeg")) *
                           radians,
                 yaw = wrap_rad(pi / 2 - number(pf, "yawDeg") * radians);
    const auto relative_enu =
        rotate({filter[0] * std::cos(bearing), -filter[0] * std::sin(bearing)}, yaw);
    if (std::isfinite(measurement_at)) {
        const double dt = (at - measurement_at) / 1000;
        if (dt >= .03 && dt <= .6) {
            const auto velocity = t.value("velocity", Json::object());
            const Vec measured = {
                (relative_enu.x - last_relative.x) / dt + number(velocity, "vy", 0),
                (relative_enu.y - last_relative.y) / dt + number(velocity, "vx", 0)};
            target_velocity = target_velocity * (1 - velocity_alpha) + measured * velocity_alpha;
        } else if (dt > .6)
            target_velocity = {0, 0};
    }
    last_relative = relative_enu;
    measurement_at = at;
}
std::vector<double> NeuralFollow::observation(double now, const Json &t, const Json &pf,
                                              const Json &estimate) {
    if (!estimate.is_object())
        ingest(t, pf);
    const double yaw = wrap_rad(pi / 2 - number(pf, "yawDeg") * radians),
                 bearing = neural_wrap(filter[1] * number(config, "bearingSign") *
                                           number(config, "bearingScale") +
                                       number(config, "bearingOffsetDeg")) *
                           radians;
    const auto target = rotate(target_velocity, -yaw),
               boat = rotate({number(pf, "boatVy", 0), number(pf, "boatVx", 0)}, -yaw),
               body = rotate(last_relative, -yaw);
    const double age =
                     std::max(0.0, (now - (estimate.is_object()
                                               ? number(estimate, "lastMeasuredAt", measurement_at)
                                               : measurement_at)) /
                                       1000),
                 heading = std::isfinite(previous_heading) ? previous_heading
                                                           : number(pf, "yawDeg") * radians;
    return {filter[0] / 10,
            std::sin(bearing),
            std::cos(bearing),
            std::sin(filter[2] * radians),
            std::min(age, 2.0) / (number(config, "uwbTimeoutMs") / 1000),
            estimate.is_object() && estimate.value("source", std::string()) == "predicted" ? 0.0
                                                                                           : 1.0,
            body.x / 10,
            body.y / 10,
            target.x,
            target.y,
            boat.x,
            boat.y,
            std::sin(yaw),
            std::cos(yaw),
            -number(pf, "measuredYawRateDegS") * radians,
            previous_speed / maximum,
            std::sin(heading),
            std::cos(heading),
            number(config, "followingDistanceM") / 10,
            safety_latched ? 1.0 : 0.0};
}
Json NeuralFollow::relative_only(double now, const Json &t, const Json &pf, const Json &estimate) {
    if (t.value("flightMode", std::string()) != "MANUAL") {
        auto out = pf;
        out.update({{"safe", false}, {"reason", "manual_required"}});
        return out;
    }
    if (!t.value("armed", false)) {
        auto out = pf;
        out.update({{"safe", false}, {"reason", "armed_required"}});
        return out;
    }
    const auto n = [&](const char *key) { return number(config, key); };
    const double d = number(pf, "distanceM"),
                 bearing =
                     neural_wrap(number(pf, "azimuthDeg") * n("bearingSign") * n("bearingScale") +
                                 n("bearingOffsetDeg")) *
                     radians,
                 yaw = wrap_rad(pi / 2 - number(pf, "yawDeg") * radians);
    const Vec body = {d * std::cos(bearing), -d * std::sin(bearing)};
    const auto relative_enu = rotate(body, yaw);
    const double at =
        number(t["uwb"], "measurementUpdatedAt",
               estimate.is_object()            ? number(estimate, "lastMeasuredAt",
                                             std::isfinite(measurement_at) ? measurement_at : now)
               : std::isfinite(measurement_at) ? measurement_at
                                               : now);
    const auto source =
        estimate.is_object() ? estimate.value("source", std::string()) : std::string("measured");
    if ((source == "measured" || source == "recovering") && std::isfinite(at) &&
        (!std::isfinite(measurement_at) || at > measurement_at)) {
        const double dt = std::isfinite(measurement_at) ? (at - measurement_at) / 1000 : 0;
        target_velocity = dt >= .03 && dt <= .45
                              ? target_velocity * (1 - velocity_alpha) +
                                    (relative_enu - last_relative) * (velocity_alpha / dt)
                              : Vec{0, 0};
        last_relative = relative_enu;
        measurement_at = at;
    }
    const auto velocity = rotate(target_velocity, -yaw);
    const double previous =
        std::isfinite(previous_heading) ? previous_heading : number(pf, "yawDeg") * radians;
    const std::vector<double> obs = {d / 10,
                                     std::sin(bearing),
                                     std::cos(bearing),
                                     0,
                                     std::min(2.0, std::max(0.0, (now - at) / 1000)) /
                                         (n("uwbTimeoutMs") / 1000),
                                     source == "predicted" ? 0.0 : 1.0,
                                     body.x / 10,
                                     body.y / 10,
                                     velocity.x,
                                     velocity.y,
                                     0,
                                     0,
                                     std::sin(yaw),
                                     std::cos(yaw),
                                     -number(pf, "measuredYawRateDegS") * radians,
                                     previous_speed / maximum,
                                     std::sin(previous),
                                     std::cos(previous),
                                     n("followingDistanceM") / 10,
                                     d <= n("safetyDistanceM") ? 1.0 : 0.0};
    const auto action = actor.infer(obs);
    const double nominal = (action[0] + 1) * .5 * maximum,
                 heading = wrap_rad(number(pf, "yawDeg") * radians - pi * action[1]),
                 error = neural_wrap(heading / radians - number(pf, "yawDeg"));
    const double torque = clamp(
        .5 * (-error * radians) - .25 * (-number(pf, "measuredYawRateDegS") * radians), -.5, .5);
    auto result = relative.update(
        now, pf, t, estimate,
        {{"nominalSpeedMps", nominal},
         {"nominalThrottlePercent", std::min(n("maxSpeedMps"), nominal) / n("maxSpeedMps") *
                                        n("relativeThrottleLimitPercent")},
         {"headingErrorDeg", error},
         {"yawRateDegS", -torque / .5 * n("maxYawRateDegS")}});
    previous_speed = nominal;
    previous_heading = heading;
    const bool predictive = result.value("planner", std::string()) == "nmpc";
    result.update(
        {{"trackingMethod", "neural"},
         {"controllerProfile", "ppo_missing_absolute_velocity_experiment"},
         {"neuralModel", std::filesystem::path(actor.model_path).filename().string()},
         {"neuralAction", action},
         {"neuralObservation", obs},
         {"neuralAbsoluteVelocityMissing", true},
         {"neuralObservationAdaptation",
          "own absolute velocity imputed zero; target slots use relative velocity proxy"},
         {"neuralRole", predictive ? "nominal_throttle_reference" : "direct_relative_guidance"},
         {"speedFeedForwardSource", predictive ? result["speedFeedForwardSource"]
                                    : result.value("gpsSpeedControlActive", false)
                                        ? Json("gps_speed_pi")
                                        : Json("experimental_ppo_nominal_throttle")}});
    return result;
}
Json NeuralFollow::update(double now, const Json &t, bool connected, const std::string &mode,
                          const Json &estimate, const Json &motion) {
    auto pf = preflight(now, t, connected, mode, estimate);
    if (!pf.value("safe", false))
        return pf;
    if (config["executionMode"] == "relative_imu")
        return relative_only(now, t, pf, motion.is_object() ? motion : estimate);
    const auto n = [&](const char *key) { return number(config, key); };
    const double yaw = number(pf, "yawDeg"), yaw_rate = number(pf, "measuredYawRateDegS");
    if (estimate.is_object()) {
        filter = {number(pf, "distanceM"), number(pf, "azimuthDeg"),
                  number(t.value("uwb", Json::object()), "elevationDeg", 0)};
        last_relative = rotate(
            {number(estimate["relativeBodyM"], "x"), -number(estimate["relativeBodyM"], "y")},
            pi / 2 - yaw * radians);
        target_velocity = {number(estimate["targetVelocityNedMps"], "y"),
                           number(estimate["targetVelocityNedMps"], "x")};
    } else
        ingest(t, pf);
    if (number(pf, "distanceM") <= stop_distance)
        safety_latched = true;
    if (safety_latched && number(pf, "distanceM") >= resume_distance)
        safety_latched = false;
    const double d = filter[0],
                 bearing = neural_wrap(filter[1] * n("bearingSign") * n("bearingScale") +
                                       n("bearingOffsetDeg"));
    const auto body = rotate(last_relative, -(pi / 2 - yaw * radians));
    const double speed_limit = std::min(
        maximum, config.value("commissioningMode", true) ? n("commissioningSpeedMps") : maximum);
    Json out = {{"safe", true},
                {"trackingMethod", "neural"},
                {"controllerProfile", "smart_usv_ppo"},
                {"controlMode", pf["controlMode"]},
                {"requiredFlightMode", pf["requiredFlightMode"]},
                {"navigationAvailable", pf["navigationAvailable"]},
                {"navigationReason", pf["navigationReason"]},
                {"commandFrame", "BODY_RELATIVE"},
                {"relativeBodyM", {{"x", body.x}, {"y", body.y}}},
                {"distanceM", d},
                {"bearingDeg", bearing},
                {"controlBearingDeg", bearing},
                {"speedLimitMps", speed_limit},
                {"commissioningMode", config["commissioningMode"]},
                {"holdingDistanceM", stop_distance},
                {"hardStopResumeDistanceM", resume_distance}};
    if (safety_latched) {
        neural_throttle = neural_steering = previous_speed = 0;
        out.update({{"status", "holding"},
                    {"reason", "holding_distance"},
                    {"forwardSpeedMps", 0},
                    {"desiredForwardSpeedMps", 0},
                    {"yawRateDegS", 0},
                    {"yawRateRadS", 0},
                    {"throttlePercent", 0},
                    {"steeringDegrees", 0},
                    {"velocityNed", {{"x", 0}, {"y", 0}}},
                    {"desiredSpeedMps", 0},
                    {"desiredHeadingDeg",
                     std::isfinite(previous_heading) ? previous_heading / radians : yaw},
                    {"trackingPhase", "neural_safety_stop"}});
        return out;
    }
    auto actual = t.value("flightMode", std::string());
    for (auto &ch : actual)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    if (pf["requiredFlightMode"] != actual) {
        pf.update(
            {{"safe", false},
             {"reason", pf["requiredFlightMode"] == "ACRO" ? "acro_required" : "manual_required"}});
        return pf;
    }
    if (!t.value("armed", false)) {
        pf.update({{"safe", false}, {"reason", "armed_required"}});
        return pf;
    }
    const auto obs = observation(now, t, pf, estimate), action = actor.infer(obs);
    const double desired_speed = (action[0] + 1) * .5 * maximum,
                 yaw_enu = wrap_rad(pi / 2 - yaw * radians),
                 desired_yaw = wrap_rad(yaw_enu + pi * action[1]),
                 heading = wrap_rad(pi / 2 - desired_yaw);
    const double heading_deg = heading / radians - 360 * std::floor(heading / radians / 360),
                 error = neural_wrap(heading_deg - yaw),
                 forward = std::min(speed_limit, desired_speed) *
                           std::max(0.0, std::cos(error * radians));
    const double torque = clamp(.5 * (-error * radians) - .25 * (-yaw_rate * radians), -.5, .5),
                 requested_yaw = -torque / .5 * n("maxYawRateDegS");
    neural_throttle = forward / std::max(speed_limit, .1) * n("relativeThrottleLimitPercent");
    neural_steering = requested_yaw / n("maxYawRateDegS") * n("relativeSteeringLimitDeg");
    const double commanded_speed =
                     neural_throttle / n("relativeThrottleLimitPercent") * speed_limit,
                 commanded_yaw =
                     neural_steering / n("relativeSteeringLimitDeg") * n("maxYawRateDegS");
    previous_speed = desired_speed;
    previous_heading = heading;
    out.update(
        {{"reason", "tracking"},
         {"neuralModel", std::filesystem::path(actor.model_path).filename().string()},
         {"neuralAction", action},
         {"neuralObservation", obs},
         {"guidanceBearingDeg", error},
         {"headingErrorDeg", error},
         {"forwardSpeedMps", commanded_speed},
         {"desiredForwardSpeedMps", forward},
         {"commandedForwardSpeedMps", commanded_speed},
         {"desiredYawRateDegS", requested_yaw},
         {"yawRateDegS", commanded_yaw},
         {"yawRateRadS", commanded_yaw * radians},
         {"throttlePercent", neural_throttle},
         {"steeringDegrees", neural_steering},
         {"velocityNed",
          {{"x", commanded_speed * std::cos(yaw * radians)},
           {"y", commanded_speed * std::sin(yaw * radians)}}},
         {"desiredSpeedMps",
          config["executionMode"] == "acro" ? std::min(speed_limit, desired_speed) : desired_speed},
         {"policyDesiredSpeedMps", desired_speed},
         {"desiredHeadingDeg", heading_deg},
         {"trackingPhase", "neural_policy"}});
    return out;
}
} // namespace manta
