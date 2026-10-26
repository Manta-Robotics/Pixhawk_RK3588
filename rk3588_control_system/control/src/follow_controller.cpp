#include "manta/follow_control.hpp"

namespace manta {
namespace {
int sign(double value) {
    return (value > 0) - (value < 0);
}
double toward(double current, double target, double delta) {
    return current + clamp(target - current, -delta, delta);
}
} // namespace
Json FollowController::update(double now, const Json &t, bool connected, const std::string &mode,
                              const Json &estimate, const Json &motion) {
    const auto pf = preflight(now, t, connected, mode, estimate);
    if (!pf.value("safe", false))
        return pf;
    const auto &c = config;
    const auto n = [&](const char *key) { return number(c, key); };
    auto flight_mode = t.value("flightMode", std::string());
    for (auto &ch : flight_mode)
        ch = char(std::toupper(static_cast<unsigned char>(ch)));
    const auto selected = pf["controlMode"].get<std::string>(),
               required = pf["requiredFlightMode"].get<std::string>();
    if (c["executionMode"] == "relative_imu") {
        if (flight_mode != "MANUAL")
            return zero_control("manual_required", {{"controlMode", "relative_manual"},
                                                    {"requiredFlightMode", "MANUAL"}});
        if (!t.value("armed", false))
            return zero_control("armed_required", {{"controlMode", "relative_manual"},
                                                   {"requiredFlightMode", "MANUAL"}});
        return relative.update(now, pf, t, motion);
    }
    const double d = number(pf, "distanceM"), yaw = number(pf, "yawDeg"),
                 yaw_rate = number(pf, "measuredYawRateDegS"), vx = number(pf, "boatVx"),
                 vy = number(pf, "boatVy");
    const bool nav = pf["navigationAvailable"], acro = c["executionMode"] == "acro";
    const double bearing = wrap(number(pf, "azimuthDeg") * n("bearingSign") * n("bearingScale") +
                                n("bearingOffsetDeg"));
    filtered_bearing =
        std::isfinite(filtered_bearing)
            ? wrap(filtered_bearing + n("bearingFilterAlpha") * wrap(bearing - filtered_bearing))
            : bearing;
    const double control_bearing = filtered_bearing;
    const Vec body = {d * std::cos(bearing * radians), d * std::sin(bearing * radians)};
    const double speed_limit =
        std::min(n("maxSpeedMps"), c.value("commissioningMode", true) ? n("commissioningSpeedMps")
                                                                      : n("maxSpeedMps"));
    if (c.value("hardStopEnabled", true)) {
        if (d <= n("safetyDistanceM"))
            hard_stop = true;
        else if (hard_stop && d >= n("hardStopResumeDistanceM"))
            hard_stop = false;
    } else
        hard_stop = false;
    const bool close =
        c.value("closeApproachEnabled", true) && !hard_stop && d < n("safetyResumeDistanceM");
    if (!c.value("hardStopEnabled", true)) {
        if (d < n("safetyDistanceM"))
            distance_recovery = true;
        else if (distance_recovery && d >= n("distanceRecoveryResumeM"))
            distance_recovery = false;
    } else
        distance_recovery = false;
    const double hold_error = d - n("distanceRecoveryResumeM");
    const bool recovery = close && distance_recovery,
               hold = close && !recovery && d <= n("distanceHoldUpperM");
    const double progress =
        clamp((d - n("distanceHoldUpperM")) /
                  std::max(.01, n("safetyResumeDistanceM") - n("distanceHoldUpperM")),
              0, 1);
    const double close_speed =
        n("closeApproachMinSpeedMps") +
        (n("closeApproachMaxSpeedMps") - n("closeApproachMinSpeedMps")) * progress;
    const double close_throttle =
        n("closeApproachMinThrottlePercent") +
        (n("closeApproachMaxThrottlePercent") - n("closeApproachMinThrottlePercent")) * progress;
    Json out = {{"safe", true},
                {"controlMode", selected},
                {"requiredFlightMode", required},
                {"navigationAvailable", nav},
                {"navigationReason", pf["navigationReason"]},
                {"commandFrame", selected == "guided_velocity" ? "BODY_NED" : "BODY_RELATIVE"},
                {"relativeBodyM", {{"x", body.x}, {"y", body.y}}},
                {"distanceM", d},
                {"bearingDeg", bearing},
                {"rangeErrorM", d - n("followingDistanceM")},
                {"speedLimitMps", speed_limit},
                {"commissioningMode", c["commissioningMode"]},
                {"holdingDistanceM", n("safetyDistanceM")},
                {"hardStopResumeDistanceM", n("hardStopResumeDistanceM")}};
    if (hard_stop) {
        const bool held = hard_stop;
        reset();
        hard_stop = held;
        out.update({{"status", "holding"},
                    {"reason", "holding_distance"},
                    {"controlBearingDeg", bearing},
                    {"rangeRateMps", 0},
                    {"bearingRateDegS", 0},
                    {"targetRadialSpeedMps", 0},
                    {"rangeIntegralMps", 0},
                    {"boatTowardTargetMps", 0},
                    {"turnSlowdownScale", 0},
                    {"closeApproachActive", false},
                    {"trackingPhase", "hard_stop"},
                    {"forwardSpeedMps", 0},
                    {"yawRateDegS", 0},
                    {"yawRateRadS", 0},
                    {"throttlePercent", 0},
                    {"steeringDegrees", 0},
                    {"velocityNed", {{"x", 0}, {"y", 0}}},
                    {"desiredSpeedMps", 0}});
        return out;
    }
    const Json mode_extra = {{"controlMode", selected},
                             {"requiredFlightMode", required},
                             {"navigationAvailable", nav},
                             {"navigationReason", pf["navigationReason"]}};
    if (flight_mode != required)
        return zero_control(required == "MANUAL"   ? "manual_required"
                            : required == "GUIDED" ? "guided_required"
                                                   : "acro_required",
                            mode_extra);
    if (!t.value("armed", false))
        return zero_control("armed_required", mode_extra);
    const double absolute_bearing = (yaw + bearing) * radians,
                 boat_toward =
                     nav ? vx * std::cos(absolute_bearing) + vy * std::sin(absolute_bearing) : 0;
    double sample_dt = 0;
    if (std::isfinite(previous_at) && std::isfinite(previous_distance) &&
        std::isfinite(previous_bearing)) {
        const double dt = (now - previous_at) / 1000;
        if (dt >= .03 && dt <= .6) {
            sample_dt = dt;
            range_rate = (d - previous_distance) / dt;
            bearing_rate = bearing_rate * (1 - n("bearingRateAlpha")) +
                           wrap(control_bearing - previous_bearing) / dt * n("bearingRateAlpha");
            target_radial = nav ? target_radial * (1 - n("targetSpeedAlpha")) +
                                      (range_rate + boat_toward) * n("targetSpeedAlpha")
                                : 0;
        } else if (dt > .6)
            range_rate = bearing_rate = target_radial = range_integral = 0;
    }
    previous_distance = d;
    previous_bearing = control_bearing;
    previous_at = now;
    std::string feed_source = "range_difference";
    const bool motion_now = motion.is_object() && number(motion, "evaluatedAt") == now;
    const auto motion_velocity =
        motion.is_object() ? motion.value("targetVelocityNedMps", Json::object()) : Json::object();
    const bool motion_trusted =
        motion_now && motion.value("velocityEstimate", Json::object()).value("trusted", false) &&
        std::isfinite(number(motion_velocity, "x")) && std::isfinite(number(motion_velocity, "y"));
    if (acro && nav && motion_now && motion.contains("velocityEstimate") &&
        !motion["velocityEstimate"].is_null()) {
        target_radial = motion_trusted
                            ? number(motion_velocity, "x") * std::cos(absolute_bearing) +
                                  number(motion_velocity, "y") * std::sin(absolute_bearing)
                            : 0;
        if (motion_trusted)
            range_rate = target_radial - boat_toward;
        feed_source = motion_trusted ? "confirmed_motion" : "position_only";
    }
    const double range_error = d - n("followingDistanceM"),
                 controlled_error = std::abs(range_error) <= n("rangeDeadbandM") ? 0 : range_error;
    if (selected == "relative_acro" || selected == "relative_manual") {
        if (sample_dt > 0)
            range_integral =
                clamp(range_integral + n("rangeIntegralGain") * controlled_error * sample_dt,
                      -std::min(n("rangeIntegralLimitMps"), n("maxReverseSpeedMps")),
                      std::min(n("rangeIntegralLimitMps"), speed_limit));
    } else
        range_integral = 0;
    const double feed = selected == "guided_velocity" || acro ? target_radial : range_integral;
    const bool relative_tracking = acro && nav && feed_source == "confirmed_motion";
    double correction = n("rangeGain") * controlled_error + n("rangeRateGain") * range_rate,
           approach_limit = missing;
    if (relative_tracking && c.value("closeApproachEnabled", true)) {
        approach_limit = hold || recovery
                             ? 0
                             : n("closeApproachMaxSpeedMps") * progress +
                                   n("rangeGain") * std::max(0.0, d - n("safetyResumeDistanceM"));
        correction = std::min(correction, approach_limit);
    }
    double forward = clamp(feed + correction, -n("maxReverseSpeedMps"), speed_limit);
    const double previous_speed =
        std::abs(throttle_command) / n("relativeThrottleLimitPercent") * speed_limit;
    const double lookahead =
        clamp(n("lookaheadBaseM") +
                  n("lookaheadSpeedGainS") * std::max(previous_speed, std::abs(forward)),
              n("lookaheadMinM"), n("lookaheadMaxM"));
    if (sample_dt > 0) {
        const bool changed =
            cross_integral != 0 && body.y != 0 && sign(body.y) != sign(cross_integral);
        cross_integral *= std::exp(-(n("ilosIntegralDecay") + (changed ? 2 : 0)) * sample_dt);
        cross_integral += body.y * sample_dt;
        const double limit =
            n("ilosIntegralGain") > 0
                ? lookahead * std::tan(n("ilosMaxCorrectionDeg") * radians) / n("ilosIntegralGain")
                : 0;
        cross_integral = clamp(cross_integral, -limit, limit);
    }
    const double ilos =
        clamp(std::atan2(n("ilosIntegralGain") * cross_integral, lookahead) / radians,
              -n("ilosMaxCorrectionDeg"), n("ilosMaxCorrectionDeg"));
    const double guidance_bearing = wrap(control_bearing + ilos),
                 abs_guidance = std::abs(guidance_bearing),
                 deadbanded =
                     abs_guidance <= n("bearingDeadbandDeg")
                         ? 0
                         : sign(guidance_bearing) * (abs_guidance - n("bearingDeadbandDeg"));
    const double curvature = clamp(2 * std::sin(deadbanded * radians) / lookahead,
                                   -n("maxCurvatureMInv"), n("maxCurvatureMInv"));
    const double curvature_scale = std::max(
        n("minimumCurveSpeedScale"), 1 / (1 + n("curvatureSpeedGain") * std::abs(curvature)));
    double los_rate = 0;
    if (acro && nav && motion_trusted) {
        const double psi = yaw * radians, dx = number(motion_velocity, "x") - vx,
                     dy = number(motion_velocity, "y") - vy;
        const double vf = dx * std::cos(psi) + dy * std::sin(psi),
                     vr = -dx * std::sin(psi) + dy * std::cos(psi);
        los_rate = clamp((body.x * vr - body.y * vf) / std::max(.09, d * d) / radians,
                         -n("maxYawRateDegS"), n("maxYawRateDegS"));
    }
    const double fov_slow = n("fieldOfViewHalfDeg") - 25, fov_stop = n("fieldOfViewHalfDeg") - 10;
    const double outward =
        std::max(0.0, sign(bearing != 0 ? bearing : los_rate) * (los_rate - yaw_rate));
    const double fov_projected =
                     std::max(std::abs(bearing), std::abs(control_bearing)) + .5 * outward,
                 fov_scale =
                     acro ? clamp((fov_stop - fov_projected) / (fov_stop - fov_slow), 0, 1) : 1;
    const bool fov_priority = acro && std::abs(bearing) >= fov_slow;
    auto guidance =
        traditional_guidance(c, acro ? bearing : guidance_bearing, yaw_rate - los_rate, forward);
    if (acro)
        guidance["desiredYawRateDegS"] = clamp(number(guidance, "desiredYawRateDegS") + los_rate,
                                               -n("maxYawRateDegS"), n("maxYawRateDegS"));
    const double turn_scale = c["controllerProfile"] == "traditional_fast"
                                  ? number(guidance, "speedScale")
                                  : curvature_scale;
    const double alignment =
        acro ? std::min(fov_scale, clamp((n("bearingStopDeg") - abs_guidance) /
                                             (n("bearingStopDeg") - n("bearingSlowdownDeg")),
                                         0, 1))
             : 1;
    forward *= turn_scale;
    forward *= alignment;
    if (close) {
        range_integral = 0;
        const double speed = hold ? 0
                             : recovery
                                 ? -std::min(n("maxReverseSpeedMps"), n("closeApproachMinSpeedMps"))
                                 : close_speed * std::max(.15, turn_scale);
        if (relative_tracking && !recovery)
            forward = std::min(forward, (feed + approach_limit) * turn_scale * alignment);
        else
            forward = acro ? std::min(forward, speed) : speed;
    }
    forward = clamp(forward, -n("maxReverseSpeedMps"), speed_limit);
    double braking_limit = speed_limit, braking_distance = 0, braking_clearance = missing;
    bool braking = false;
    if (acro && nav) {
        const auto &p = c["prediction"];
        const double deceleration = std::max(.05, number(p, "braking_deceleration_mps2", .3));
        const double age = std::max(0.0, (now - measurement_time(t.value("uwb", Json::object()), now)) /
                                             1000),
                     reaction = .6 + std::min(age, n("uwbTimeoutMs") / 1000);
        const double margin = std::max(.05, number(p, "measurement_noise_m", .12)),
                     target_speed = recovery
                                        ? 0
                                        : clamp(std::min(target_radial, boat_toward + range_rate),
                                                0, speed_limit),
                     closing = std::max(0.0, boat_toward - target_speed);
        braking_clearance = std::max(0.0, d - n("followingDistanceM") - margin);
        braking_distance = closing * reaction + squared(closing) / (2 * deceleration);
        const double closing_limit =
            std::sqrt(squared(deceleration * reaction) + 2 * deceleration * braking_clearance) -
            deceleration * reaction;
        braking_limit = std::min(speed_limit, target_speed + std::max(0.0, closing_limit));
        braking = forward > braking_limit || braking_distance >= braking_clearance;
        if (braking)
            range_integral = std::min(0.0, range_integral);
        forward = std::min(forward, braking_limit);
    }
    const double proportional =
        clamp(forward / speed_limit * n("relativeThrottleLimitPercent"),
              -n("relativeThrottleLimitPercent"), n("relativeThrottleLimitPercent"));
    const bool apply_minimum =
        !acro && range_error >= n("minimumThrottleRangeErrorM") && proportional > .01;
    const double desired_throttle =
        acro            ? proportional
        : close         ? hold       ? 0
                          : recovery ? -n("closeApproachMinThrottlePercent")
                                     : close_throttle
        : apply_minimum ? sign(proportional) *
                              std::max(std::abs(proportional), n("minimumEffectiveThrottlePercent"))
                        : proportional;
    const double throttle_dt =
        std::isfinite(throttle_at) ? clamp((now - throttle_at) / 1000, 0, .25) : .1;
    throttle_at = now;
    const bool slowing = sign(desired_throttle) != sign(throttle_command) ||
                         std::abs(desired_throttle) < std::abs(throttle_command);
    const double throttle_rate = slowing ? n("throttleDecelPercentS") : n("throttleAccelPercentS"),
                 throttle_target = throttle_command != 0 && desired_throttle != 0 &&
                                           sign(desired_throttle) != sign(throttle_command)
                                       ? 0
                                       : desired_throttle;
    throttle_command = toward(throttle_command, throttle_target, throttle_rate * throttle_dt);
    if (acro) {
        throttle_command = std::min(throttle_command, braking_limit / speed_limit *
                                                          n("relativeThrottleLimitPercent"));
        throttle_command = clamp(throttle_command, -fov_scale * n("relativeThrottleLimitPercent"),
                                 fov_scale * n("relativeThrottleLimitPercent"));
    }
    if (std::abs(throttle_command) < .01)
        throttle_command = 0;
    const double commanded_speed =
        throttle_command / n("relativeThrottleLimitPercent") * speed_limit;
    const double desired_yaw = c["controllerProfile"] == "traditional_fast"
                                   ? number(guidance, "desiredYawRateDegS")
                                   : clamp(commanded_speed * curvature / radians,
                                           -n("maxYawRateDegS"), n("maxYawRateDegS"));
    const double same_limit = acro ? n("relativeSteeringLimitDeg")
                                   : std::abs(throttle_command) * n("sameDirectionSteeringRatio"),
                 unconstrained = desired_yaw / n("maxYawRateDegS") * n("relativeSteeringLimitDeg");
    const bool same_limited = std::abs(unconstrained) > same_limit + 1e-9;
    double desired_steering =
        clamp(unconstrained, -std::min(n("relativeSteeringLimitDeg"), same_limit),
              std::min(n("relativeSteeringLimitDeg"), same_limit));
    const int desired_sign = sign(desired_steering);
    if (acro) {
        reverse_sign = 0;
        reverse_at = 0;
    } else if (desired_sign != 0 && last_steering_sign != 0 && desired_sign != last_steering_sign) {
        if (abs_guidance < n("bearingReverseConfirmDeg")) {
            desired_steering = 0;
            reverse_sign = 0;
            reverse_at = 0;
        } else {
            if (reverse_sign != desired_sign) {
                reverse_sign = desired_sign;
                reverse_at = now;
            }
            if (now - reverse_at < n("bearingReverseConfirmMs") ||
                std::abs(steering_command) >= .05)
                desired_steering = 0;
            else {
                last_steering_sign = desired_sign;
                reverse_sign = 0;
                reverse_at = 0;
            }
        }
    } else if (desired_sign == last_steering_sign) {
        reverse_sign = 0;
        reverse_at = 0;
    }
    const double steering_dt =
        std::isfinite(steering_at) ? clamp((now - steering_at) / 1000, 0, .25) : .1;
    steering_at = now;
    const double steering_rate = std::abs(desired_steering) < std::abs(steering_command)
                                     ? n("steeringReturnDegS")
                                     : n("steeringSlewDegS");
    steering_command =
        acro ? desired_steering
             : toward(steering_command, desired_steering, steering_rate * steering_dt);
    if (std::abs(steering_command) < .01)
        steering_command = 0;
    if (steering_command != 0 && desired_steering != 0)
        last_steering_sign = sign(steering_command);
    const double final_limit = acro ? n("relativeSteeringLimitDeg")
                                    : std::abs(throttle_command) * n("sameDirectionSteeringRatio");
    steering_command =
        clamp(steering_command, -std::min(n("relativeSteeringLimitDeg"), final_limit),
              std::min(n("relativeSteeringLimitDeg"), final_limit));
    const double commanded_yaw =
        steering_command / n("relativeSteeringLimitDeg") * n("maxYawRateDegS");
    out.update({{"reason", "tracking"},
                {"fieldOfViewHalfDeg", n("fieldOfViewHalfDeg")},
                {"fovProjectedBearingDeg", fov_projected},
                {"fovSpeedScale", fov_scale},
                {"fovYawPriority", fov_priority},
                {"lineOfSightRateDegS", los_rate},
                {"controlBearingDeg", control_bearing},
                {"guidanceBearingDeg", guidance_bearing},
                {"deadbandedBearingDeg", deadbanded},
                {"crossTrackErrorM", body.y},
                {"crossTrackIntegralMs", cross_integral},
                {"ilosCorrectionDeg", ilos},
                {"lookaheadM", lookahead},
                {"curvatureMInv", curvature},
                {"rangeRateMps", range_rate},
                {"bearingRateDegS", bearing_rate},
                {"targetRadialSpeedMps", target_radial},
                {"speedFeedForwardSource", feed_source},
                {"relativeSpeedTracking", relative_tracking},
                {"relativeSpeedCorrectionMps", correction},
                {"relativeApproachLimitMps", nullable(approach_limit)},
                {"rangeIntegralMps", range_integral},
                {"boatTowardTargetMps", boat_toward},
                {"controllerProfile", c["controllerProfile"]},
                {"measuredYawRateDegS", yaw_rate},
                {"traditionalHeadingGain", n("traditionalHeadingGain")},
                {"traditionalYawRateDamping", n("traditionalYawRateDamping")},
                {"traditionalRawYawRateDegS", guidance["rawYawRateDegS"]},
                {"traditionalTurnSpeedScale", guidance["speedScale"]},
                {"turnSlowdownScale", turn_scale},
                {"curvatureSpeedScale", curvature_scale},
                {"turnAlignmentScale", alignment},
                {"sameDirectionSteeringLimitDeg", same_limit},
                {"sameDirectionLimited", same_limited},
                {"closeApproachActive", close},
                {"closeApproachProgress", progress},
                {"closeApproachSpeedMps", close_speed},
                {"closeApproachThrottlePercent", close_throttle},
                {"distanceHoldErrorM", hold_error},
                {"distanceHoldActive", hold},
                {"distanceRecoveryActive", recovery},
                {"brakingActive", braking},
                {"brakingSpeedLimitMps", braking_limit},
                {"brakingDistanceM", braking_distance},
                {"brakingClearanceM", nullable(braking_clearance)},
                {"trackingPhase", close ? hold       ? "distance_hold"
                                          : recovery ? "distance_recovery"
                                                     : "close_approach"
                                  : c["controllerProfile"] == "traditional_fast" &&
                                          abs_guidance > n("bearingDeadbandDeg")
                                      ? "traditional_fast_turn"
                                  : std::abs(curvature) > .05 ? "arc_turn"
                                                              : "approaching"},
                {"forwardSpeedMps", commanded_speed},
                {"desiredForwardSpeedMps", forward},
                {"proportionalThrottlePercent", proportional},
                {"minimumEffectiveThrottleApplied", apply_minimum},
                {"commandedForwardSpeedMps", commanded_speed},
                {"desiredThrottlePercent", desired_throttle},
                {"desiredYawRateDegS", desired_yaw},
                {"yawRateDegS", commanded_yaw},
                {"yawRateRadS", commanded_yaw * radians},
                {"throttlePercent", throttle_command},
                {"steeringDegrees", steering_command},
                {"velocityNed",
                 {{"x", commanded_speed * std::cos(yaw * radians)},
                  {"y", commanded_speed * std::sin(yaw * radians)}}},
                {"desiredSpeedMps", std::abs(forward)}});
    return out;
}
} // namespace manta
