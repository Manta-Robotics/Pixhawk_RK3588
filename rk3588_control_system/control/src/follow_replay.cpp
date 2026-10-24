#include "manta/follow_control.hpp"
#include "manta/neural_follow.hpp"
#include "manta/prediction.hpp"
#include "manta/propulsion.hpp"
#include "manta/recovery.hpp"
#include <iostream>
#include <memory>

int main() {
    using namespace manta;
    Json config = normalize_follow_config(Json::object());
    auto controller = std::make_unique<FollowController>(Json::object());
    std::unique_ptr<NeuralFollow> neural;
    auto motor = std::make_unique<MotorLimiter>(Json::object());
    auto resume = std::make_unique<AutoResume>(Json::object());
    auto reject = std::make_unique<RejectNmpc>(Json::object());
    auto drift = std::make_unique<DriftHold>(Json::object());
    auto estimator = std::make_unique<TargetEstimator>(Json::object(), Json::object());
    auto intermittent = std::make_unique<IntermittentFollow>(Json::object(), Json::object());
    auto relative = std::make_unique<RelativeControl>(config);
    auto gps = std::make_unique<GpsSpeedLoop>(config);
    auto nmpc = std::make_unique<RelativeNmpc>(config);
    std::string line;
    while (std::getline(std::cin, line)) {
        try {
            const auto request = Json::parse(line);
            const auto op = request.value("op", std::string());
            Json out = nullptr;
            if (op == "reset") {
                config = normalize_follow_config(request.value("config", Json::object()));
                relative = std::make_unique<RelativeControl>(config);
                controller =
                    std::make_unique<FollowController>(request.value("config", Json::object()));
                if (request.value("neural", false))
                    neural = std::make_unique<NeuralFollow>(request.at("config"));
                const auto raw = request.value("config", Json::object());
                intermittent = std::make_unique<IntermittentFollow>(
                    raw.value("prediction", Json::object()), raw);
                gps = std::make_unique<GpsSpeedLoop>(config);
                nmpc = std::make_unique<RelativeNmpc>(config);
                out = config;
            } else if (op == "estimator_reset") {
                estimator =
                    std::make_unique<TargetEstimator>(request.value("config", Json::object()),
                                                      request.value("calibration", Json::object()));
            } else if (op == "motor_reset") {
                motor = std::make_unique<MotorLimiter>(request.at("config"),
                                                       request.value("traditional", false));
                out = motor->reset(request.value("now", 0.0));
            } else if (op == "motor_target")
                out = motor->set_target(request.at("target"));
            else if (op == "motor_step")
                out = motor->step(request.at("now"));
            else if (op == "motor_propulsion")
                out = motor->propulsion(request.at("now"), request.at("result"),
                                        request.at("telemetry"), request.at("connected"));
            else if (op == "resume_reset") {
                resume = std::make_unique<AutoResume>(request.at("config"));
                out = resume->snapshot();
            } else if (op == "resume_search") {
                resume->search();
                out = resume->snapshot();
            } else if (op == "resume_started") {
                resume->started(request.at("telemetry"), request.at("now"),
                                request.value("automatic", false));
                out = resume->snapshot();
            } else if (op == "resume_stopped") {
                resume->stopped(request.at("reason"));
                out = resume->snapshot();
            } else if (op == "resume_failed") {
                resume->attempt_failed(request.at("reason"));
                out = resume->snapshot();
            } else if (op == "resume_begin")
                out = resume->begin_attempt();
            else if (op == "resume_cancel") {
                resume->cancel();
                out = resume->snapshot();
            } else if (op == "resume_observe")
                out =
                    resume->observe(request.at("now"), request.at("telemetry"),
                                    request.at("preflight"), request.value("duringAttempt", false));
            else if (op == "resume_ramp")
                out = resume->ramp(request.at("result"), request.at("now"));
            else if (op == "reject_reset") {
                reject = std::make_unique<RejectNmpc>(request.at("config"));
            } else if (op == "reject_observe")
                reject->observe(request.at("now"), request.at("telemetry"), request.at("result"));
            else if (op == "reject_evidence")
                out = reject->range_evidence(request.at("now"), request.at("telemetry"));
            else if (op == "reject_solve")
                out = reject->solve(request.at("now"), request.at("telemetry"),
                                    request.at("direction"), request.value("nearRaw", Json(false)));
            else if (op == "drift_reset") {
                drift = std::make_unique<DriftHold>(request.at("config"));
            } else if (op == "drift_track")
                drift->track(request.at("now"), request.at("telemetry"), request.at("result"));
            else if (op == "drift_bearing")
                out = drift->observe_bearing(request.at("now"), request.at("telemetry"));
            else if (op == "drift_update")
                out =
                    drift->update(request.at("now"), request.at("telemetry"), request.at("result"));
            else if (op == "estimate")
                out = estimator->update(request.at("now"), request.at("telemetry"));
            else if (op == "intermittent")
                out = intermittent->update(
                    *controller, request.at("now"), request.at("telemetry"),
                    request.value("connected", false),
                    request.value("controlMode", std::string("relative_manual")));
            else if (op == "govern_acro")
                out = intermittent->govern_acro(
                    request.at("result"), request.at("speedLimitMps"), request.at("yawLimitDegS"),
                    request.at("steeringSpan"), request.at("throttleSpan"));
            else if (op == "neural")
                out = neural->update(request.at("now"), request.at("telemetry"),
                                     request.value("connected", false),
                                     request.value("controlMode", std::string("auto")),
                                     request.value("targetEstimate", Json(nullptr)),
                                     request.value("targetMotionEstimate", Json(nullptr)));
            else if (op == "controller")
                out = controller->update(request.at("now"), request.at("telemetry"),
                                         request.value("connected", false),
                                         request.value("controlMode", std::string("auto")),
                                         request.value("targetEstimate", Json(nullptr)),
                                         request.value("targetMotionEstimate", Json(nullptr)));
            else if (op == "preflight")
                out = controller->preflight(request.at("now"), request.at("telemetry"),
                                            request.value("connected", false),
                                            request.value("controlMode", std::string("auto")),
                                            request.value("targetEstimate", Json(nullptr)));
            else if (op == "relative")
                out = relative->update(request.at("now"), request.at("preflight"),
                                       request.at("telemetry"),
                                       request.value("estimate", Json(nullptr)),
                                       request.value("policy", Json(nullptr)));
            else if (op == "pause")
                relative->pause();
            else if (op == "applied")
                relative->track_applied(request.at("throttle"), request.at("steering"));
            else if (op == "nmpc")
                out = nmpc->solve(request.at("now"), request.at("telemetry"), request.at("range"),
                                  request.at("bearing"), request.value("estimate", Json(nullptr)),
                                  request.at("referenceThrottle"),
                                  request.value("gpsAvailable", false));
            else if (op == "nmpc_applied")
                nmpc->track_applied(request.at("throttle"), request.at("steering"));
            else if (op == "gps")
                out = gps->update(request.at("now"), request.at("telemetry"),
                                  request.at("bearingDeg"), request.at("rangeRateMps"),
                                  request.at("desiredClosingMps"), request.at("appliedThrottle"),
                                  number(request, "policySpeedMps"), request.value("stop", false));
            else if (op == "gps_applied")
                gps->track_applied(request.at("throttle"), request.at("dt"));
            else
                throw std::invalid_argument("unknown_follow_operation");
            std::cout << out.dump() << '\n';
        } catch (const std::exception &error) {
            std::cout << Json{{"error", error.what()}}.dump() << '\n';
        }
    }
}
