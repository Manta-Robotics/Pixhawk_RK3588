#pragma once
#include "mavlink_core.hpp"
#include "neural_follow.hpp"
#include "propulsion.hpp"
#include "recovery.hpp"
#include <memory>

namespace manta {
// One owner for operator intents, controller state and command leases. Web
// processes can come and go without running a second control loop.
class ControlRuntime {
  public:
    ControlRuntime(Json config, MavlinkCore &core, MavlinkCore::Event event);
    Json command(const Json &request, double wall, double mono);
    void tick(double wall, double mono);
    void shutdown(double wall, double mono);
    Json snapshot() const;

  private:
    Json config, raw, state, rover, t, source_guard, guarded_uwb;
    MavlinkCore &core;
    MavlinkCore::Event event;
    std::unique_ptr<FollowController> traditional;
    std::unique_ptr<NeuralFollow> neural;
    std::unique_ptr<IntermittentFollow> intermittent;
    std::unique_ptr<AutoResume> resume;
    std::unique_ptr<DriftHold> drift;
    std::unique_ptr<MotorLimiter> manual_motor, neural_motor, traditional_motor;
    std::string method = "traditional", control_mode = "auto", source = "WEB", phase, required_mode,
                neural_error;
    bool active = false, automatic = false;
    uint64_t transition = 0;
    double wall = 0, mono = 0, last_tick = missing, next_follow = 0, next_manual = 0,
           last_manual = -1e12, last_publish = -1e12, deadline = 0;
    double holding_at = 0, manual_confirmed = 0, mode_requested = 0, resume_started = 0,
           resume_requested = 0;
    double next_guard = 0;
    double throttle_limit = 100, steering_limit = 45, throttle_span = 100, steering_span = 45;
    FollowController &controller();
    Json send(const std::string &name, const Json &params = Json::object());
    void emit(const std::string &type, const Json &payload);
    void publish(Json result, bool running);
    Json idle(const std::string &reason, const std::string &status = "",
              const Json &preflight = Json::object()) const;
    Json normalized(double throttle, double steering) const;
    Json drive(double throttle, double steering, const std::string &label);
    Json relative_drive(Json &result);
    void neutral(const std::string &reason);
    void disarm(const std::string &reason);
    void stop(const std::string &reason, bool disarm_vehicle, bool cancel_intent = true);
    Json start(bool automatic_start = false);
    Json select(const std::string &requested);
    void advance_start();
    void update_follow();
    bool holding(Json &result);
    void reset_hold();
    bool neutral_feedback() const;
    void guard_uwb();
};
} // namespace manta
