#pragma once
#include "prediction.hpp"

namespace manta {
class AutoResume {
  public:
    explicit AutoResume(Json config)
        : c(std::move(config)), p(c.value("prediction", Json::object())),
          enabled(c.value("auto_resume_after_uwb_timeout", false)) {
        cancel();
    }
    void clear_window();
    void cancel();
    void search();
    void started(const Json &telemetry, double now, bool automatic = false);
    void stopped(const std::string &reason);
    void attempt_failed(const std::string &reason);
    Json transition_failure(double now, const Json &telemetry, const Json &preflight) const;
    bool observe(double now, const Json &telemetry, const Json &preflight,
                 bool during_attempt = false);
    bool begin_attempt();
    Json ramp(Json result, double now);
    Json snapshot() const;
    bool requested = false, pending = false, attempting = false;

  private:
    Json c, p, target, window_target, last_stop_reason;
    bool enabled;
    int samples = 0;
    double first_at = missing, last_at = missing, resume_at = missing;
    Vec point{};
};
class RejectNmpc {
  public:
    explicit RejectNmpc(Json config) : c(std::move(config)) { reset(); }
    void reset();
    Json range_evidence(double now, const Json &t) const;
    void observe(double now, const Json &t, const Json &result);
    Json solve(double now, const Json &t, const Json &direction, const Json &near_raw = false);
    Json command, accepted;

  private:
    Json c, plan;
};
class DriftHold {
  public:
    explicit DriftHold(Json config) : c(std::move(config)), nmpc(c) { reset(); }
    void reset();
    void track(double now, const Json &t, const Json &result);
    Json observe_bearing(double now, const Json &t);
    void refresh_range(double now, const Json &t);
    Json accepted_prediction(double now, const Json &result, double forward, double speed) const;
    Json update(double now, const Json &t, const Json &result);

  private:
    Json c, anchor, last_tracking, certificate, target_id;
    std::vector<Json> bearings;
    double creep_speed = 0, creep_at = missing, last_progress = missing;
    bool far_pursuit = false;
    RejectNmpc nmpc;
};
} // namespace manta
