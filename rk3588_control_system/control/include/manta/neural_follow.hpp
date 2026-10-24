#pragma once
#include "follow_control.hpp"

namespace manta {
class JsonActor {
  public:
    explicit JsonActor(const std::string &path);
    std::vector<double> infer(const std::vector<double> &observation) const;
    std::string model_path;

  private:
    struct Layer {
        std::vector<std::vector<double>> weight;
        std::vector<double> bias;
        bool elu;
    };
    std::vector<double> mean, stddev;
    double epsilon;
    std::vector<Layer> layers;
};
class NeuralFollow : public FollowController {
  public:
    explicit NeuralFollow(const Json &raw);
    void reset() override;
    Json preflight(double now, const Json &telemetry, bool connected,
                   const std::string &mode = "auto", const Json &estimate = nullptr) const override;
    Json update(double now, const Json &telemetry, bool connected, const std::string &mode = "auto",
                const Json &estimate = nullptr, const Json &motion = nullptr) override;
    JsonActor actor;

  private:
    double maximum, stop_distance, resume_distance, velocity_alpha;
    bool initialized = false, safety_latched = false;
    std::array<double, 3> filter{0, 0, 0}, uncertainty{1, 1, 1};
    double measurement_at = missing, previous_speed = 0, previous_heading = missing,
           neural_throttle = 0, neural_steering = 0;
    Vec last_relative{}, target_velocity{};
    void ingest(const Json &telemetry, const Json &preflight);
    std::vector<double> observation(double now, const Json &telemetry, const Json &preflight,
                                    const Json &estimate);
    Json relative_only(double now, const Json &telemetry, const Json &preflight,
                       const Json &estimate);
};
} // namespace manta
