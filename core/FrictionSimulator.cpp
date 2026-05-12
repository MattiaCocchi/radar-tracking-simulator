#include "FrictionSimulator.h"
#include <algorithm>
#include <stdexcept>

FrictionSimulator::FrictionSimulator(const WheelModel& model,
                                     double sigma_sensor,
                                     double initial_omega,
                                     double initial_mu)
    : model_      (model)
    , true_omega_ (initial_omega)
    , true_mu_    (initial_mu)
    , sigma_      (sigma_sensor)
    , rng_        (std::random_device{}())
    , noise_dist_ (0.0, sigma_sensor)
{
    if (sigma_sensor < 0.0)
        throw std::invalid_argument("Sensor noise must be non-negative");
}

void FrictionSimulator::addSurfaceEvent(double time_s,
                                        double new_mu,
                                        const std::string& label)
{
    events_.push_back({time_s, new_mu, label});
    // Keep sorted by time so we can process in order
    std::sort(events_.begin(), events_.end(),
              [](const SurfaceEvent& a, const SurfaceEvent& b)
              { return a.time_s < b.time_s; });
    next_event_idx_ = 0; // reset (call addSurfaceEvent before step())
}

double FrictionSimulator::step(double tau, double Vx, double dt, double t_now)
{
    // ── 1. Apply any surface event whose time has arrived ─────────────────────
    while (next_event_idx_ < events_.size() &&
           t_now >= events_[next_event_idx_].time_s)
    {
        true_mu_ = events_[next_event_idx_].new_mu;
        ++next_event_idx_;
        // Note: EKF does NOT know about this event — it must infer μ from data
    }

    // ── 2. Integrate true wheel dynamics with RK4 ─────────────────────────────
    // RK4 gives better accuracy than Euler at the same dt,
    // making the simulated "reality" more faithful to the physics.
    true_omega_ = model_.integrateOmega(true_omega_, true_mu_, tau, Vx, dt);
    true_omega_ = std::max(true_omega_, 0.0);

    // ── 3. Synthetic ABS sensor: add Gaussian noise ───────────────────────────
    // ABS sensors (magnetoresistive ring encoders) have typical noise
    // σ ≈ 0.1–0.5 rad/s depending on wheel speed and quantisation.
    return true_omega_ + noise_dist_(rng_);
}