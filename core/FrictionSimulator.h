#pragma once

#include "WheelModel.h"
#include <random>
#include <vector>
#include <string>

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  FrictionSimulator — Ground Truth Plant & Sensor Emulator               ║
// ║                                                                          ║
// ║  Simulates the "real world" side of the V-model:                         ║
// ║    1. Integrates true wheel dynamics (RK4) with true μ(t)               ║
// ║    2. Adds Gaussian noise to ω → synthetic ABS sensor reading           ║
// ║    3. Manages surface-change scenarios (time-tagged μ transitions)       ║
// ║                                                                          ║
// ║  This class is NEVER seen by the EKF — it represents physical reality.  ║
// ╚══════════════════════════════════════════════════════════════════════════╝

// ── Scenario event: surface changes at a given simulation time ────────────────
struct SurfaceEvent {
    double time_s;   ///< Simulation time at which the surface changes [s]
    double new_mu;   ///< New friction coefficient value
    std::string label; ///< Human-readable label (e.g., "ICE", "DRY")
};

// ─────────────────────────────────────────────────────────────────────────────
class FrictionSimulator {
public:
    // @param model           Wheel model (shared with EKF for consistent params)
    // @param sigma_sensor    ABS sensor noise std-dev [rad/s]
    // @param initial_omega   Starting wheel speed [rad/s]
    // @param initial_mu      Starting friction coefficient
    FrictionSimulator(const WheelModel& model,
                      double sigma_sensor,
                      double initial_omega,
                      double initial_mu);

    // ── Scenario building ─────────────────────────────────────────────────────
    // Add a time-tagged surface transition (call before run())
    void addSurfaceEvent(double time_s, double new_mu, const std::string& label);

    // ── Step the simulation by dt seconds ────────────────────────────────────
    // @param tau   Applied torque  [N·m]   (positive=drive, negative=brake)
    // @param Vx    Vehicle speed   [m/s]   (assumed known, e.g. from GPS/IMU)
    // @param dt    Time step       [s]
    // @param t_now Current simulation time [s]
    //
    // Updates internal true state and returns noisy measurement.
    [[nodiscard]] double step(double tau, double Vx, double dt, double t_now);

    // ── Accessors ─────────────────────────────────────────────────────────────
    [[nodiscard]] double trueOmega() const noexcept { return true_omega_; }
    [[nodiscard]] double trueMu()    const noexcept { return true_mu_;    }

private:
    const WheelModel& model_;

    double true_omega_;
    double true_mu_;
    double sigma_;

    // Surface event queue
    std::vector<SurfaceEvent> events_;
    std::size_t               next_event_idx_ = 0;

    // Noise generator — seeded once, never re-seeded in the loop
    std::mt19937                 rng_;
    std::normal_distribution<double> noise_dist_;
};