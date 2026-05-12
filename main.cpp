#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <string>

#include "core/WheelModel.h"
#include "core/ExtendedKalmanFilter.h"
#include "core/FrictionSimulator.h"
#include "core/TelemetryLogger.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Enhanced Simulation Config
// ─────────────────────────────────────────────────────────────────────────────
namespace Config {
    // Wheel physical parameters
    constexpr double I_WHEEL   = 1.8;     // [kg·m²]
    constexpr double R_EFF     = 0.315;   // [m]
    constexpr double FZ        = 5000.0;  // Normal load [N]

    // Vehicle dynamics
    constexpr double VX        = 25.0;    // Constant speed for this test [m/s]
    constexpr double TAU_BRAKE = -450.0;  // Slightly harder braking to trigger lock-up
    
    // Thermal & Rolling Resistance
    constexpr double T_AMBIENT = 25.0;    // Initial temp [°C]
    constexpr double T_PEAK    = 95.0;    // Temp after heavy braking [°C]

    // EKF Tuning
    constexpr double Q_OMEGA   = 0.5;
    constexpr double Q_MU      = 0.05;    // Smoother tracking
    constexpr double SIGMA_ABS = 0.2;     // Sensor noise [rad/s]

    // Timing
    constexpr double DT_S      = 0.01;    // 100 Hz
    constexpr double SIM_TOTAL = 10.0;    
    constexpr int    STEPS     = static_cast<int>(SIM_TOTAL / DT_S);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Dynamic Temperature Profile
// ─────────────────────────────────────────────────────────────────────────────
static double getTireTemperature(double t_s, double tau) noexcept {
    if (tau < -100.0) {
        return Config::T_AMBIENT + (Config::T_PEAK - Config::T_AMBIENT)
               * (1.0 - std::exp(-0.5 * t_s));
    }
    return Config::T_AMBIENT;
}

static double torqueProfile(double t_s) noexcept {
    if (t_s < 0.5)  return 0.0;
    if (t_s < 9.5)  return Config::TAU_BRAKE;
    return 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main Execution Loop
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    using namespace Config;
    using Clock = std::chrono::steady_clock;

    // 1. Setup Models
    const WheelParams params = Surface::dryAsphalt(I_WHEEL, R_EFF, FZ);
    WheelModel model(params);

    // 2. Plant (The "Real" World)
    FrictionSimulator plant(model, SIGMA_ABS, VX / R_EFF, 1.0);
    plant.addSurfaceEvent(3.0, 0.1, "BLACK ICE");     // Sudden drop
    plant.addSurfaceEvent(6.0, 0.6, "WET ASPHALT");   // Partial recovery

    // 3. The Estimator (Virtual Sensor)
    ExtendedKalmanFilter ekf(model, Q_OMEGA, Q_MU, SIGMA_ABS);
    ekf.init(VX / R_EFF, 0.8); // Start with a guess of 0.8 mu

    // 4. Logger
    TelemetryLogger<4096> logger("friction_v2_results.csv");

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n>>> Starting High-Fidelity EKF Simulation (v2.0)\n";
    std::cout << ">>> Surfaces: DRY (3s) -> ICE (3s) -> WET (4s)\n\n";

    std::cout << std::setw(8)  << "Time[s]"
              << std::setw(10) << "Mode"
              << std::setw(10) << "Confid."
              << std::setw(10) << "Mu_True"
              << std::setw(10) << "Mu_EKF"
              << std::setw(10) << "Temp[C]" << "\n"
              << std::string(60, '-') << "\n";

    auto next_tick = Clock::now();
    const auto dt_chrono = std::chrono::milliseconds(10);

    for (int step = 0; step < STEPS; ++step) {
        const double t_s = step * DT_S;
        const double tau = torqueProfile(t_s);
        const double T_c = getTireTemperature(t_s, tau);

        // --- STEP 1: Physics ---
        double omega_meas = plant.step(tau, VX, DT_S, t_s);

        // --- STEP 2: Estimator ---
        ekf.predict(tau, VX, DT_S, T_c);
        ekf.update(omega_meas, VX, T_c);

        // --- STEP 3: Logging & Diagnostics ---
        std::string mode_str = (ekf.getMode() == EkfMode::FreezeMu) ? "[FREEZE]" : "NORMAL";
        
        if (step % 50 == 0) {
            std::cout << std::setw(8)  << t_s 
                      << std::setw(10) << mode_str
                      << std::setw(10) << ekf.getConfidenceScore()
                      << std::setw(10) << plant.trueMu()
                      << std::setw(10) << ekf.getMu()
                      << std::setw(10) << T_c << "\n";
        }

        // ── All 13 TelemetryRow fields, each in its correct slot ──────────────
        logger.write({
            t_s * 1000.0,                              // t_ms
            plant.trueOmega(),                         // true_omega
            omega_meas,                                // meas_omega
            ekf.getOmega(),                            // ekf_omega
            plant.trueMu(),                            // true_mu
            ekf.getMu(),                               // ekf_mu
            ekf.getMuStdDev(),                         // ekf_mu_std  ← real σ_μ
            ekf.getLastInnovation(),                   // innovation  ← real residual
            ekf.getLastMahalanobis(),                  // mahalanobis ← real χ²
            tau,                                       // tau         ← torque [N·m]
            ekf.getConfidenceScore(),                  // confidence  [0..1]
            static_cast<double>(ekf.getMode()),        // mode        [0/1/2]
            T_c                                        // temp_c      [°C]
        });

        // Real-time pacing
        next_tick += dt_chrono;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "\n>>> Simulation Complete. Data saved to friction_v2_results.csv\n";
    return 0;
}