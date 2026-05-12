#pragma once

#include "WheelModel.h"
#include <cmath>
#include <optional>
/**
 * ExtendedKalmanFilter - State Estimator v2.0
 * * State vector:  x = [omega, mu]
 * - omega : wheel angular velocity [rad/s] (from ABS)
 * - mu    : tire-road friction coefficient [-] (Virtual Sensor)
 * * Process model:
 * omega+ = omega + dt * (torque - (Fx + F_rr) * R) / I
 * mu+    = mu (random-walk)
 * * Safety Features:
 * - Lock-up freeze: Inhibits mu update when observability is low.
 * - Confidence score: [0..1] metric based on P trace.
 * - Thermal correction: Uses tire temperature for Pacejka Jacobians.
 * * Performance: Zero heap allocation (stack-only matrices).
 */
// ── EKF operating mode reported in diagnostics ───────────────────────────────
enum class EkfMode : uint8_t {
    Normal     = 0,   ///< Both ω and μ are being updated
    FreezeMu   = 1,   ///< μ update frozen (low observability / lock-up)
    Coasting   = 2,   ///< No torque, minimal slip — ω tracked, μ held
};

// ─────────────────────────────────────────────────────────────────────────────
class ExtendedKalmanFilter {
public:
    // ── Freeze policy thresholds (tunable) ───────────────────────────────────
    struct FreezePolicy {
        double omega_min_rad_s  = 1.0;   ///< |ω| below this → freeze (near lock-up)
        double kappa_sat        = 0.80;  ///< |κ| above this → fully saturated, freeze
        double observability_lo = 0.05;  ///< observabilityIndex < this → freeze
    };

    // ── Construction ──────────────────────────────────────────────────────────
    // @param model    Reference to wheel model (shared with simulator)
    // @param q_omega  Process noise σ for ω  [rad/s]
    // @param q_mu     Process noise σ for μ  (how fast μ can change) [−]
    // @param r_omega  Measurement noise σ    [rad/s] (ABS spec)
    ExtendedKalmanFilter(const WheelModel& model,
                         double q_omega,
                         double q_mu,
                         double r_omega);

    // ── Initialise state and covariance ──────────────────────────────────────
    void init(double omega_init, double mu_init,
              double P_omega_init = 10.0, double P_mu_init = 1.0);

    // ── EKF Predict ──────────────────────────────────────────────────────────
    // @param tau        Applied torque   [N·m]
    // @param Vx         Vehicle speed    [m/s]
    // @param dt         Time step        [s]
    // @param T_celsius  Tyre temperature [°C] (optional, defaults to 85°C)
    void predict(double tau, double Vx, double dt, double T_celsius = 85.0);

    // ── EKF Update ───────────────────────────────────────────────────────────
    // @param omega_meas  ABS wheel-speed reading [rad/s]
    // @param Vx          Current vehicle speed   [m/s]  (needed for freeze check)
    // @param T_celsius   Tyre temperature        [°C]
    void update(double omega_meas, double Vx, double T_celsius = 85.0);

    // ── Freeze policy accessor / mutator ──────────────────────────────────────
    [[nodiscard]] const FreezePolicy& freezePolicy() const noexcept { return freeze_; }
    void setFreezePolicy(const FreezePolicy& p) noexcept { freeze_ = p; }

    // ── State accessors ───────────────────────────────────────────────────────
    [[nodiscard]] double getOmega()       const noexcept { return x_[0]; }
    [[nodiscard]] double getMu()          const noexcept { return x_[1]; }
    [[nodiscard]] double getMuStdDev()    const noexcept { return std::sqrt(P_[1][1]); }
    [[nodiscard]] double getOmegaStdDev() const noexcept { return std::sqrt(P_[0][0]); }

    // ── Confidence score [0, 1] ───────────────────────────────────────────────
    // Combines:
    //   a) Covariance health: 1 − tanh(trace(P) / P_SCALE) ∈ (0,1]
    //      → 1.0 when P is small (filter converged), → 0 when P blows up
    //   b) Observability at current slip: observabilityIndex ∈ [0,1]
    //      → high only when κ is in the informative region
    //
    // Final score = a · b  (both must be good for high confidence)
    //
    // Interpretation:
    //   > 0.7 : High confidence  — use μ estimate for ABS/ESC decisions
    //   0.3–0.7: Medium          — flag for secondary confirmation
    //   < 0.3 : Low              — μ estimate unreliable, use safe defaults
    [[nodiscard]] double getConfidenceScore() const noexcept;

    // ── Diagnostics ───────────────────────────────────────────────────────────
    [[nodiscard]] double  getLastInnovation()    const noexcept { return last_innovation_; }
    [[nodiscard]] double  getLastMahalanobis()   const noexcept { return last_mahala_; }
    [[nodiscard]] double  getObservability()     const noexcept { return last_observability_; }
    [[nodiscard]] EkfMode getMode()              const noexcept { return mode_; }
    [[nodiscard]] bool    isMuFrozen()           const noexcept { return mode_ == EkfMode::FreezeMu; }

private:
    const WheelModel& model_;

    double   x_[2];       ///< State:            [ω, μ]
    double   P_[2][2];    ///< Error covariance:  2×2 symmetric PD
    double   Q_[2][2];    ///< Process noise:     2×2 diagonal
    double   R_;          ///< Measurement noise variance (scalar)

    FreezePolicy freeze_{};

    // Diagnostics
    double  last_innovation_   = 0.0;
    double  last_mahala_       = 0.0;
    double  last_observability_= 0.0;
    EkfMode mode_              = EkfMode::Normal;

    // Confidence score normalisation constant
    // Tuned so that P_trace = 1.0 → ~76% covariance penalty
    static constexpr double P_SCALE_ = 100.0;

    // Physical bounds
    static constexpr double MU_MIN_ = 0.02;
    static constexpr double MU_MAX_ = 1.50;
    static constexpr double OM_MIN_ = 0.0;

    // ── 2×2 matrix utilities (zero-allocation) ────────────────────────────────
    static void mat_add   (const double A[2][2], const double B[2][2], double C[2][2]) noexcept;
    static void mat_mul   (const double A[2][2], const double B[2][2], double C[2][2]) noexcept;
    static void mat_trans (const double A[2][2], double AT[2][2])                      noexcept;
};