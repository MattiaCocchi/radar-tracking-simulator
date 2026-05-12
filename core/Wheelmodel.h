#pragma once

#include <cmath>
#include <algorithm>
/**
 * WheelModel - Physical Dynamics & Force Modeling v2.0
 * * * Moment Balance (Newton-Euler):
 * I * d(omega)/dt = torque - (Fx(kappa, mu, T) + F_rr(omega)) * R_eff
 * * * Longitudinal Force (Pacejka Magic Formula):
 * Fx = mu * Fz * sin( C * arctan( B(T) * kappa ) )
 * where B(T) = B0 * (1 + alpha * (T - T_ref)) [Thermal correction]
 * * * Rolling Resistance:
 * F_rr = Crr * Fz * tanh(omega/epsilon) [Continuous approximation]
 * * * Observability:
 * Provides the Jacobian-based mu-observability index for EKF freeze logic.
 * * Reference: Pacejka H.B., "Tyre and Vehicle Dynamics", 3rd Ed.
 */
struct WheelParams {
    double I_wheel;       ///< Moment of inertia              [kg·m²]
    double R_eff;         ///< Effective rolling radius        [m]
    double Fz;            ///< Normal load on this wheel       [N]
    double B;             ///< Pacejka B — stiffness factor   (dry≈10, wet≈7, ice≈3)
    double C;             ///< Pacejka C — shape factor       (longitudinal ≈ 1.9)

    // ── Rolling resistance ────────────────────────────────────────────────────
    // Typical passenger tyre: Crr ≈ 0.010–0.015
    // Torque loss = Crr · Fz · R_eff  (≈ 10–17 N·m per wheel)
    double Crr = 0.012;

    // ── Thermal B model ───────────────────────────────────────────────────────
    // B(T) = B₀ · (1 + alpha_T · (T − T_ref))
    // Cold tyre: B < B₀ → shallower grip curve, lower peak
    // Hot tyre:  B = B₀ → nominal (operating window ≈ 80–100°C for road tyres)
    double T_ref   = 85.0;    ///< Optimal temperature [°C]
    double alpha_T = 0.003;   ///< Thermal sensitivity [1/°C]
};

namespace Surface {
    inline WheelParams dryAsphalt(double I, double R, double Fz)
        { return {I, R, Fz, 10.0, 1.9}; }
    inline WheelParams wetAsphalt(double I, double R, double Fz)
        { return {I, R, Fz,  7.0, 1.9}; }
    inline WheelParams ice(double I, double R, double Fz)
        { return {I, R, Fz,  3.0, 1.9}; }
}

// ─────────────────────────────────────────────────────────────────────────────
class WheelModel {
public:
    explicit WheelModel(const WheelParams& params) : p_(params) {}

    // ── Longitudinal slip ratio ───────────────────────────────────────────────
    [[nodiscard]]
    double slipRatio(double omega, double Vx) const noexcept {
        const double Vx_s = std::max(Vx, VX_MIN_);
        return (omega * p_.R_eff - Vx_s) / Vx_s;
    }

    // ── Thermal B factor ─────────────────────────────────────────────────────
    [[nodiscard]]
    double thermalB(double T_celsius) const noexcept {
        return std::max(p_.B * (1.0 + p_.alpha_T * (T_celsius - p_.T_ref)),
                        B_COLD_FLOOR_);
    }

    // ── Pacejka Fx (with optional temperature) ────────────────────────────────
    [[nodiscard]]
    double pacejkaFx(double kappa, double mu,
                     double T_celsius = 85.0) const noexcept {
        const double B_eff = thermalB(T_celsius);
        return mu * p_.Fz * std::sin(p_.C * std::atan(B_eff * kappa));
    }

    // ── Rolling resistance ────────────────────────────────────────────────────
    // tanh(ω/ε) gives smooth C∞ sign approximation — avoids Jacobian
    // discontinuity at ω=0 that would cause chattering in the EKF.
    [[nodiscard]]
    double rollingResistance(double omega) const noexcept {
        constexpr double EPS = 0.5; // smoothing width [rad/s]
        return p_.Crr * p_.Fz * std::tanh(omega / EPS);
    }

    // ── Angular acceleration (full model) ─────────────────────────────────────
    [[nodiscard]]
    double omegaDot(double omega, double mu, double tau, double Vx,
                    double T_celsius = 85.0) const noexcept {
        const double kappa = slipRatio(omega, Vx);
        const double Fx    = pacejkaFx(kappa, mu, T_celsius);
        const double F_rr  = rollingResistance(omega);
        return (tau - (Fx + F_rr) * p_.R_eff) / p_.I_wheel;
    }

    // ── RK4 integration ───────────────────────────────────────────────────────
    // μ and T are constant within each dt (valid for dt << τ_μ, τ_T).
    [[nodiscard]]
    double integrateOmega(double omega, double mu, double tau, double Vx,
                          double dt, double T_celsius = 85.0) const noexcept {
        const auto f = [&](double w) {
            return omegaDot(w, mu, tau, Vx, T_celsius);
        };
        const double k1 = f(omega);
        const double k2 = f(omega + 0.5 * dt * k1);
        const double k3 = f(omega + 0.5 * dt * k2);
        const double k4 = f(omega + dt * k3);
        return omega + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }

    // ── Analytical Jacobians ──────────────────────────────────────────────────

    // ∂Fx/∂ω:  chain rule  (∂Fx/∂κ) · (∂κ/∂ω)
    [[nodiscard]]
    double dFx_dOmega(double kappa, double mu, double Vx,
                      double T_celsius = 85.0) const noexcept {
        const double B_eff  = thermalB(T_celsius);
        const double Bk     = B_eff * kappa;
        const double dFx_dk = mu * p_.Fz
                             * std::cos(p_.C * std::atan(Bk))
                             * p_.C * B_eff / (1.0 + Bk * Bk);
        const double dk_dw  = p_.R_eff / std::max(Vx, VX_MIN_);
        return dFx_dk * dk_dw;
    }

    // ∂Fx/∂μ:  linear in μ
    [[nodiscard]]
    double dFx_dMu(double kappa, double T_celsius = 85.0) const noexcept {
        const double B_eff = thermalB(T_celsius);
        return p_.Fz * std::sin(p_.C * std::atan(B_eff * kappa));
    }

    // ── Observability index of μ at current operating point ───────────────────
    //
    // Derived from the information contribution of μ to the measurement ω:
    //   I_μ ∝ |∂ω̇/∂μ| = |∂Fx/∂μ| · R_eff / I_wheel
    //
    // Normalised to [0, 1]:
    //   0 = μ completely unobservable (free rolling or full lock-up)
    //   1 = maximum sensitivity (near-peak slip)
    //
    // This drives the EKF freeze decision:
    //   observability < threshold  →  freeze μ update, hold P[1][1]
    [[nodiscard]]
    double observabilityIndex(double kappa, double mu,
                              double T_celsius = 85.0) const noexcept {
        const double dFx_dm   = std::abs(dFx_dMu(kappa, T_celsius));
        // Normalise by theoretical maximum: μ·Fz (when sin(·)=1)
        const double norm     = mu * p_.Fz + 1e-6;
        return std::clamp(dFx_dm / norm, 0.0, 1.0);
    }

    [[nodiscard]] const WheelParams& params() const noexcept { return p_; }

private:
    WheelParams p_;
    static constexpr double VX_MIN_       = 0.5;
    static constexpr double B_COLD_FLOOR_ = 1.0;
};