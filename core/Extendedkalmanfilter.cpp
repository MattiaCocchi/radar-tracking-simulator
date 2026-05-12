#include "ExtendedKalmanFilter.h"
#include <cstring>   // memset, memcpy
#include <algorithm> // std::clamp

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

ExtendedKalmanFilter::ExtendedKalmanFilter(const WheelModel& model,
                                           double q_omega,
                                           double q_mu,
                                           double r_omega)
    : model_(model), R_(r_omega * r_omega)
{
    // State initialised in init()
    x_[0] = x_[1] = 0.0;
    last_observability_ = 1.0;
    // ── Process noise Q (diagonal) ────────────────────────────────────────────
    // Q models uncertainty in the plant model itself:
    //   Q[0][0] = σ²_ω : unmodelled wheel dynamics (e.g., tyre flexibility)
    //   Q[1][1] = σ²_μ : how quickly friction can change (random-walk rate)
    //              ↑ Increasing this lets the filter track faster μ jumps.
    memset(Q_, 0, sizeof(Q_));
    Q_[0][0] = q_omega * q_omega;
    Q_[1][1] = q_mu    * q_mu;

    memset(P_, 0, sizeof(P_));
}

// ─────────────────────────────────────────────────────────────────────────────
//  init
// ─────────────────────────────────────────────────────────────────────────────

void ExtendedKalmanFilter::init(double omega_init, double mu_init,
                                double P_omega_init, double P_mu_init)
{
    x_[0] = omega_init;
    x_[1] = mu_init;
    last_observability_ = 1.0;
    // High initial covariance → the filter trusts early measurements quickly
    memset(P_, 0, sizeof(P_));
    P_[0][0] = P_omega_init;
    P_[1][1] = P_mu_init;
}

// ─────────────────────────────────────────────────────────────────────────────
//  EKF Predict step
//  ─────────────────────────────────────────────────────────────────────────────
//
//  1.  Propagate state via nonlinear model f(x):
//        ω⁺ = ω + dt·(τ − Fx(κ,μ)·R_eff) / I
//        μ⁺ = μ                      (friction is a slowly-varying parameter)
//
//  2.  Compute Jacobian F_jac = ∂f/∂x |_{x_k}:
//
//        F_jac = | ∂ω⁺/∂ω    ∂ω⁺/∂μ |
//                | ∂μ⁺/∂ω    ∂μ⁺/∂μ |
//
//        ∂ω⁺/∂ω = 1 − dt/I · R_eff · (∂Fx/∂ω)
//        ∂ω⁺/∂μ =   − dt/I · R_eff · (∂Fx/∂μ)
//        ∂μ⁺/∂ω = 0
//        ∂μ⁺/∂μ = 1
//
//  3.  Propagate covariance:  P⁺ = F·P·Fᵀ + Q

void ExtendedKalmanFilter::predict(double tau, double Vx, double dt, double T_celsius)
{
    const double omega = x_[0];
    const double mu    = x_[1];
    const double kappa = model_.slipRatio(omega, Vx);
    const double Fx    = model_.pacejkaFx(kappa, mu);
    const auto&  p     = model_.params();

    // ── 1. Nonlinear state propagation (Euler; plant uses RK4 separately) ────
    x_[0] = omega + dt * (tau - Fx * p.R_eff) / p.I_wheel;
    x_[1] = mu;  // random-walk: no drift

    // Apply physical bounds
    x_[0] = std::max(x_[0], OM_MIN_);
    x_[1] = std::clamp(x_[1], MU_MIN_, MU_MAX_);

    // ── 2. Build Jacobian F_jac ───────────────────────────────────────────────
    const double dFx_dw = model_.dFx_dOmega(kappa, mu, Vx);
    const double dFx_dm = model_.dFx_dMu(kappa);

    const double scale = -dt * p.R_eff / p.I_wheel;

    double F_jac[2][2];
    F_jac[0][0] = 1.0 + scale * dFx_dw;   // ∂ω⁺/∂ω
    F_jac[0][1] =       scale * dFx_dm;   // ∂ω⁺/∂μ
    F_jac[1][0] = 0.0;                     // ∂μ⁺/∂ω
    F_jac[1][1] = 1.0;                     // ∂μ⁺/∂μ

    // ── 3. Covariance propagation:  P = F·P·Fᵀ + Q ───────────────────────────
    double FP[2][2], FPFt[2][2], Ft[2][2];
    mat_mul  (F_jac, P_,  FP);
    mat_trans(F_jac, Ft);
    mat_mul  (FP,    Ft,  FPFt);
    mat_add  (FPFt,  Q_,  P_);
    last_observability_ = model_.observabilityIndex(kappa, x_[1], T_celsius);
}

// ─────────────────────────────────────────────────────────────────────────────
//  EKF Update step
//  ─────────────────────────────────────────────────────────────────────────────
//
//  Measurement model:  z = H·x,   H = [1, 0]  (ABS reads ω directly)
//
//  y = z − H·x_pred                         (innovation)
//  S = H·P·Hᵀ + R                           (innovation covariance, scalar)
//  K = P·Hᵀ / S                             (Kalman gain, 2×1)
//
//  x_upd = x_pred + K·y
//
//  Joseph form (numerically more stable than standard form):
//  P_upd = (I − K·H) · P · (I − K·H)ᵀ + K·R·Kᵀ

void ExtendedKalmanFilter::update(double omega_meas, double Vx, double T_celsius)
{
    // H = [1, 0]  →  H·x = x[0],  H·P·Hᵀ = P[0][0]

    // ── Innovation ────────────────────────────────────────────────────────────
    const double y = omega_meas - x_[0];
    last_innovation_ = y;

    // ── Innovation covariance (scalar) ────────────────────────────────────────
    const double S = P_[0][0] + R_;
    last_mahala_    = (y * y) / S;  // Mahalanobis distance (chi² statistic)
    const double kappa = model_.slipRatio(x_[0], Vx);
    const double slip_abs = std::abs(kappa);

    // Trigger per il Freeze (osservabilità bassa)
    if (x_[0] < freeze_.omega_min_rad_s || slip_abs > freeze_.kappa_sat) {
        mode_ = EkfMode::FreezeMu;
    } else {
        mode_ = EkfMode::Normal;
    }
    // ── Kalman gain  K = P·Hᵀ/S  (2×1 vector since H is 1×2) ─────────────────
    const double K0 = P_[0][0] / S;
    const double K1 = P_[1][0] / S;

    // ── State update ──────────────────────────────────────────────────────────
    x_[0] += K0 * y;
    x_[1] += K1 * y;

    // Physical bounds
    x_[0] = std::max(x_[0], OM_MIN_);
    x_[1] = std::clamp(x_[1], MU_MIN_, MU_MAX_);

    // ── Covariance update (Joseph form) ───────────────────────────────────────
    // IKH = I − K·H:
    //   IKH = | 1-K0   0 |
    //         | -K1    1 |
    double IKH[2][2];
    IKH[0][0] = 1.0 - K0;  IKH[0][1] = 0.0;
    IKH[1][0] = -K1;        IKH[1][1] = 1.0;

    double IKH_t[2][2], IKH_P[2][2], IKH_P_IKHt[2][2];
    mat_trans(IKH,   IKH_t);
    mat_mul  (IKH,   P_,     IKH_P);
    mat_mul  (IKH_P, IKH_t,  IKH_P_IKHt);

    // K·R·Kᵀ (scalar R, so just R·K·Kᵀ):
    double KRKt[2][2];
    KRKt[0][0] = R_ * K0 * K0;  KRKt[0][1] = R_ * K0 * K1;
    KRKt[1][0] = R_ * K1 * K0;  KRKt[1][1] = R_ * K1 * K1;

    mat_add(IKH_P_IKHt, KRKt, P_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Zero-allocation 2×2 matrix utilities
// ─────────────────────────────────────────────────────────────────────────────

void ExtendedKalmanFilter::mat_add(
        const double A[2][2], const double B[2][2], double C[2][2]) noexcept
{
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            C[i][j] = A[i][j] + B[i][j];
}

void ExtendedKalmanFilter::mat_mul(
        const double A[2][2], const double B[2][2], double C[2][2]) noexcept
{
    // C must not alias A or B
    double tmp[2][2];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            tmp[i][j] = 0.0;
            for (int k = 0; k < 2; ++k)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(C, tmp, sizeof(tmp));
}

void ExtendedKalmanFilter::mat_trans(
        const double A[2][2], double AT[2][2]) noexcept
{
    AT[0][0] = A[0][0];  AT[0][1] = A[1][0];
    AT[1][0] = A[0][1];  AT[1][1] = A[1][1];
}

double ExtendedKalmanFilter::getConfidenceScore() const noexcept {
    // Calcoliamo la "salute" della matrice di covarianza
    // Più i valori sulla diagonale di P sono piccoli, più il filtro è certo.
    const double trace = P_[0][0] + P_[1][1];
    
    // Usiamo una funzione che mappa il valore in un range [0, 1]
    // 1.0 = massima confidenza, 0.0 = incertezza totale
    const double cov_health = 1.0 - std::tanh(trace / P_SCALE_);
    
    // Moltiplichiamo per l'osservabilità (se non "vediamo" la fisica, la confidenza cala)
    return std::clamp(cov_health * last_observability_, 0.0, 1.0);
}