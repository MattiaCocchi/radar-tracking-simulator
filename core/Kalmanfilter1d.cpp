#include "KalmanFilter1D.h"

// ─── Constructor ─────────────────────────────────────────────────────────────

KalmanFilter1D::KalmanFilter1D(double sigma_meas, double q_pos, double q_vel)
    : R_(sigma_meas * sigma_meas),
      Qp_(q_pos * q_pos),
      Qv_(q_vel * q_vel),
      initialized_(false)
{
    x_[0] = x_[1] = 0.0;
    // High initial covariance → we trust the first measurement completely
    P_[0][0] = 500.0; P_[0][1] = 0.0;
    P_[1][0] = 0.0;   P_[1][1] = 500.0;
}

// ─── init ─────────────────────────────────────────────────────────────────────

void KalmanFilter1D::init(double pos, double vel) {
    x_[0] = pos;
    x_[1] = vel;
    P_[0][0] = R_;    // position uncertainty = measurement uncertainty
    P_[0][1] = 0.0;
    P_[1][0] = 0.0;
    P_[1][1] = 100.0; // velocity is unknown
    initialized_ = true;
}

// ─── Predict step ─────────────────────────────────────────────────────────────
//
//  F = [[1, dt], [0, 1]]
//
//  x_pred = F * x
//  P_pred = F * P * F^T + Q
//
//  Expanded (avoids heap allocation):
//    P_pred[0][0] = P[0][0] + dt*(P[1][0]+P[0][1]) + dt²*P[1][1] + Qp
//    P_pred[0][1] = P[0][1] + dt*P[1][1]
//    P_pred[1][0] = P[1][0] + dt*P[1][1]
//    P_pred[1][1] = P[1][1] + Qv

void KalmanFilter1D::predict(double dt) {
    // State prediction
    x_[0] += x_[1] * dt;
    // x_[1] unchanged (constant velocity)

    // Covariance prediction
    double p00 = P_[0][0] + dt*(P_[1][0] + P_[0][1]) + dt*dt*P_[1][1] + Qp_;
    double p01 = P_[0][1] + dt * P_[1][1];
    double p10 = P_[1][0] + dt * P_[1][1];
    double p11 = P_[1][1] + Qv_;

    P_[0][0] = p00;  P_[0][1] = p01;
    P_[1][0] = p10;  P_[1][1] = p11;
}

// ─── Update step ──────────────────────────────────────────────────────────────
//
//  H = [1, 0]   (measure only position)
//
//  S  = H * P * H^T + R  =  P[0][0] + R          (innovation covariance, scalar)
//  K  = P * H^T / S      =  [P[0][0], P[1][0]] / S
//  y  = z - H*x          =  meas - x[0]           (innovation)
//  x  = x + K*y
//  P  = (I - K*H) * P

void KalmanFilter1D::update(double measurement) {
    if (!initialized_) {
        init(measurement, 0.0);
        return;
    }

    double S  = P_[0][0] + R_;          // innovation covariance
    double K0 = P_[0][0] / S;           // Kalman gain – position row
    double K1 = P_[1][0] / S;           // Kalman gain – velocity row
    double y  = measurement - x_[0];    // innovation (residual)

    // State update
    x_[0] += K0 * y;
    x_[1] += K1 * y;

    // Covariance update  P = (I - K*H) * P
    double p00 = (1.0 - K0) * P_[0][0];
    double p01 = (1.0 - K0) * P_[0][1];
    double p10 = -K1 * P_[0][0] + P_[1][0];
    double p11 = -K1 * P_[0][1] + P_[1][1];

    P_[0][0] = p00;  P_[0][1] = p01;
    P_[1][0] = p10;  P_[1][1] = p11;
}