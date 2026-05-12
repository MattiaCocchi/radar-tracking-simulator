#include "Tracker.h"

Tracker::Tracker(double sigma_meas, double q_pos, double q_vel, double alpha)
    : kf_x_(std::make_unique<KalmanFilter1D>(sigma_meas, q_pos, q_vel)),
      kf_y_(std::make_unique<KalmanFilter1D>(sigma_meas, q_pos, q_vel)),
      alpha_state_{0, 0, 0, 0},
      alpha_(alpha),
      firstData_(true)
{}

void Tracker::ProcessData(double x, double y, double dt) {
    // Kalman
    if (kf_x_->isInitialized()) {
        kf_x_->predict(dt);
        kf_y_->predict(dt);
    }
    kf_x_->update(x);
    kf_y_->update(y);

    // α-filter 
    if (firstData_) {
        alpha_state_.x = x;
        alpha_state_.y = y;
        firstData_ = false;
    } else {
        alpha_state_.x = (1.0 - alpha_) * alpha_state_.x + alpha_ * x;
        alpha_state_.y = (1.0 - alpha_) * alpha_state_.y + alpha_ * y;
    }
}

//Kalman gett

State Tracker::GetEstimatedState() const {
    return State{ kf_x_->getPos(), kf_y_->getPos(),
                  kf_x_->getVel(), kf_y_->getVel() };
}

double Tracker::GetEstimatedVx() const { return kf_x_->getVel(); }
double Tracker::GetEstimatedVy() const { return kf_y_->getVel(); }

// ── α-filter get

State Tracker::GetAlphaEstimatedState() const {
    return alpha_state_;
}