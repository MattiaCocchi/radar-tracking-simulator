#ifndef KALMANFILTER1D_H
#define KALMANFILTER1D_H

/**
 * KalmanFilter1D  –  Constant-Velocity (CV) model for one axis.
 *
 * State vector:   x = [pos, vel]^T   (2×1)
 * Transition:     F = [[1, dt],       (position integrates velocity)
 *                       [0,  1 ]]
 * Measurement:    H = [1, 0]          (we only measure position)
 * Process noise:  Q = diag(q_pos, q_vel)
 * Meas. noise:    R = sigma_meas²
 *
 * Why this beats a plain α-filter
 * ─────────────────────────────────
 * An α-filter is a 0th-order estimator: it has no internal velocity state.
 * When the target moves at constant velocity the filter always lags by
 *   lag ≈ (1-α)/α · v · T
 * The CV-Kalman carries velocity in its state, so after a brief
 * convergence period it predicts the next position and eliminates the lag.
 */
class KalmanFilter1D {
public:
    /**
     * @param sigma_meas  Standard deviation of the radar measurement [m]
     * @param q_pos       Process-noise std-dev for position  (model uncertainty)
     * @param q_vel       Process-noise std-dev for velocity  (manoeuvre noise)
     */
    KalmanFilter1D(double sigma_meas, double q_pos = 0.5, double q_vel = 0.1);

    /** Initialise with a known position and (optionally) velocity. */
    void init(double pos, double vel = 0.0);

    /** Prediction step – call once per time-step before update(). */
    void predict(double dt);

    /** Correction step – call with the new radar measurement. */
    void update(double measurement);

    double getPos() const { return x_[0]; }
    double getVel() const { return x_[1]; }

    bool isInitialized() const { return initialized_; }

private:
    double x_[2];      // state:       [pos, vel]
    double P_[2][2];   // covariance:  2×2 symmetric positive-definite matrix
    double R_;         // measurement variance  (σ²)
    double Qp_;        // process noise – position
    double Qv_;        // process noise – velocity
    bool   initialized_;
};

#endif // KALMANFILTER1D_H