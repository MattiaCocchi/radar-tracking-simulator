#ifndef TRACKER_H
#define TRACKER_H

#include "Entity.h"
#include "KalmanFilter1D.h"
#include <memory>


class Tracker {
public:
    /**
     * @param sigma_meas  Radar measurement noise std-dev [m]  (must match Radar)
     * @param q_pos       Model position uncertainty            (tune for manoeuvre)
     * @param q_vel       Model velocity uncertainty            (tune for manoeuvre)
     * @param alpha       α-filter coefficient (kept for comparison)
     */
    explicit Tracker(double sigma_meas = 3.0,
                     double q_pos      = 0.5,
                     double q_vel      = 0.1,
                     double alpha      = 0.2);

    /**
     * Feed a new radar measurement.
     * @param x   Measured X position [m]
     * @param y   Measured Y position [m]
     * @param dt  Time elapsed since last call [s]
     */
    void ProcessData(double x, double y, double dt = 1.0);

    // Kalman 
    State  GetEstimatedState()    const;   
    double GetEstimatedVx()       const;   
    double GetEstimatedVy()       const;  

    State  GetAlphaEstimatedState() const;

private:
    // Kalman filters (one per axis)
    std::unique_ptr<KalmanFilter1D> kf_x_;
    std::unique_ptr<KalmanFilter1D> kf_y_;

    // Legacy α-filter state
    State  alpha_state_;
    double alpha_;
    bool   firstData_;
};

#endif 