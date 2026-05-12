#ifndef KALMANFILTER1D_H
#define KALMANFILTER1D_H



class KalmanFilter1D {
public:
    /**
     * @param sigma_meas  deviazione standard delle misurazinoi radar [m]
     * @param q_pos       precessing del rumeore per posizione  (model uncertainty)
     * @param q_vel       processing del rumre per velocit  (manoeuvre noise)
     */
    KalmanFilter1D(double sigma_meas, double q_pos = 0.5, double q_vel = 0.1);

    void init(double pos, double vel = 0.0);

    void predict(double dt);

    void update(double measurement);

    double getPos() const { return x_[0]; }
    double getVel() const { return x_[1]; }

    bool isInitialized() const { return initialized_; }

private:
    double x_[2];      
    double P_[2][2];   
    double R_;         
    double Qp_;        
    double Qv_;        
    bool   initialized_;
};

#endif 