#include "Radar.h"

Radar::Radar(double sigma) : 
        m_gen(std::random_device{}()),
        n_dist(0.0, sigma){}

std::pair<double, double> Radar::detect(double posX, double posY){
    double getErrX = n_dist(m_gen);
    double getErrY = n_dist(m_gen);

    double misureX = posX + getErrX;
    double misureY = posY + getErrY;

    return {misureX, misureY};
}        