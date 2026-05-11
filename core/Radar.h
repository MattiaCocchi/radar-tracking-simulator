#ifndef RADAR_H
#define RADAR_H


#include <iostream>
#include <random>

class Radar{
    private:
    std::mt19937 m_gen;
    std::normal_distribution<double> n_dist;
    public:
    std::pair<double, double> detect(double trueX, double trueY);
    Radar(double sigma);
};

#endif