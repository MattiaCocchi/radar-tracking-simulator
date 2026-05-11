#ifndef RADAR_H
#define RADAR_H

#include <random>
#include <utility>

class Radar {
public:
    explicit Radar(double sigma);

    // Returns a noisy (x, y) measurement
    std::pair<double, double> detect(double trueX, double trueY);

    double getSigma() const { return m_sigma; }

private:
    double                       m_sigma;
    std::mt19937                 m_gen;
    std::normal_distribution<double> n_dist;
};

#endif // RADAR_H