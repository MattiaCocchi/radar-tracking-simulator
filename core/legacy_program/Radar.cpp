#include "Radar.h"

Radar::Radar(double sigma)
    : m_sigma(sigma),
      m_gen(std::random_device{}()),
      n_dist(0.0, sigma) {}

std::pair<double, double> Radar::detect(double trueX, double trueY) {
    return { trueX + n_dist(m_gen),
             trueY + n_dist(m_gen) };
}