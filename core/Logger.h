#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>
#include <stdexcept>

/**
 * CSVLogger
 * ─────────
 * Writes one row per time-step to a CSV file so results can be loaded
 * by Python/matplotlib, Qt Charts, or any spreadsheet for analysis.
 *
 * Usage:
 *   CSVLogger log("simulation.csv");
 *   log.write(t, realX, realY, radarX, radarY,
 *              kalmanX, kalmanY, kalmanVx, kalmanVy,
 *              alphaX, alphaY);
 */
class CSVLogger {
public:
    explicit CSVLogger(const std::string& filename) : file_(filename) {
        if (!file_.is_open())
            throw std::runtime_error("Cannot open log file: " + filename);
        // Header
        file_ << "t,real_x,real_y,radar_x,radar_y,"
                 "kalman_x,kalman_y,kalman_vx,kalman_vy,"
                 "alpha_x,alpha_y\n";
    }

    void write(double t,
               double rx,  double ry,
               double mx,  double my,
               double kx,  double ky,  double kvx, double kvy,
               double ax,  double ay)
    {
        file_ << t    << ','
              << rx   << ',' << ry  << ','
              << mx   << ',' << my  << ','
              << kx   << ',' << ky  << ','
              << kvx  << ',' << kvy << ','
              << ax   << ',' << ay  << '\n';
    }

private:
    std::ofstream file_;
};

#endif // LOGGER_H