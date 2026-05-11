#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>
#include <cmath>

#include "core/Entity.h"
#include "core/Radar.h"
#include "core/Tracker.h"
#include "core/Logger.h"

// ─── Helpers ──────────────────────────────────────────────────────────────────

static double rmse(double err) { return std::abs(err); }

/**
 * SimulationRunner<E>
 * ────────────────────
 * Template wrapper: can hold any entity-like type that exposes
 *   update(double dt) and getState() → State.
 *
 * Enables tracking multiple heterogeneous entities without code duplication.
 */
template<typename EntityType>
class SimulationRunner {
public:
    SimulationRunner(std::unique_ptr<EntityType> entity,
                     std::unique_ptr<Radar>      radar,
                     std::unique_ptr<Tracker>    tracker,
                     std::unique_ptr<CSVLogger>  logger,
                     const std::string&          name)
        : entity_ (std::move(entity))
        , radar_  (std::move(radar))
        , tracker_(std::move(tracker))
        , logger_ (std::move(logger))
        , name_   (name)
    {}

    void run(int steps, double dt) {
        const int W = 8;
        std::cout << "\n=== " << name_ << " ===\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << std::setw(4)  << "t"
                  << std::setw(W)  << "Real X"
                  << std::setw(W)  << "Radar X"
                  << std::setw(W)  << "α-filt"
                  << std::setw(W)  << "Kalman"
                  << std::setw(W)  << "KalVx"
                  << std::setw(9)  << "|err_α|"
                  << std::setw(9)  << "|err_K|"
                  << '\n';
        std::cout << std::string(4 + W*5 + 18, '-') << '\n';

        double sum_err_alpha = 0, sum_err_kalman = 0;

        for (int t = 1; t <= steps; ++t) {
            entity_->update(dt);
            State real = entity_->getState();

            auto [mx, my] = radar_->detect(real.x, real.y);
            tracker_->ProcessData(mx, my, dt);

            State kEst  = tracker_->GetEstimatedState();
            State aEst  = tracker_->GetAlphaEstimatedState();

            double ea = rmse(real.x - aEst.x);
            double ek = rmse(real.x - kEst.x);
            sum_err_alpha  += ea;
            sum_err_kalman += ek;

            logger_->write(t * dt,
                           real.x,  real.y,
                           mx,      my,
                           kEst.x,  kEst.y, kEst.vx, kEst.vy,
                           aEst.x,  aEst.y);

            std::cout << std::setw(3)  << t << 's'
                      << std::setw(W)  << real.x
                      << std::setw(W)  << mx
                      << std::setw(W)  << aEst.x
                      << std::setw(W)  << kEst.x
                      << std::setw(W)  << kEst.vx
                      << std::setw(9)  << ea
                      << std::setw(9)  << ek
                      << '\n';
        }

        std::cout << std::string(4 + W*5 + 18, '-') << '\n';
        std::cout << "Mean |err| X →  α-filter: "
                  << std::setw(7) << sum_err_alpha  / steps
                  << "m    Kalman: "
                  << std::setw(7) << sum_err_kalman / steps << "m\n";
    }

private:
    std::unique_ptr<EntityType> entity_;
    std::unique_ptr<Radar>      radar_;
    std::unique_ptr<Tracker>    tracker_;
    std::unique_ptr<CSVLogger>  logger_;
    std::string                 name_;
};

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    constexpr double SIGMA_RADAR = 3.0;   // radar noise [m]
    constexpr double DT          = 1.0;   // time step  [s]
    constexpr int    STEPS       = 30;

    // ── Entity 1: slow aircraft (vx=10, vy=2) ───────────────────────────────
    SimulationRunner<Entity> sim1(
        std::make_unique<Entity>(0.0, 0.0, 10.0, 2.0),
        std::make_unique<Radar>(SIGMA_RADAR),
        std::make_unique<Tracker>(SIGMA_RADAR, 0.5, 0.1),
        std::make_unique<CSVLogger>("aircraft_slow.csv"),
        "Aircraft A  (vx=10 m/s, vy=2 m/s)"
    );
    sim1.run(STEPS, DT);

    // ── Entity 2: fast aircraft (vx=50, vy=10) ──────────────────────────────
    SimulationRunner<Entity> sim2(
        std::make_unique<Entity>(0.0, 0.0, 50.0, 10.0),
        std::make_unique<Radar>(SIGMA_RADAR),
        std::make_unique<Tracker>(SIGMA_RADAR, 2.0, 0.5),
        std::make_unique<CSVLogger>("aircraft_fast.csv"),
        "Aircraft B  (vx=50 m/s, vy=10 m/s)"
    );
    sim2.run(STEPS, DT);

    std::cout << "\nCSV logs written: aircraft_slow.csv, aircraft_fast.csv\n";
    std::cout << "(Tip: load in Python/matplotlib or Qt Charts for visual comparison)\n";
    return 0;
}