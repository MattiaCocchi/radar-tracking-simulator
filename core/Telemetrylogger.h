#pragma once

#include <fstream>
#include <string>
#include <array>
#include <cstddef>
#include <stdexcept>

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  TelemetryLogger — High-frequency CSV logger                            ║
// ║                                                                          ║
// ║  Design goals:                                                           ║
// ║   • No dynamic allocation in write() — uses a fixed ring buffer         ║
// ║   • Bulk file I/O on flush() to avoid blocking the RT loop              ║
// ║   • Ready for mmap / UDP socket replacement on embedded targets         ║
// ╚══════════════════════════════════════════════════════════════════════════╝

struct TelemetryRow {
    double t_ms;           ///< Simulation timestamp          [ms]
    double true_omega;     ///< Ground truth wheel speed       [rad/s]
    double meas_omega;     ///< Noisy ABS measurement          [rad/s]
    double ekf_omega;      ///< EKF estimated wheel speed      [rad/s]
    double true_mu;        ///< Ground truth friction          [−]
    double ekf_mu;         ///< EKF estimated friction         [−]
    double ekf_mu_std;     ///< EKF std-dev on μ estimate      [−]
    double innovation;     ///< EKF innovation (residual)      [rad/s]
    double mahalanobis;    ///< Mahalanobis distance           [χ²]
    double tau;            ///< Applied torque                 [N·m]
};

// ─────────────────────────────────────────────────────────────────────────────
template<std::size_t BufferSize = 1024>
class TelemetryLogger {
public:
    explicit TelemetryLogger(const std::string& filename)
        : file_(filename), write_idx_(0), total_rows_(0)
    {
        if (!file_.is_open())
            throw std::runtime_error("Cannot open telemetry file: " + filename);

        // CSV header — matches TelemetryRow fields
        file_ << "t_ms,"
                 "true_omega_rad_s,meas_omega_rad_s,ekf_omega_rad_s,"
                 "true_mu,ekf_mu,ekf_mu_std,"
                 "innovation_rad_s,mahalanobis,"
                 "tau_Nm\n";
        file_.flush();
    }

    // ── Write one row (no I/O — copies into ring buffer) ─────────────────────
    // Safe to call from the real-time loop at 100 Hz.
    void write(const TelemetryRow& row) noexcept {
        buffer_[write_idx_] = row;
        write_idx_ = (write_idx_ + 1) % BufferSize;
        ++total_rows_;

        // Auto-flush when buffer is 75% full
        if (write_idx_ == BufferSize * 3 / 4)
            flush();
    }

    // ── Flush buffered rows to disk ───────────────────────────────────────────
    // Call at end of simulation or from a low-priority thread.
    void flush() {
        // Flush from the oldest unwritten entry
        const std::size_t start = (total_rows_ > BufferSize)
                                  ? write_idx_
                                  : 0;
        const std::size_t count = std::min(total_rows_, BufferSize);

        for (std::size_t i = 0; i < count; ++i) {
            const TelemetryRow& r = buffer_[(start + i) % BufferSize];
            file_ << r.t_ms         << ','
                  << r.true_omega   << ','
                  << r.meas_omega   << ','
                  << r.ekf_omega    << ','
                  << r.true_mu      << ','
                  << r.ekf_mu       << ','
                  << r.ekf_mu_std   << ','
                  << r.innovation   << ','
                  << r.mahalanobis  << ','
                  << r.tau          << '\n';
        }
        file_.flush();
        // Reset — rows already written
        write_idx_  = 0;
        total_rows_ = 0;
    }

    ~TelemetryLogger() { flush(); }

private:
    std::ofstream                      file_;
    std::array<TelemetryRow, BufferSize> buffer_;
    std::size_t                        write_idx_;
    std::size_t                        total_rows_;
};