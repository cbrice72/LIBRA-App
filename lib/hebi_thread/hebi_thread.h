/******************************************************************************
 * @file   hebi_thread.h
 * @brief  Control code for LIBRA arm HEBI actuators; header file.
 *         (adapted from Yuto Goto's work)
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <group_command.hpp>   // HEBI
#include <group_feedback.hpp>  // HEBI
#include <hebi.h>              // HEBI
#include <lookup.hpp>          // HEBI
#include <QThread>             // Qt
#include <trajectory.hpp>      // HEBI

// Project Headers
//   (none)

/**
 * @brief TODO: documentation.
 */
class HebiThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    /**
     * @brief LIBRA joint names.
     */
    enum Joint { kRoll = 0, kPitch, kJ1, kJ2, kJ3 };

    HebiThread();
    ~HebiThread() = default;

    // --- Actuator Commands ---

    bool Connect();
    void Move(double roll, double pitch, double j1, double j2, double j3);
    void Stop();

    // --- Getters & Setters ---

    double GetCommandPosition(Joint joint);
    double GetFeedbackPosition(Joint joint);
    double GetFeedbackEffort(Joint joint);
    double GetFeedbackEffortMA();  // used by pumps
    double GetFeedbackEffortMB();  // used by pumps

    void SetDebugMode(bool enabled);

  signals:
    void InformState(std::array<double, 5> target, std::array<double, 5> actual,
                     std::array<double, 5> torque);

  private:
    /**
     * @brief HEBI actuator names.
     * @todo implement `kYaw`.
     */
    enum Act { kHebiMA = 0, kHebiMB, kHebiJ1, kHebiJ2, kHebiJ3 };

    void run() override;

    // --- Helper Functions ---

    static std::chrono::system_clock::rep GetCurrentTimeInSec();

    // --- Data Members ---

    bool debug_mode_{false};

    std::unique_ptr<hebi::GroupCommand> command_;
    std::unique_ptr<hebi::GroupFeedback> feedback_;
    std::shared_ptr<hebi::Group> group_{nullptr};
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_{nullptr};

    double start_time_;
};
