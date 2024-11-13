/******************************************************************************
 * @file   arm_thread.h
 * @brief  Control code for LIBRA-II arm actuators; header file.
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
#include <QThread>             // Qt::Core
#include <trajectory.hpp>      // HEBI

// Project Headers
//   (none)

/**
 * @brief TODO: documentation.
 */
class ArmThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    /**
     * @brief LIBRA joint names.
     */
    enum Joint { kYaw = 0, kPitch };

    ArmThread();
    ~ArmThread() = default;

    // --- Actuator Commands ---

    bool Connect();
    void Move(double pitch);
    void Stop();

    // --- Getters & Setters ---

    double GetCommandPosition(Joint joint);
    double GetFeedbackPosition(Joint joint);
    double GetFeedbackEffort(Joint joint);

    void SetDebugMode(bool enabled);

  signals:
    void InformState(std::array<double, 2> target, std::array<double, 2> actual,
                     std::array<double, 2> torque);

  private:
    const uint8_t kYawID = 0;    // TODO: EPOS (Maxon)
    const uint8_t kPitchID = 0;  // HEBI

    void run() override;

    // --- Helper Functions ---

    static std::chrono::system_clock::rep GetCurrentTimeInSec();

    // --- Data Members ---

    bool debug_mode_{false};

    // EPOS (Maxon)

    // TODO

    // HEBI

    std::unique_ptr<hebi::GroupCommand> command_;
    std::unique_ptr<hebi::GroupFeedback> feedback_;
    std::shared_ptr<hebi::Group> group_{nullptr};
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_{nullptr};

    double start_time_;
};
