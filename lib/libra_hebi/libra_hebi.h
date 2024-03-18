/******************************************************************************
 * @file   libra_hebi.h
 * @brief  Control code for LIBRA arm HEBI actuators; header file.
 *         (adapted from Yuto Goto's work)
 *
 * @author Christian Brice
 * @date   2024/2/22
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   HEBI Actuators
#include <group_command.hpp>
#include <group_feedback.hpp>
#include <hebi.h>
#include <lookup.hpp>
#include <trajectory.hpp>
// Project Headers
//   (none)

/**
 * @brief TODO.
 */
class LibraHebi {
  public:
    LibraHebi();

    /**
     * @brief LIBRA joint names.
     * @todo implement `kYaw`.
     */
    enum Joint { kRoll = 0, kPitch, kJ1, kJ2, kJ3 };

    // --- Actuator Commands ---

    bool Connect();
    void Move(double roll, double pitch, double j1, double j2, double j3);
    void Stop();

    // --- Getters & Setters ---

    double GetCommandPosition(Joint joint);
    double GetFeedbackPosition(Joint joint);
    double GetFeedbackEffort(Joint joint);
    double GetFeedbackEffortMA();
    double GetFeedbackEffortMB();

    void SetDebugMode(bool enabled);

  private:
    /**
     * @brief HEBI actuator names.
     * @todo implement `kYaw`.
     */
    enum Act { kHebiMA = 0, kHebiMB, kHebiJ1, kHebiJ2, kHebiJ3 };

    // --- Helper Functions ---

    static std::chrono::system_clock::rep GetCurrentTimeInSec();
    
    void Loop();

    // --- Data Members ---

    bool debug_mode_{false};

    std::unique_ptr<hebi::GroupCommand> command_;
    std::unique_ptr<hebi::GroupFeedback> feedback_;
    std::shared_ptr<hebi::Group> group_{nullptr};
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_{nullptr};
    
    double start_time_;
};
