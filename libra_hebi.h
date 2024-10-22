/******************************************************************************
 * @file   libra_hebi.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
#include <windows.h>
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
class LIBRA_HEBI {
  public:
    LIBRA_HEBI();

    /**
     * @brief LIBRA joint names.
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

    bool is_open_ = false;

  private:
    /**
     * @brief HEBI actuator names.
     */
    enum Act { kHebiMA = 0, kHebiMB, kHebiJ1, kHebiJ2, kHebiJ3 };

    static void CALLBACK Callback(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1,
                                  DWORD dw2);
    void Loop();

    // --- Data Members ---

    std::unique_ptr<hebi::GroupCommand> command_;
    std::unique_ptr<hebi::GroupFeedback> feedback_;
    std::shared_ptr<hebi::Group> group_{nullptr};
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_{nullptr};
    DWORD start_time_ = timeGetTime();
};
