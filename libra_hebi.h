/******************************************************************************
 * @file   libra_hebi.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
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

    int connect();
    void move(double roll, double pitch, double j1, double j2, double j3);
    void stop();

    // --- Getters & Setters ---

    double getCommandPosition(Joint joint);
    double getFeedbackPosition(Joint joint);
    double getFeedbackEffort(Joint joint);
    double getFeedbackEffortMA();
    double getFeedbackEffortMB();

  private:
    /**
     * @brief HEBI actuator names.
     */
    enum Act { kHebiMA = 0, kHebiMB, kHebiJ1, kHebiJ2, kHebiJ3 };

    static void CALLBACK callback(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1,
                                  DWORD dw2);
    void loop();

    // --- Data Members ---

    hebi::GroupCommand* command;
    hebi::GroupFeedback* feedback;
    std::shared_ptr<hebi::Group> group = nullptr;
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory = nullptr;
    DWORD start_time = timeGetTime();
};
