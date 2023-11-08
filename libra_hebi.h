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
     * @brief TODO.
     */
    enum {
        ROLL,
        PITCH,
        J1,
        J2,
        J3
    };

    // --- Motor Commands ---

    int connect();
    void move(double roll, double pitch, double j1, double j2, double j3);
    void stop();

    // --- Getters & Setters ---

    double getCommandPosition(int joint);
    double getFeedbackPosition(int joint);
    double getFeedbackEffort(int joint);
    double getFeedbackEffortMA();
    double getFeedbackEffortMB();

  private:
    /**
     * @brief TODO.
     */
    enum {
        HEBI_MA,
        HEBI_MB,
        HEBI_J1,
        HEBI_J2,
        HEBI_J3
    };

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
