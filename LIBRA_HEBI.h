#pragma once

#include <group_command.hpp>
#include <group_feedback.hpp>
#include <hebi.h>
#include <lookup.hpp>
#include <trajectory.hpp>
#include <windows.h>

class LIBRA_HEBI {
  public:
    LIBRA_HEBI();
    int connect();
    void move(double roll, double pitch, double j1, double j2, double j3);
    void stop();

    enum {
        ROLL,
        PITCH,
        J1,
        J2,
        J3,
    };

    double getCommandPosition(int joint);
    double getFeedbackPosition(int joint);
    double getFeedbackEffort(int joint);
    double getFeedbackEffortMA();
    double getFeedbackEffortMB();

  private:
    static void CALLBACK callback(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1,
                                  DWORD dw2);
    void loop();

    hebi::GroupCommand* command;
    hebi::GroupFeedback* feedback;
    std::shared_ptr<hebi::Group> group = nullptr;
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory = nullptr;
    DWORD start_time = timeGetTime();

    enum {
        HEBI_MA,
        HEBI_MB,
        HEBI_J1,
        HEBI_J2,
        HEBI_J3
    };
};
