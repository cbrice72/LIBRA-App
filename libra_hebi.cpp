/******************************************************************************
 * @file   libra_hebi.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
 ******************************************************************************/

// Related Header
#include "libra_hebi.h"
// C++ Standard Library Headers
#include <stdexcept>
// POSIX/Windows Library Headers
#include "mmsystem.h"
#pragma comment(lib, "winmm.lib")
// Other Libraries' Headers
//   (none)
// Project Headers
#include "pretty_print.h"

/* --- TABLE OF CONTENTS ---
 * !General Functions
 * !Actuator Commands
 * !Getters & Setters
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new LIBRA_HEBI object.
 */
LIBRA_HEBI::LIBRA_HEBI() {
    command_ = std::make_unique<hebi::GroupCommand>(5);
    feedback_ = std::make_unique<hebi::GroupFeedback>(5);

    // タイマ割り込み開始 10ms毎に割り込み処理をする
    // Timer interrupt: process interrupts every 10 ms
    timeSetEvent(10, 0, Callback, reinterpret_cast<DWORD>(this),
                 TIME_PERIODIC | TIME_CALLBACK_FUNCTION);
}

/**
 * @brief TODO.
 *
 * @param uID TODO
 * @param uMsg TODO
 * @param dwUser TODO
 * @param dw1 TODO
 * @param dw2 TODO
 */
void CALLBACK LIBRA_HEBI::Callback(UINT /*uID*/, UINT /*uMsg*/, DWORD dwUser,
                                   DWORD /*dw1*/, DWORD /*dw2*/) {
    (reinterpret_cast<LIBRA_HEBI*>(dwUser))->Loop();
}

/**
 * @brief TODO.
 */
void LIBRA_HEBI::Loop() {
    command_->setVelocity(Eigen::VectorXd::Zero(5));

    if (trajectory_ != nullptr) {
        const double time = (timeGetTime() - start_time_) / 1000.0;
        if (trajectory_->getDuration() > time) {
            Eigen::VectorXd pos_cmd(5);
            Eigen::VectorXd vel_cmd(5);
            trajectory_->getState(time, &pos_cmd, &vel_cmd, nullptr);
            command_->setPosition(pos_cmd);
            command_->setVelocity(vel_cmd);
        }
    }

    if (group_ != nullptr) {
        group_->sendCommand(*command_);
        group_->getNextFeedback(*feedback_);
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

/**
 * @brief Connects to the HEBI actuator group and sets their parameters.
 *
 * @return true If connection and initialization succeeded
 * @return false Otherwise
 */
bool LIBRA_HEBI::Connect() {
    hebi::Lookup lookup;

    /* HEBIライブラリを改造して追加実装したavailable関数を使用。
     * 内部の_lookupがnullptrでないときに使用可能。
     * The HEBI library must be modified to implement the `available()` function.
     * It can be implemented by making the following additions to Lookup.hpp/.cpp:
     *
     * - Declare the following in the `public` field of the `Lookup` class.
     *     ```
     *     bool available();
     *     ```
     *
     * - Define the `available()` function as follows:
     *     ```
     *     bool Lookup::available() {
     *         return lookup_ != nullptr;
     *     }
     *     ```
     */
    /*
    if (!lookup.available()) {
        // colorize::Print("HEBI - Lookupを使用できません（LAN未接続）\n",
        //                 colorize::Level::kError);
        colorize::Print(
            "HEBI - Unable to use Lookup (LAN not connected)!\n",
            colorize::Level::kError);
        return false;
    }
    */

    group_ = lookup.getGroupFromNames({"X8-16"}, {"MA", "MB", "J1", "J2", "J3"});
    if (group_ == nullptr) {
        // colorize::Print("HEBI - アクチュエータが接続されていません\n",
        //                 colorize::Level::kError);
        colorize::Print("HEBI - Actuators not connected!\n",
                        colorize::Level::kError);
        command_->setPosition(Eigen::VectorXd::Zero(5));
        return false;
    }

    if (!command_->readSafetyParameters("params/safety.xml")) {
        // colorize::Print("HEBI - 安全パラメータファイルを読み込めません\n",
        //                 colorize::Level::kError);
        colorize::Print("HEBI - Failed to load safety parameters!\n",
                        colorize::Level::kError);
        system("pause");
        return false;
    }

    if (!command_->readGains("params/gain.xml")) {
        // colorize::Print("HEBI - ゲインパラメータファイルを読み込めません\n",
        //                 colorize::Level::kError);
        colorize::Print("HEBI - Failed to load gain parameters!\n",
                        colorize::Level::kError);
        system("pause");
        return false;
    }

    group_->sendCommand(*command_);  // initialize the actuator group
    command_->clear();
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());  // hold current position

    return true;
}

/**
 * @brief TODO.
 *
 * @param roll Desired roll angle, in degrees
 * @param pitch Desired pitch angle, in degrees
 * @param j1 Desired J1 (arm yaw) angle, in degrees
 * @param j2 Desired J2 (arm yaw) angle, in degrees
 * @param j3 Desired J3 (arm pitch) angle, in degrees
 */
void LIBRA_HEBI::Move(double roll, double pitch, double j1, double j2,
                      double j3) {
    Eigen::MatrixXd positions(5, 2);
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(5, 2);
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(5, 2);

    // Populate positions vector
    positions.col(0) = command_->getPosition();
    positions(Act::kHebiMA, 1) = -roll - pitch;  // 変更! - Change!
    positions(Act::kHebiMB, 1) = -roll + pitch;  // 変更! - Change!
    positions(Act::kHebiJ1, 1) = j1;
    positions(Act::kHebiJ2, 1) = -j2;
    positions(Act::kHebiJ3, 1) = j3;
    positions.col(1) *= M_PI / 180;  // convert to rad

    // TODO
    double max_diff_rad = 0;
    for (int i = 0; i < 5; i++) {
        if (abs(positions(i, 1) - positions(i, 0)) > max_diff_rad) {
            max_diff_rad = abs(positions(i, 1) - positions(i, 0));
        }
    }

    // TODO
    Eigen::VectorXd time(2);
    time << 0, max_diff_rad * 30 / M_PI;

    // Log start time and send movement command
    start_time_ = timeGetTime();
    trajectory_ =
        hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions,
                                                            &velocities,
                                                            &accelerations);
}

/**
 * @brief Clears the active actuator movement command(s).
 */
void LIBRA_HEBI::Stop() {
    trajectory_ = nullptr;
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the commanded position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's commanded position value, in degrees
 */
double LIBRA_HEBI::GetCommandPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = (-command_->getPosition()[Act::kHebiMA]
                   - command_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kPitch:
            ret = (-command_->getPosition()[Act::kHebiMA]
                   + command_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kJ1:
            ret = command_->getPosition()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -command_->getPosition()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = command_->getPosition()[Act::kHebiJ3];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's actual position value, in degrees
 */
double LIBRA_HEBI::GetFeedbackPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = (-feedback_->getPosition()[Act::kHebiMA]
                   - feedback_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kPitch:
            ret = (-feedback_->getPosition()[Act::kHebiMA]
                   + feedback_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kJ1:
            ret = feedback_->getPosition()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -feedback_->getPosition()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = feedback_->getPosition()[Act::kHebiJ3];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual torque value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's actual torque value, in Newton-meters
 */
double LIBRA_HEBI::GetFeedbackEffort(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = -feedback_->getEffort()[Act::kHebiMA]
                  - feedback_->getEffort()[Act::kHebiMB];
            break;
        case kPitch:
            ret = -feedback_->getEffort()[Act::kHebiMA]
                  + feedback_->getEffort()[Act::kHebiMB];
            break;
        case kJ1:
            ret = feedback_->getEffort()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -feedback_->getEffort()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = feedback_->getEffort()[Act::kHebiJ3];
            break;
    }

    return ret;
}

/**
 * @brief Returns the actual torque value for the 2-DoF Joint's "A" actuator.
 *
 * @return double The actuator's actual torque value, in Newton-meters
 */
double LIBRA_HEBI::GetFeedbackEffortMA() {
    return feedback_->getEffort()[Act::kHebiMA];
}

/**
 * @brief Returns the actual torque value for the 2-DoF Joint's "B" actuator.
 *
 * @return double The actuator's actual torque value, in Newton-meters
 */
double LIBRA_HEBI::GetFeedbackEffortMB() {
    return feedback_->getEffort()[Act::kHebiMB];
}
