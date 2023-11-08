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
//   (none)

/* --- TABLE OF CONTENTS ---
 * !General Functions
 * !Motor Commands
 * !Getters & Setters
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new LIBRA_HEBI object.
 */
LIBRA_HEBI::LIBRA_HEBI() {
    command = new hebi::GroupCommand(5);
    feedback = new hebi::GroupFeedback(5);

    // タイマ割り込み開始 10ms毎に割り込み処理をする
    // Timer interrupt: start interrupt processing every 10ms
    timeSetEvent(10, 0, callback, reinterpret_cast<DWORD>(this),
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
void CALLBACK LIBRA_HEBI::callback(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1,
                                   DWORD dw2) {
    (reinterpret_cast<LIBRA_HEBI*>(dwUser))->loop();
}

/**
 * @brief TODO.
 */
void LIBRA_HEBI::loop() {
    command->setVelocity(Eigen::VectorXd::Zero(5));

    if (trajectory != nullptr) {
        double time = (timeGetTime() - start_time) / 1000.0;
        if (trajectory->getDuration() > time) {
            Eigen::VectorXd pos_cmd(5);
            Eigen::VectorXd vel_cmd(5);
            trajectory->getState(time, &pos_cmd, &vel_cmd, nullptr);
            command->setPosition(pos_cmd);
            command->setVelocity(vel_cmd);
        }
    }

    if (group != nullptr) {
        group->sendCommand(*command);
        group->getNextFeedback(*feedback);
    }
}

//------------------------------------------------------------------------------
// !Motor Commands
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int LIBRA_HEBI::connect() {
    int rtn = 0;
    hebi::Lookup lookup;

    // HEBIライブラリを改造して追加実装したavailable関数を使用。内部の_lookupがnullptrでないときに使用可能。
    /* The HEBI library must be modified to implement the `available()`
     * function. This function returns `true` when the Lookup object's internal
     * `lookup_` data member is not nullptr. It can be implemented by making the
     * following additions to Lookup.hpp/.cpp:
     * - Declare the following in the `public` field of the `Lookup` class.
     *     ```
     *     bool available();
     *     ```
     * - Define the `available()` function as follows:
     *     ```
     *     bool Lookup::available() {
     *         return lookup_ != nullptr;
     *     }
     *     ```
     */
    /*
    if (!lookup.available()) {
        //printf("[エラー]Lookupを使用できません（LAN未接続）\n");
        std::cout << "[ERROR] HEBI - Unable to use Lookup (LAN not connected)!"
                  << std::endl;
        return -1;
    }
    */

    group = lookup.getGroupFromNames({"X8-16"}, {"MA", "MB", "J1", "J2", "J3"});
    if (group == nullptr) {
        // printf("[エラー]HEBI - アクチュエータが接続されていません\n");
        std::cout << "[ERROR] HEBI - Actuators not connected!" << std::endl;
        command->setPosition(Eigen::VectorXd::Zero(5));
        return -1;
    }

    if (!command->readSafetyParameters("params/safety.xml")) {
        // printf("[エラー]HEBI - 安全パラメータファイルを読み込めません\n");
        std::cout << "[ERROR] HEBI - Failed to load safety parameters!"
                  << std::endl;
        system("pause");
        return -1;
    }
    if (!command->readGains("params/gain.xml")) {
        // printf("[エラー]HEBI - ゲインパラメータファイルを読み込めません\n");
        std::cout << "[HEBI] HEBI - Failed to load gain parameters!"
                  << std::endl;
        system("pause");
        return -1;
    }

    group->sendCommand(*command);
    command->clear();
    group->getNextFeedback(*feedback);
    command->setPosition(feedback->getPosition());

    return 0;
}

/**
 * @brief TODO.
 *
 * @param roll TODO
 * @param pitch TODO
 * @param j1 TODO
 * @param j2 TODO
 * @param j3 TODO
 */
void LIBRA_HEBI::move(double roll, double pitch, double j1, double j2,
                      double j3) {
    Eigen::MatrixXd positions(5, 2);
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(5, 2);
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(5, 2);

    positions.col(0) = command->getPosition();
    positions(HEBI_MA, 1) = -roll - pitch;  // 変更! - Change!
    positions(HEBI_MB, 1) = -roll + pitch;  // 変更! - Change!
    positions(HEBI_J1, 1) = j1;
    positions(HEBI_J2, 1) = -j2;
    positions(HEBI_J3, 1) = j3;
    positions.col(1) *= M_PI / 180;

    double max_diff_rad = 0;
    for (int i = 0; i < 5; i++) {
        if (abs(positions(i, 1) - positions(i, 0)) > max_diff_rad) {
            max_diff_rad = abs(positions(i, 1) - positions(i, 0));
        }
    }

    Eigen::VectorXd time(2);
    time << 0, max_diff_rad * 30 / M_PI;

    start_time = timeGetTime();
    trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
        time, positions, &velocities, &accelerations);
}

/**
 * @brief TODO.
 */
void LIBRA_HEBI::stop() {
    trajectory = nullptr;
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @param joint TODO
 * @return double TODO
 */
double LIBRA_HEBI::getCommandPosition(int joint) {
    double ret = 0;
    if (joint == ROLL) {
        ret =
            (-command->getPosition()[HEBI_MA] - command->getPosition()[HEBI_MB])
            / 2;
    }
    if (joint == PITCH) {
        ret =
            (-command->getPosition()[HEBI_MA] + command->getPosition()[HEBI_MB])
            / 2;
    }
    if (joint == J1) {
        ret = command->getPosition()[HEBI_J1];
    }
    if (joint == J2) {
        ret = -command->getPosition()[HEBI_J2];
    }
    if (joint == J3) {
        ret = command->getPosition()[HEBI_J3];
    }
    ret *= 180 / M_PI;
    return ret;
}

/**
 * @brief TODO.
 *
 * @param joint TODO
 * @return double TODO
 */
double LIBRA_HEBI::getFeedbackPosition(int joint) {
    double ret = 0;
    if (joint == ROLL) {
        ret = (-feedback->getPosition()[HEBI_MA]
               - feedback->getPosition()[HEBI_MB])
              / 2;
    }
    if (joint == PITCH) {
        ret = (-feedback->getPosition()[HEBI_MA]
               + feedback->getPosition()[HEBI_MB])
              / 2;
    }
    if (joint == J1) {
        ret = feedback->getPosition()[HEBI_J1];
    }
    if (joint == J2) {
        ret = -feedback->getPosition()[HEBI_J2];
    }
    if (joint == J3) {
        ret = feedback->getPosition()[HEBI_J3];
    }
    ret *= 180 / M_PI;
    return ret;
}

/**
 * @brief TODO.
 *
 * @param joint TODO
 * @return double TODO
 */
double LIBRA_HEBI::getFeedbackEffort(int joint) {
    double ret = 0;
    if (joint == ROLL) {
        ret = -feedback->getEffort()[HEBI_MA] - feedback->getEffort()[HEBI_MB];
    }
    if (joint == PITCH) {
        ret = -feedback->getEffort()[HEBI_MA] + feedback->getEffort()[HEBI_MB];
    }
    if (joint == J1) {
        ret = feedback->getEffort()[HEBI_J1];
    }
    if (joint == J2) {
        ret = -feedback->getEffort()[HEBI_J2];
    }
    if (joint == J3) {
        ret = feedback->getEffort()[HEBI_J3];
    }
    return ret;
}

double LIBRA_HEBI::getFeedbackEffortMA() {
    return feedback->getEffort()[HEBI_MA];
}

/**
 * @brief TODO.
 *
 * @return double TODO
 */
double LIBRA_HEBI::getFeedbackEffortMB() {
    return feedback->getEffort()[HEBI_MB];
}
