/******************************************************************************
 * @file   log_thread.h
 * @brief  Consolidated logging class for LIBRA sensor data; header file.
 *
 * @note TODO: this might be abandoned?
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <fstream>

// Other Library Headers
#include <QThread>  // Qt::Core

// Project Headers
//   (none)

/**
 * @brief Consolidated logging class for LIBRA sensors.
 *        Creates a CSV file every time the LIBRA App is run.
 */
class LogThread : public QThread {
  public:
    explicit LogThread(QObject* parent, const bool& debug_mode);
    ~LogThread() override;

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
    };

    // --- Log Commands ---

    void TakeLogShot();

  signals:
    // --- Log Updates ---

  private:
    void run() override;

    // --- Helper Functions ---

    std::ofstream InitializeLog(std::string name);

    // --- Data Members ---

    bool debug_mode_{false};

    std::ofstream continuous_log_;
    std::ofstream snapshot_log_;
};