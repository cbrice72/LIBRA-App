/******************************************************************************
 * @file   main.cpp
 * @brief  Root file of the LIBRA App (v2) program.
 *
 * @author brice.c.aa
 ******************************************************************************/

// Related Header
//   (none)

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QApplication>  // Qt::Widgets
#ifdef BUILD_WITH_ROS2
# include <rclcpp/rclcpp.hpp>  // ROS2 Core
#endif

// Project Headers
#include "ui/main_window/main_window.h"

/**
 * @brief The designated start of the program.
 *
 * @param argc Argument count
 * @param argv Argument vector
 * @return int The `exit()` code after the application exits
 */
int main(int argc, char* argv[]) {
#ifdef BUILD_WITH_ROS2
    // Ensure the ROS context is initialized BEFORE the app
    rclcpp::init(argc, argv);
#endif

    // Initialize the app
    QApplication app(argc, argv);
    QIcon::setThemeName("Papirus-Dark");  // "Papirus" for light theme

    // Display the main app window
    MainWindow w_main;
    w_main.show();
    int result = app.exec();

    // Cleanup
#ifdef BUILD_WITH_ROS2
    rclcpp::shutdown();
#endif

    return result;
}
