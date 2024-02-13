/******************************************************************************
 * @file   main.cpp
 * @brief  Root file of the LIBRA App (v2) program.
 *
 * @author brice.c.aa
 * @date   2024/2/13
 ******************************************************************************/

// Related Header
//   (none)
// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   Qt
#include <QApplication>
// Project Headers
#include "main_window.h"

/**
 * @brief The designated start of the program.
 *
 * @param argc Argument
 * count
 * @param argv Argument vector
 * @return Return code from `exit()`
 * after the application exits
 */
int main(int argc, char* argv[]) {
    // Initialize the app
    QApplication app(argc  , argv);

    // Display the main app window
    MainWindow w_main;
    w_main.show();
    return app.exec();
}
