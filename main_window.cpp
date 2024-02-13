/******************************************************************************
 * @file   main_window.cpp
 * @brief  Main app window implementation file.
 *
 * @author brice.c.aa
 * @date   2023/7/4
 ******************************************************************************/

// Related Header
#include "main_window.h"
// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   HEBI
#include "hebi.h"
//   Maxon
//#include "Maxon/Definitions.h"
// Project Headers
#include "open_epos_window.h"
#include "ui_main_window.h"
//#include "utility.h"  // use pretty_print.h

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Menu Bar
 * !Main Window
 * !Uncategorized
 */

/* Constants */

//constexpr uint example = 0;

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget (default: nullptr)
 */
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow) {
    ui_->setupUi(this);
}

/**
 * @brief Standard destructor.
 */
MainWindow::~MainWindow() {}


//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Menu Bar
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Main Window
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Uncategorized
//------------------------------------------------------------------------------
