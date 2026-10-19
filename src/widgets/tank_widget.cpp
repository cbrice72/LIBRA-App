/******************************************************************************
 * @file   tank_widget.cpp
 * @brief  Fluid tank visualization; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "tank_widget.h"

// C++ Standard Library Headers
#include <cmath>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Getters & Setters
 * !QWidget Overrides
 */

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 */
TankWidget::TankWidget(QWidget* parent) : QWidget(parent) {
    // Configure widget appearance
    setMinimumSize(100, 100);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Gets current fluid level, as a fraction in [0.0, 1.0].
 */
double TankWidget::GetLevel() const {
    return level_;
}

/**
 * @brief Sets the shown fluid level.
 *
 * @param level A fraction in the range [0.0, 1.0]
 */
void TankWidget::SetLevel(double level) {
    // Validate input
    if (std::isnan(level) || std::isinf(level)) {
        return;
    }
    level = qBound(0.0, level, 1.0);

    if (!qFuzzyCompare(level_, level)) {
        level_ = level;
        emit LevelChanged(level_);
        update();
    }
}

//------------------------------------------------------------------------------
// !QWidget Overrides
//------------------------------------------------------------------------------

/**
 * @brief Overrides `paintEvent()` to handle this widget's custom drawing.
 */
void TankWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Common constants
    const int margin = 10;
    const int border_thickness = 2;
    const int corner_radius = 10;

    // Define and draw tank
    QRectF tank = rect().adjusted(margin, margin, -margin, -margin);

    painter.setPen(QPen(Qt::darkGray, border_thickness));
    painter.setBrush(Qt::white);
    painter.drawRoundedRect(tank, corner_radius, corner_radius);

    // Define and draw fluid
    const double fluid_height = tank.height() * level_;

    QRectF fluid = tank.adjusted(border_thickness, tank.height() - fluid_height,
                                 -border_thickness, -border_thickness);

    // - Transparent blue gradient to symbolize water
    QLinearGradient gradient(fluid.topLeft(), fluid.topRight());
    gradient.setColorAt(0, QColor(0, 120, 255, 200));
    gradient.setColorAt(1, QColor(0, 160, 255, 200));

    painter.setPen(Qt::NoPen);
    painter.setBrush(gradient);
    painter.drawRoundedRect(fluid, corner_radius, corner_radius);

    // Draw level percentage (middle of tank)
    painter.setPen(Qt::black);
    QFont font = painter.font();
    font.setPixelSize(16);
    painter.setFont(font);

    painter.drawText(tank, Qt::AlignCenter,
                     QString("%1%").arg(qRound(level_ * 100)));

    // Draw level markers (left of tank)
    const int marker_width = 10;
    const int num_markers = 5;
    painter.setPen(QPen(Qt::black, 1));

    for (int i = 0; i <= num_markers; ++i) {
        double y = tank.top() + (tank.height() * i / num_markers);
        painter.drawLine(tank.left() - margin / 2, y,
                         tank.left() + marker_width, y);
    }
}
