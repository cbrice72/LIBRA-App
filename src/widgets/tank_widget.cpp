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
 * @brief Gets current fluid level as a fraction of capacity.
 *
 * @note May be > 1.0 if overfilled.
 */
double TankWidget::GetLevel() const {
    return capacity_ > 0.0 ? volume_ / capacity_ : 0.0;
}

/**
 * @brief Sets the tank capacity (for visualizing fill level).
 *
 * @param capacity Tank capacity (L)
 */
void TankWidget::SetCapacity(double capacity) {
    // Validate input
    if (!std::isfinite(capacity) || capacity <= 0.0) {
        return;
    }

    capacity_ = capacity;
    update();
}

/**
 * @brief Sets the shown water volume.
 *
 * @param volume Water volume (L)
 */
void TankWidget::SetVolume(double volume) {
    // Validate input
    if (!std::isfinite(volume)) {
        return;
    }

    volume = qMax(volume, 0.0);  // can exceed capacity (overfilled)
    if (!qFuzzyCompare(volume_, volume)) {
        volume_ = volume;
        emit LevelChanged(GetLevel());
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
    const double fluid_height = tank.height() * qMin(GetLevel(), 1.0);

    QRectF fluid = tank.adjusted(border_thickness, tank.height() - fluid_height,
                                 -border_thickness, -border_thickness);

    // - Transparent blue gradient to symbolize water
    QLinearGradient gradient(fluid.topLeft(), fluid.topRight());
    gradient.setColorAt(0, QColor(0, 120, 255, 200));
    gradient.setColorAt(1, QColor(0, 160, 255, 200));

    painter.setPen(Qt::NoPen);
    painter.setBrush(gradient);
    painter.drawRoundedRect(fluid, corner_radius, corner_radius);

    // Draw volume (middle of tank)
    painter.setPen(Qt::black);
    QFont font = painter.font();
    font.setPixelSize(16);
    painter.setFont(font);

    painter.drawText(tank, Qt::AlignCenter,
                     QString("%1 L").arg(volume_, 0, 'f', 2));

    // Draw level markers (left of tank)
    const int marker_width = 10;
    const int num_markers = 5;
    painter.setPen(QPen(Qt::black, 1));

    for (int i = 0; i <= num_markers; ++i) {
        double y = tank.top() + (tank.height() * i / num_markers);
        painter.drawLine(tank.left() - (margin / 2.0), y,
                         tank.left() + marker_width, y);
    }
}
