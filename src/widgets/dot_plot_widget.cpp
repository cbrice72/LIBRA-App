/******************************************************************************
 * @file   dot_plot_widget.cpp
 * @brief  Minimalist 2D plot with reference rings; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "dot_plot_widget.h"

// C++ Standard Library Headers
#include <algorithm>
#include <cmath>

// Other Library Headers
#include <QPainter>  // Qt::Gui

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Getters & Setters
 * !QWidget Overrides
 */

// Appearance

constexpr double kEdgeMargin = 1.2;  // plot edge, as multiple of largest ring
constexpr double kDotRadius = 4.0;   // px
constexpr double kLineWidth = 1.0;   // px
constexpr double kAxisOpacity = 0.4;

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 */
DotPlotWidget::DotPlotWidget(QWidget* parent) : QWidget(parent) {
    // Configure widget appearance
    setMinimumSize(60, 60);
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Sets the reference rings, which also determines graph scale.
 *
 * @param rings Rings to draw; radii in data units (> 0)
 */
void DotPlotWidget::SetRings(const std::vector<Ring>& rings) {
    // Validate input
    auto is_valid = [](const Ring& ring) {
        return std::isfinite(ring.radius) && ring.radius > 0.0;
    };
    if (!std::all_of(rings.begin(), rings.end(), is_valid)) {
        qCritical(
            "DotPlotWidget::SetRings(): all radii must be finite and > 0");
        return;
    }

    // Update members
    rings_ = rings;
    update();
}

/**
 * @brief Sets the dot's color.
 *
 * @param color Fill color
 */
void DotPlotWidget::SetDotColor(const QColor& color) {
    // No validation necessary, just update members
    dot_color_ = color;
    update();
}

/**
 * @brief Moves the dot.
 *
 * @param point Position in data units (+x: right, +y: up); `none` hides the dot
 */
void DotPlotWidget::SetDot(const std::optional<QPointF>& point) {
    // Validate input
    if (point && !(std::isfinite(point->x()) && std::isfinite(point->y()))) {
        qWarning("DotPlotWidget::SetDot(): point is not finite");
        return;
    }

    // Update members
    dot_ = point;
    update();
}

//------------------------------------------------------------------------------
// !QWidget Overrides
//------------------------------------------------------------------------------

/**
 * @brief Overrides `paintEvent()` to handle this widget's custom drawing.
 */
void DotPlotWidget::paintEvent(QPaintEvent* /*event*/) {
    // Ensure plot fits inside widget (map to shortest side)
    double largest_ring = rings_.empty() ? 1.0 : 0.0;
    for (const Ring& ring : rings_) {
        largest_ring = std::max(largest_ring, ring.radius);
    }
    const double edge = kEdgeMargin * largest_ring;
    const double edge_px = (std::min(width(), height()) / 2.0)
                           - kDotRadius;  // dot at edge shouldn't get clipped
    const double px_per_unit = edge_px / edge;
    const QPointF center = QRectF(rect()).center();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Axes
    QColor axis_color = palette().color(QPalette::WindowText);
    axis_color.setAlphaF(kAxisOpacity);
    painter.setPen(QPen(axis_color, kLineWidth));
    painter.drawLine(center - QPointF(edge_px, 0.0),
                     center + QPointF(edge_px, 0.0));
    painter.drawLine(center - QPointF(0.0, edge_px),
                     center + QPointF(0.0, edge_px));

    // Rings
    for (const Ring& ring : rings_) {
        painter.setPen(QPen(ring.color, kLineWidth));
        painter.drawEllipse(center, ring.radius * px_per_unit,
                            ring.radius * px_per_unit);
    }

    if (!dot_) {
        return;
    }

    // Dot
    const double dist = std::hypot(dot_->x(), dot_->y());
    const double shrink = (dist > edge) ? edge / dist : 1.0;
    const QPointF offset(dot_->x() * shrink * px_per_unit,
                         -dot_->y() * shrink * px_per_unit);

    painter.setPen(Qt::NoPen);
    painter.setBrush(dot_color_);
    painter.drawEllipse(center + offset, kDotRadius, kDotRadius);
}
