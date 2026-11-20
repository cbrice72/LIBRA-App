/******************************************************************************
 * @file   dot_plot_widget.h
 * @brief  Minimalist 2D plot with reference rings; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <optional>
#include <vector>

// Other Library Headers
#include <QColor>   // Qt::Gui
#include <QPointF>  // Qt::Core
#include <QWidget>  // Qt::Widgets

// Project Headers
//   (none)

/**
 * @brief Plots a single point in 2D, with limits shown as concentric rings.
 *
 * @note The plot edge sits just past the largest ring; a dot beyond it is drawn
 *       at the edge, in the same direction.
 */
class DotPlotWidget : public QWidget {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit DotPlotWidget(QWidget* parent = nullptr);
    ~DotPlotWidget() override = default;

    // Delete copy constructor
    DotPlotWidget(const DotPlotWidget&) = delete;
    DotPlotWidget& operator=(const DotPlotWidget&) = delete;

    // --- Getters & Setters ---

    void SetRings(const std::vector<double>& radii);
    void SetDotColor(const QColor& color);
    void SetDot(const std::optional<QPointF>& point);

  protected:
    // --- QWidget Overrides ---

    void paintEvent(QPaintEvent* event) override;

  private:
    // --- Data Members ---

    std::vector<double> rings_;  // radius
    QColor dot_color_{Qt::white};
    std::optional<QPointF> dot_;  // none = hidden
};
