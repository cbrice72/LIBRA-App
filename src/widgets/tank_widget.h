/******************************************************************************
 * @file   tank_widget.h
 * @brief  Fluid tank visualization; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QPainter>      // Qt::Gui
#include <QPainterPath>  // Qt::Gui
#include <QWidget>       // Qt::Widgets

// Project Headers
//   (none)

/**
 * @brief A simple widget for visualizing a tank's fluid level.
 */
class TankWidget : public QWidget {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

    // Enable animation and styling through Qt's property framework
    Q_PROPERTY(double GetLevel READ GetLevel NOTIFY LevelChanged)

  public:
    explicit TankWidget(QWidget* parent = nullptr);
    ~TankWidget() override = default;

    // Delete copy constructor
    TankWidget(const TankWidget&) = delete;
    TankWidget& operator=(const TankWidget&) = delete;

    // --- Getters & Setters ---

    double GetLevel() const;
    void SetLevel(double level);

  signals:
    void LevelChanged(const double& level);

  protected:
    // --- QWidget Overrides ---

    void paintEvent(QPaintEvent* event) override;

  private:
    // --- Data Members ---

    double level_{0.0};
};
