/******************************************************************************
 * @file   tank_widget.h
 * @brief  Fluid tank visualization definition file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QPainter>      // Qt::Gui
#include <QPainterPath>  // Qt::Gui
#include <QTimer>        // Qt::Core
#include <QWidget>       // Qt::Widgets

// Project Headers
#include "arduino_defs.h"  // Water::State

/**
 * @brief A simple widget for visualizing a tank's fluid level.
 *
 * @todo TODO(brice.c.aa): Check for correctness.
 */
class TankWidget : public QWidget {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

    // Enable animation and styling through Qt's property framework
    Q_PROPERTY(double GetLevel READ GetLevel NOTIFY LevelChanged)

  public:
    explicit TankWidget(QWidget* parent = nullptr);
    ~TankWidget() override;

    // Delete copy constructor
    TankWidget(const TankWidget&) = delete;
    TankWidget& operator=(const TankWidget&) = delete;

    // --- Getters & Setters ---

    double GetLevel() const;
    double GetVolume() const;

    void UpdateState(Water::State state);
    void OverrideLevel(double new_level);

  signals:
    void LevelChanged(double level);
    void TankFull();
    void TankEmpty();

  protected:
    // --- QWidget Overrides ---

    void paintEvent(QPaintEvent* event) override;

  private:
    // --- Helper Functions ---

    void UpdateAnimation();

    // --- Data Members ---

    double level_;
    Water::State state_;
    QTimer* animation_timer_{nullptr};
};
