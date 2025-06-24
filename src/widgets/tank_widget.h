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
//   (none)

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
    // Constants for 3L*3 setup (LIBRA-I)
    /*
    static constexpr double kFillRate = 0.102;   // L/s
    static constexpr double kDrainRate = 0.068;  // L/s
    static constexpr double kCapacity = 99;      // L
    */

    // Constants for 3L + 6L + 10L setup (LIBRA-II)
    static constexpr double kFillRate = 0.129;   // L/s
    static constexpr double kDrainRate = 0.061;  // L/s
    static constexpr double kCapacity = 19;      // L

    // Constants for 3L + 6L*2 + 10L*2 setup (LIBRA-II)
    /*
    static constexpr double kFillRate = ???;   // L/s
    static constexpr double kDrainRate = ???;  // L/s
    static constexpr double kCapacity = 35;      // L
    */

    explicit TankWidget(QWidget* parent = nullptr);
    ~TankWidget() override;

    // Delete copy constructor
    TankWidget(const TankWidget&) = delete;
    TankWidget& operator=(const TankWidget&) = delete;

    // --- Getters & Setters ---

    double GetLevel() const;
    double GetVolume() const;

    void OverrideLevel(double new_level);

  public slots:
    void Fill();
    void Drain();
    void Stop();

  signals:
    void LevelChanged(double level);
    void TankFull();
    void TankEmpty();

  protected:
    // --- QWidget Overrides ---

    void paintEvent(QPaintEvent* event) override;

  private:
    /**
     * @brief Fluid system state.
     */
    enum class FlowMode { Stopped = 0, Filling, Draining };

    // --- Helper Functions ---

    void UpdateAnimation();

    // --- Data Members ---

    double level_;
    FlowMode flow_mode_;
    QTimer* animation_timer_{nullptr};
};
