/******************************************************************************
 * @file   tank_widget.h
 * @brief  Fluid tank visualization definition file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

#include <QPainter>      // Qt::Gui
#include <QPainterPath>  // Qt::Gui
#include <QTimer>        // Qt::Core
#include <QWidget>       // Qt::Widgets

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
    Q_PROPERTY(double level READ level NOTIFY LevelChanged)

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

    /**
     * @brief Standard constructor.
     */
    explicit TankWidget(QWidget* parent = nullptr)
        : QWidget(parent), level_(0.0), animation_timer_(new QTimer(this)) {
        setMinimumSize(100, 100);
        setSizePolicy(QSizePolicy::MinimumExpanding,
                      QSizePolicy::MinimumExpanding);

        // Configure animation timer
        animation_timer_->setInterval(33);  // 30 FPS (1000ms / 30fps ≈ 33ms)
        connect(animation_timer_, &QTimer::timeout,   // Every timer timeout...
                this, &TankWidget::UpdateAnimation);  // ... trigger animation
    }

    /**
     * @brief Getter for current fluid level.
     *
     * @return Current fluid level as a fraction in the range [0.0, 1.0].
     */
    double level() const {
        return level_;
    }

    /**
     * @brief Getter for current fluid volume.
     *
     * @return Current fluid volume (in L).
     */
    double volume() const {
        return level_ * kCapacity;
    }

    /**
     * @brief Setter for overriding the current fluid level (WARNING: see note).
     *
     * @param new_level A fraction in the range [0.0, 1.0]
     *
     * @note ONLY USE IF INACCURACIES ARE NOTICED IN REAL TIME!!!
     */
    void OverrideLevel(double new_level) {
        // Validate input
        new_level = qBound(0.0, new_level, 1.0);

        // Override current level
        level_ = new_level;
        update();
    }

  public slots:

    /**
     * @brief Start filling the tank.
     *
     * @note Stops filling when full or upon StopFlow().
     */
    void Fill() {
        flow_mode_ = FlowMode::Filling;
        if (!animation_timer_->isActive()) {
            animation_timer_->start();
        }
    }

    /**
     * @brief Start draining the tank continuously.
     *
     * @note Stops filling when empty or upon StopFlow().
     */
    void Drain() {
        flow_mode_ = FlowMode::Draining;
        if (!animation_timer_->isActive()) {
            animation_timer_->start();
        }
    }

    /**
     * @brief Stop any ongoing fill/drain operation.
     */
    void Stop() {
        flow_mode_ = FlowMode::Stopped;
        animation_timer_->stop();
    }

  signals:
    void LevelChanged(double level);
    void TankFull();
    void TankEmpty();

  protected:

    /**
     * @brief Override `paintEvent()` to handle this widget's custom drawing.
     */
    void paintEvent(QPaintEvent*) override {
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
        const double fluid_top = tank.bottom() - fluid_height;

        QRectF fluid = tank.adjusted(border_thickness,
                                     tank.height() - fluid_height,
                                     //- (border_thickness * 4),
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

  private slots:

    /**
     * @brief Controls the animated filling/draining of the tank.
     */
    void UpdateAnimation() {
        double old_level = level_;
        const double time_step = animation_timer_->interval()
                                 / 1000.0;  // convert ms to s

        // Calculate step size based on flow rate and frame time
        switch (flow_mode_) {
            case FlowMode::Filling:
                level_ = qMin(level_ + (kFillRate / kCapacity) * time_step, 1.0);

                // Check if we've hit the upper limit
                if (qFuzzyCompare(level_, 1.0)) {
                    Stop();
                    emit TankFull();
                }
                break;

            case FlowMode::Draining:
                level_ = qMax(level_ - (kDrainRate / kCapacity) * time_step,
                              0.0);

                // Check if we've hit the lower limit
                if (qFuzzyCompare(level_, 0.0)) {
                    Stop();
                    emit TankEmpty();
                }
                break;
        }

        if (!qFuzzyCompare(old_level, level_)) {
            emit LevelChanged(level_);
        }
        update();
    }

  private:
    /**
     * @brief Fluid system state.
     */
    enum class FlowMode { Stopped = 0, Filling, Draining };

    double level_{0.0};
    FlowMode flow_mode_{FlowMode::Stopped};
    QTimer* animation_timer_;
};
