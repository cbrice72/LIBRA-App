/******************************************************************************
 * @file   tank_widget.cpp
 * @brief  Fluid tank visualization implementation file.
 *
 * @author brice.c.aa
 ******************************************************************************/

// Related Header
#include "tank_widget.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Getters & Setters
 * !Slots
 * !QWidget Overrides
 */

// Widget Animation

constexpr int kAnimTime = 1000 / 30;  // 1 s -> 1000 ms / 30 FPS animation

// Per-setup Fluid Flow Constants
#if LIBRA_VERSION == 1
// - 2*(3L + 6L + 10L)

static constexpr double kFillRate = 2 * 0.129;   // L/s
static constexpr double kDrainRate = 2 * 0.061;  // L/s
static constexpr double kCapacity = 38;          // L
#elif LIBRA_VERSION == 2
// - 3L + 6L + 10L

static constexpr double kFillRate = 0.129;   // L/s
static constexpr double kDrainRate = 0.061;  // L/s
static constexpr double kCapacity = 19;      // L
/*
// - 3L + 2*6L + 2*10L

static constexpr double kFillRate = ???;   // L/s
static constexpr double kDrainRate = ???;  // L/s
static constexpr double kCapacity = 35;    // L
*/
#endif

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 */
TankWidget::TankWidget(QWidget* parent)
    : QWidget(parent),
      level_(0.0),
      state_(Water::State::kStopped),
      animation_timer_(new QTimer(this)) {
    // Configure widget appearance
    setMinimumSize(100, 100);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    // Configure animation timer
    animation_timer_->setInterval(kAnimTime);

    connect(animation_timer_, &QTimer::timeout,   // Every timer timeout...
            this, &TankWidget::UpdateAnimation);  // ... trigger animation
}

/**
 * @brief Standard destructor.
 */
TankWidget::~TankWidget() {
    animation_timer_->stop();
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Controls the animated filling/draining of the tank.
 */
void TankWidget::UpdateAnimation() {
    if (!isVisible()) {
        animation_timer_->stop();
        return;
    }

    const double old_level = level_;
    const double time_step = animation_timer_->interval()
                             / 1000.0;  // convert ms to s

    // Calculate step size based on flow rate and frame time
    switch (state_) {
        case Water::State::kFilling:
            level_ = qMin(level_ + (kFillRate / kCapacity) * time_step, 1.0);

            // Check if we've hit the upper limit
            if (qFuzzyCompare(level_, 1.0)) {
                UpdateState(Water::State::kStopped);
                emit TankFull();
            }
            break;

        case Water::State::kDraining:
            level_ = qMax(level_ - (kDrainRate / kCapacity) * time_step, 0.0);

            // Check if we've hit the lower limit
            if (qFuzzyCompare(level_, 0.0)) {
                UpdateState(Water::State::kStopped);
                emit TankEmpty();
            }
            break;

        case Water::State::kStopped:
            // Do nothing
            break;
    }

    if (!qFuzzyCompare(old_level, level_)) {
        emit LevelChanged(level_);
        update();
    }
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns current fluid level.
 *
 * @return double Current fluid level as a fraction in the range [0.0, 1.0].
 */
double TankWidget::GetLevel() const {
    return level_;
}

/**
 * @brief Returns current fluid volume.
 *
 * @return double Current fluid volume (in L).
 */
double TankWidget::GetVolume() const {
    return level_ * kCapacity;
}

/**
 * @brief Overwrites the current fluid level (WARNING: see note).
 *
 * @param new_level A fraction in the range [0.0, 1.0]
 *
 * @note ONLY TO BE USED IF INACCURACIES ARE NOTICED IN REAL TIME!!!
 */
void TankWidget::OverrideLevel(double new_level) {
    // Validate input
    if (std::isnan(new_level) || std::isinf(new_level)) {
        return;
    }
    new_level = qBound(0.0, new_level, 1.0);

    // Override current level
    if (!qFuzzyCompare(level_, new_level)) {
        level_ = new_level;
        emit LevelChanged(level_);
        update();
    }
}

//------------------------------------------------------------------------------
// !Slots
//------------------------------------------------------------------------------

/**
 * @brief Controls the animation timer to change the level of the tank.
 *
 * @param state
 */
void TankWidget::UpdateState(Water::State state) {
    state_ = state;  // manages internal logic

    switch (state) {
        case Water::State::kFilling:
        case Water::State::kDraining:
            // Start filling or draining the tank
            // (NOTE: automatically stops when full/empty)
            if (!animation_timer_->isActive()) {
                animation_timer_->start();
            }
            break;
        case Water::State::kStopped:
        default:
            animation_timer_->stop();
            break;
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
