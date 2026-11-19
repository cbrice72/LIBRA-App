/******************************************************************************
 * @file   line_graph_widget.cpp
 * @brief  Minimalist scrolling line graph; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "line_graph_widget.h"

// C++ Standard Library Headers
#include <algorithm>
#include <cmath>
#include <limits>

// Other Library Headers
#include <QPainter>   // Qt::Gui
#include <QPolygonF>  // Qt::Gui

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Getters & Setters
 * !QWidget Overrides
 * !Helper Functions
 */

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 */
LineGraphWidget::LineGraphWidget(QWidget* parent) : QWidget(parent) {
    // Configure widget appearance
    setMinimumSize(200, 100);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Sets the sample spacing and how much history to keep; clears stored
 *        samples.
 *
 * @param period_s Time between samples (s)
 * @param history_s Time span to keep, i.e., the longest window (s)
 */
void LineGraphWidget::SetSampling(double period_s, double history_s) {
    // Validate input
    if (!std::isfinite(period_s) || !std::isfinite(history_s)) {
        qCritical(
            "LineGraphWidget::SetSampling(): period or history is not finite");
        return;
    }
    if (period_s <= 0.0 || history_s < period_s) {
        qCritical("LineGraphWidget::SetSampling(): period must be > 0 and <= "
                  "history");
        return;
    }

    // Update members
    period_s_ = period_s;
    capacity_ = static_cast<std::size_t>(std::ceil(history_s / period_s));
    for (auto& series : series_) {
        series.samples.clear();
    }
    update();
}

/**
 * @brief Sets the shown time span.
 *
 * @param window_s Time span (s); clamped to history
 */
void LineGraphWidget::SetWindow(double window_s) {
    // Validate input
    if (!std::isfinite(window_s) || window_s <= 0.0) {
        qCritical("LineGraphWidget::SetWindow(): window is not finite or <= 0");
        return;
    }

    // Update members
    window_s_ = std::min(window_s, capacity_ * period_s_);
    update();
}

/**
 * @brief Adds a data series (line).
 *
 * @param color Line color
 *
 * @note Clears all stored samples so the new line starts at the same time as
 *       the others.
 */
std::size_t LineGraphWidget::AddSeries(const QColor& color) {
    // No validation necessary, just update members
    for (auto& series : series_) {
        series.samples.clear();
    }
    series_.push_back({color, {}});
    update();

    return series_.size() - 1;  // index
}

/**
 * @brief Appends a sample to every line, dropping the oldest once history
 *        is full.
 *
 * @param values One per line, in `AddSeries()` index order; assumed to be
 *               exactly one sampling period after the last
 */
void LineGraphWidget::AddSamples(const std::vector<double>& values) {
    // Validate input
    if (values.size() != series_.size()) {
        qWarning("LineGraphWidget::AddSamples(): number of values (%zu) does "
                 "not match number of series (%zu)",
                 values.size(), series_.size());
        return;
    }

    auto is_inf = [](double val) { return std::isinf(val); };
    if (std::any_of(values.begin(), values.end(), is_inf)) {
        qWarning("LineGraphWidget::AddSamples(): one or more values are "
                 "infinite");
        return;
    }

    // Update members
    for (std::size_t i = 0; i < series_.size(); ++i) {
        auto& samples = series_[i].samples;
        samples.push_back(values[i]);
        // Drop oldest sample if history is full
        if (samples.size() > capacity_) {
            samples.pop_front();
        }
    }
    update();
}

/**
 * @brief Gets a line's average over the shown window, skipping gaps.
 *
 * @param series Line index, from `AddSeries()`
 */
std::optional<double> LineGraphWidget::GetWindowMean(std::size_t series) const {
    // Validate input
    if (series >= series_.size()) {
        qWarning("LineGraphWidget::GetWindowMean(): series index (%zu) is out "
                 "of range (%zu)",
                 series, series_.size());
        return std::nullopt;
    }
    const auto& samples = series_[series].samples;
    const std::size_t shown = std::min(samples.size(), GetWindowCount());
    if (shown == 0) {
        qWarning("LineGraphWidget::GetWindowMean(): series (%zu) has no "
                 "samples",
                 series);
        return std::nullopt;
    }

    double sum = 0.0;
    std::size_t count = 0;
    for (auto it = samples.end() - static_cast<std::ptrdiff_t>(shown);
         it != samples.end(); ++it) {
        if (!std::isnan(*it)) {
            sum += *it;
            ++count;
        }
    }
    if (count == 0) {
        return std::nullopt;
    }
    return sum / static_cast<double>(count);
}

//------------------------------------------------------------------------------
// !QWidget Overrides
//------------------------------------------------------------------------------

/**
 * @brief Overrides `paintEvent()` to handle this widget's custom drawing.
 */
void LineGraphWidget::paintEvent(QPaintEvent* /*event*/) {
    if (series_.empty()) {
        return;
    }

    // Newest sample sits at right edge
    const std::size_t window_count = GetWindowCount();
    const std::size_t shown = std::min(series_.front().samples.size(),
                                       window_count);
    if (shown < 2) {
        return;
    }
    const auto window_start = static_cast<std::ptrdiff_t>(
        series_.front().samples.size() - shown);

    // Use one scale for all lines, for ease of comparison
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -y_min;
    for (const auto& series : series_) {
        for (auto it = series.samples.begin() + window_start;
             it != series.samples.end(); ++it) {
            if (!std::isnan(*it)) {  // skip gaps
                y_min = std::min(y_min, *it);
                y_max = std::max(y_max, *it);
            }
        }
    }
    if (y_min > y_max) {
        return;  // only gaps in the window
    }
    if (y_max - y_min < 1e-9) {
        // If line is relatively flat, add padding so it's easier to see
        y_min -= 0.5;
        y_max += 0.5;
    }

    // Draw each line in pixel space
    const double line_width = 1.5;
    const QRectF area = QRectF(rect()).adjusted(line_width, line_width,
                                                -line_width, -line_width);
    const double dx = area.width() / static_cast<double>(window_count - 1);
    auto to_x = [&](std::size_t i) {
        return std::floor(area.right() - ((shown - 1 - i) * dx));
    };
    auto to_y = [&](double val) {
        return area.bottom()
               - ((val - y_min) / (y_max - y_min) * area.height());
    };

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    for (const auto& series : series_) {
        const auto first = series.samples.begin() + window_start;

        // Carry out min-max downsampling
        // (the wider the window, the more samples exist on a single pixel, so
        // samples sharing a pixel are grouped into bin + drawn as its min/max)
        QPolygonF line;
        double bin_x = kNoData;
        double bin_min = kNoData;
        double bin_max = kNoData;
        auto add_bin = [&]() {
            if (std::isnan(bin_min)) {
                return;
            }
            line << QPointF(bin_x, to_y(bin_min));
            if (bin_max != bin_min) {
                line << QPointF(bin_x, to_y(bin_max));
            }
        };

        painter.setPen(QPen(series.color, line_width));
        for (std::size_t i = 0; i < shown; ++i) {
            const double val = first[static_cast<std::ptrdiff_t>(i)];
            const double x = to_x(i);
            if (std::isnan(val)) {
                // Gap: draw the line so far, then start a new one
                add_bin();
                painter.drawPolyline(line);
                line.clear();
                bin_x = kNoData;
                bin_min = kNoData;
                bin_max = kNoData;
            } else if (x != bin_x) {
                // New pixel: draw the previous bin, then start a new one
                add_bin();
                bin_x = x;
                bin_min = val;
                bin_max = val;
            } else {
                // Same pixel: update the bin's min/max
                bin_min = std::min(bin_min, val);
                bin_max = std::max(bin_max, val);
            }
        }
        add_bin();
        painter.drawPolyline(line);
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Number of samples spanning the shown window (inherently, at least 2).
 */
std::size_t LineGraphWidget::GetWindowCount() const {
    return static_cast<std::size_t>(
        std::max(2.0, std::round(window_s_ / period_s_)));
}
