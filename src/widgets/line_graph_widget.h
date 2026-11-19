/******************************************************************************
 * @file   line_graph_widget.h
 * @brief  Minimalist scrolling line graph; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

// Other Library Headers
#include <QColor>   // Qt::Gui
#include <QWidget>  // Qt::Widgets

// Project Headers
//   (none)

/**
 * @brief Plots a rolling window of samples as lines on an invisible grid.
 *        Newest value shown at right, scaled to the window's min/max.
 *
 * @note A NaN sample means "no data" and leaves a gap in its line.
 */
class LineGraphWidget : public QWidget {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    static constexpr double kNoData =  // sample value that leaves a gap
        std::numeric_limits<double>::quiet_NaN();

    explicit LineGraphWidget(QWidget* parent = nullptr);
    ~LineGraphWidget() override = default;

    // Delete copy constructor
    LineGraphWidget(const LineGraphWidget&) = delete;
    LineGraphWidget& operator=(const LineGraphWidget&) = delete;

    // --- Getters & Setters ---

    void SetSampling(double period_s, double history_s);
    void SetWindow(double window_s);

    std::size_t AddSeries(const QColor& color);
    void AddSamples(const std::vector<double>& values);

    [[nodiscard]] std::optional<double> GetWindowMean(std::size_t series) const;

  protected:
    // --- QWidget Overrides ---

    void paintEvent(QPaintEvent* event) override;

  private:
    /**
     * @brief A single plotted line.
     */
    struct Series {
        QColor color;
        std::deque<double> samples;  // oldest first
    };

    // --- Helper Functions ---

    [[nodiscard]] std::size_t GetWindowCount() const;

    // --- Data Members ---

    std::vector<Series> series_;  // data to show
    std::size_t capacity_{1};     // samples per series (i.e., longest window)
    double period_s_{1.0};        // time between samples (s)
    double window_s_{1.0};        // time span (s)
};
