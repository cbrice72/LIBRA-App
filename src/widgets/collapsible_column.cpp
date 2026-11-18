/******************************************************************************
 * @file   collapsible_column.cpp
 * @brief  Column with a clickable header that shows/hides its content;
 *         implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "collapsible_column.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QLayout>       // Qt::Widgets
#include <QMouseEvent>   // Qt::Gui
#include <QPainter>      // Qt::Gui
#include <QStyleOption>  // Qt::Widgets

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Getters & Setters
 * !QWidget Overrides
 * !Helper Functions
 */

constexpr int kHeaderPadding = 4;  // around the arrow and title (px)

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 */
CollapsibleColumn::CollapsibleColumn(QWidget* parent) : QWidget(parent) {
    // Reserve the header strip
    header_width_ = fontMetrics().height() + (2 * kHeaderPadding);
    setContentsMargins(header_width_, 0, 0, 0);
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Gets the title shown (vertically) in the header.
 */
QString CollapsibleColumn::GetTitle() const {
    return title_;
}

/**
 * @brief Sets the title shown (vertically) in the header.
 *
 * @param title Header title
 */
void CollapsibleColumn::SetTitle(const QString& title) {
    title_ = title;
    update();
}

/**
 * @brief Whether the content is shown.
 */
bool CollapsibleColumn::IsExpanded() const {
    return expanded_;
}

/**
 * @brief Shows or hides the content.
 *
 * @param expanded Whether to show the content
 */
void CollapsibleColumn::SetExpanded(const bool& expanded) {
    if (expanded == expanded_) {
        return;
    }
    expanded_ = expanded;

    // Measure while shown (hidden widgets don't count toward layout)
    int content_width = GetContentWidth();
    for (auto* child :
         findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly)) {
        child->setVisible(expanded);
    }
    if (expanded) {
        content_width = GetContentWidth();
    }

    ResizeWindow(expanded ? content_width : -content_width);
    update();  // arrow direction
}

//------------------------------------------------------------------------------
// !QWidget Overrides
//------------------------------------------------------------------------------

/**
 * @brief Overrides `mousePressEvent()` so clicking the header toggles the
 *        content.
 */
void CollapsibleColumn::mousePressEvent(QMouseEvent* event) {
    if (event->position().x() < header_width_) {
        SetExpanded(!expanded_);
        return;
    }

    QWidget::mousePressEvent(event);
}

/**
 * @brief Overrides `paintEvent()` to draw the header.
 */
void CollapsibleColumn::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);

    // Arrow next to header points left when expanded, right when collapsed
    QStyleOption option;
    option.initFrom(this);
    option.rect = QRect(0, 0, header_width_, header_width_);
    style()->drawPrimitive(expanded_ ? QStyle::PE_IndicatorArrowLeft
                                     : QStyle::PE_IndicatorArrowRight,
                           &option, &painter, this);

    // Vertical title (rotated 90 deg CCW)
    const int title_length = fontMetrics().horizontalAdvance(title_);

    painter.save();
    painter.translate(0, header_width_ + title_length);
    painter.rotate(-90);
    painter.setPen(palette().color(QPalette::WindowText));
    painter.drawText(QRect(0, 0, title_length, header_width_), Qt::AlignCenter,
                     title_);
    painter.restore();

    // Line from title to bottom edge denotes clickable header area
    const int line_x = header_width_ / 2;
    const int line_top = header_width_ + title_length + kHeaderPadding;

    painter.setPen(palette().color(QPalette::Mid));
    painter.drawLine(line_x, line_top, line_x, height() - kHeaderPadding);
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Minimum width requested by the content's layout element.
 */
int CollapsibleColumn::GetContentWidth() const {
    if (layout() == nullptr) {
        return 0;
    }

    return layout()->sizeHint().width();
}

/**
 * @brief Widens or narrows the window, unless it isn't shown yet or the
 *        screen sets its size (maximized/full screen).
 *
 * @param delta Width change (px)
 */
void CollapsibleColumn::ResizeWindow(const int& delta) {
    QWidget* win = window();
    if (win->isHidden()) {
        return;
    }
    if ((win->windowState() & (Qt::WindowMaximized | Qt::WindowFullScreen))
        != 0) {
        return;
    }

    win->resize(win->width() + delta, win->height());
}
