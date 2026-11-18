/******************************************************************************
 * @file   collapsible_column.h
 * @brief  Column with a clickable header that shows/hides its content; header
 *         file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QString>  // Qt::Core
#include <QWidget>  // Qt::Widgets

// Project Headers
//   (none)

/**
 * @brief A collapsible content section with a clickable header. Expands to the
 *        right.
 *
 * @note Inspired by MichaelVoelkel's "qt-collapsible-section" project, turned
 *       sideways (and with a few modifications).
 */
class CollapsibleColumn : public QWidget {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

    // Settable from the .ui
    Q_PROPERTY(QString title READ GetTitle WRITE SetTitle)

  public:
    explicit CollapsibleColumn(QWidget* parent = nullptr);
    ~CollapsibleColumn() override = default;

    // Delete copy constructor
    CollapsibleColumn(const CollapsibleColumn&) = delete;
    CollapsibleColumn& operator=(const CollapsibleColumn&) = delete;

    // --- Getters & Setters ---

    [[nodiscard]] QString GetTitle() const;
    void SetTitle(const QString& title);

    [[nodiscard]] bool IsExpanded() const;

  public slots:
    void SetExpanded(const bool& expanded);

  protected:
    // --- QWidget Overrides ---

    void mousePressEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

  private:
    // --- Helper Functions ---

    [[nodiscard]] int GetContentWidth() const;
    void ResizeWindow(const int& delta);

    // --- Data Members ---

    QString title_;
    bool expanded_{true};  // content from the .ui starts visible
    int header_width_{0};  // (px)
};
