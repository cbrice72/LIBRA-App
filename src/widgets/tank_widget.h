/******************************************************************************
 * @file   tank_widget.h
 * @brief  Fluid tank visualization definition file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

#include <QPainter>  // Qt::Gui
#include <QTimer>    // Qt::Core
#include <QWidget>   // Qt::Widgets

/**
 * @brief A simple widget to visualize the fluid level of a tank.
 *
 * @todo TODO(brice.c.aa): Check for correctness.
 */
class TankWidget : public QWidget {
    Q_OBJECT
    Q_PROPERTY(double level READ level WRITE setLevel NOTIFY levelChanged)

  public:
    explicit TankWidget(QWidget* parent = nullptr)
        : QWidget(parent), m_level(0.0), m_animationTimer(new QTimer(this)),
          m_targetLevel(0.0) {
        setMinimumSize(100, 200);
        setSizePolicy(QSizePolicy::MinimumExpanding,
                      QSizePolicy::MinimumExpanding);

        // Setup animation timer
        m_animationTimer->setInterval(16);  // 60 FPS
        connect(m_animationTimer, &QTimer::timeout, this,
                &TankWidget::updateAnimation);
    }

    double level() const {
        return m_level;
    }

  public slots:

    void setLevel(double newLevel) {
        newLevel = qBound(0.0, newLevel, 1.0);
        if (qFuzzyCompare(m_targetLevel, newLevel))
            return;

        m_targetLevel = newLevel;
        if (!m_animationTimer->isActive())
            m_animationTimer->start();

        emit levelChanged(m_level);
    }

  signals:
    void levelChanged(double level);

  protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Draw tank outline
        const int margin = 10;
        const int borderWidth = 4;
        QRectF tankRect = rect().adjusted(margin, margin, -margin, -margin);

        // Draw tank border
        painter.setPen(QPen(Qt::black, borderWidth));
        painter.setBrush(Qt::white);
        painter.drawRoundedRect(tankRect, 10, 10);

        // Draw fluid
        const double fluidHeight = tankRect.height() * m_level;
        QRectF fluidRect = tankRect.adjusted(borderWidth,
                                             tankRect.height() - fluidHeight,
                                             -borderWidth, -borderWidth);

        // Create gradient for fluid
        QLinearGradient gradient(fluidRect.topLeft(), fluidRect.topRight());
        gradient.setColorAt(0, QColor(0, 120, 255, 200));
        gradient.setColorAt(1, QColor(0, 160, 255, 200));

        painter.setPen(Qt::NoPen);
        painter.setBrush(gradient);
        painter.drawRect(fluidRect);

        // Draw level percentage
        painter.setPen(Qt::black);
        QFont font = painter.font();
        font.setPixelSize(16);
        painter.setFont(font);
        painter.drawText(tankRect, Qt::AlignCenter,
                         QString("%1%").arg(qRound(m_level * 100)));

        // Draw level markers
        const int markerWidth = 20;
        const int numMarkers = 10;
        painter.setPen(QPen(Qt::black, 2));
        for (int i = 0; i <= numMarkers; ++i) {
            double y = tankRect.top() + (tankRect.height() * i / numMarkers);
            painter.drawLine(tankRect.left() - margin / 2, y,
                             tankRect.left() + markerWidth, y);

            // Draw marker text
            if (i % 2 == 0) {  // Draw every other marker
                painter.drawText(QRectF(tankRect.left() - margin * 3,
                                        y - margin, margin * 2, margin * 2),
                                 Qt::AlignRight | Qt::AlignVCenter,
                                 QString("%1%").arg(100 - i * 10));
            }
        }
    }

  private slots:

    void updateAnimation() {
        const double step = 0.02;  // Animation speed
        if (m_level < m_targetLevel) {
            m_level = qMin(m_level + step, m_targetLevel);
        } else if (m_level > m_targetLevel) {
            m_level = qMax(m_level - step, m_targetLevel);
        } else {
            m_animationTimer->stop();
        }
        update();
    }

  private:
    double m_level;
    double m_targetLevel;
    QTimer* m_animationTimer;
};
