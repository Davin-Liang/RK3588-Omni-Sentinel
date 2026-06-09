#include "top_down_view.h"
#include "lidar_tracking_types.h"

#include <QPainter>
#include <QPaintEvent>
#include <QtMath>
#include <cstdio>

TopDownView::TopDownView(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 200);
    setStyleSheet("background-color: #1a1a2e; border: 1px solid #30363d; border-radius: 8px;");
}

void TopDownView::set_targets(const QVector<TrackedTarget>& targets)
{
    targets_ = targets;
    ++frameCount_;
}

void TopDownView::set_range_meters(float range)
{
    if (range > 0.0f && range <= 100.0f) {
        rangeMeters_ = range;
        update();
    }
}

void TopDownView::set_show_grid(bool show)
{
    showGrid_ = show;
    update();
}

// ============================================================================
// paintEvent
// ============================================================================

void TopDownView::paintEvent(QPaintEvent* /*event*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    float w = static_cast<float>(width());
    float h = static_cast<float>(height());
    float centerX = w * 0.5f;
    float centerY = h * 0.5f;
    float viewSize = qMin(w, h) - 40.0f;
    float mPerPx = 2.0f * rangeMeters_ / viewSize;

    QRectF viewRect(centerX - rangeMeters_ / mPerPx,
                    centerY - rangeMeters_ / mPerPx,
                    2.0f * rangeMeters_ / mPerPx,
                    2.0f * rangeMeters_ / mPerPx);

    if (showGrid_) {
        draw_grid_(p, viewRect, mPerPx);
    }

    draw_targets_(p, mPerPx);
    draw_legend_(p);
}

// ============================================================================
// 网格
// ============================================================================

void TopDownView::draw_grid_(QPainter& p, const QRectF& rect, float mPerPx)
{
    float centerX = static_cast<float>(width()) * 0.5f;
    float centerY = static_cast<float>(height()) * 0.5f;

    p.setPen(QPen(QColor(0x2a, 0x2a, 0x4a), 1));

    // 同心距离环: 1m, 2m, 5m, 10m, 15m, 20m
    float rings[] = {1.0f, 2.0f, 5.0f, 10.0f, 15.0f, 20.0f};
    QFont ringFont("Arial", 8);
    p.setFont(ringFont);

    for (float r : rings) {
        if (r > rangeMeters_) break;
        float radius = r / mPerPx;
        p.drawEllipse(QPointF(centerX, centerY), radius, radius);

        // 刻度标注（在 Y 轴上方）
        p.setPen(QPen(QColor(0x4a, 0x4a, 0x6a), 1));
        p.drawText(QPointF(centerX + 3.0f, centerY - radius),
                   QString::number(static_cast<int>(r)) + "m");
        p.setPen(QPen(QColor(0x2a, 0x2a, 0x4a), 1));
    }

    // 十字轴
    p.setPen(QPen(QColor(0x3a, 0x3a, 0x5a), 1, Qt::DashLine));
    p.drawLine(QPointF(centerX, rect.top()), QPointF(centerX, rect.bottom()));
    p.drawLine(QPointF(rect.left(), centerY), QPointF(rect.right(), centerY));
}

// ============================================================================
// 目标
// ============================================================================

void TopDownView::draw_targets_(QPainter& p, float mPerPx)
{
    float centerX = static_cast<float>(width()) * 0.5f;
    float centerY = static_cast<float>(height()) * 0.5f;

    QFont idFont("Arial", 8, QFont::Bold);
    QFont velFont("Arial", 7);

    for (const auto& t : targets_) {
        if (t.state == TrackState::Deleted) continue;

        // 坐标映射: LiDAR X=forward(屏幕↑), LiDAR Y=right(屏幕→)
        float sx = centerX + t.posY / mPerPx;
        float sy = centerY - t.posX / mPerPx;

        // 裁剪：超出视图范围则跳过
        if (sx < -20.0f || sx > static_cast<float>(width()) + 20.0f ||
            sy < -20.0f || sy > static_cast<float>(height()) + 20.0f) {
            continue;
        }

        // 按状态选择颜色
        QColor fillColor, borderColor;
        switch (t.state) {
        case TrackState::Confirmed:
            fillColor   = QColor(0x3f, 0xb9, 0x50);
            borderColor = QColor(0x2e, 0xa0, 0x43);
            break;
        case TrackState::Tentative:
            fillColor   = QColor(0xd2, 0x99, 0x22);
            borderColor = QColor(0xb0, 0x88, 0x00);
            break;
        case TrackState::Coasting:
            fillColor   = QColor(0x8b, 0x94, 0x9e);
            borderColor = QColor(0x6e, 0x76, 0x81);
            break;
        default:
            continue;
        }

        // 告警脉冲圈
        if (t.warningActive) {
            float pulsePhase = static_cast<float>(frameCount_ % 30) / 30.0f;
            float pulseR = kWarningRadius + 4.0f * qSin(pulsePhase * 2.0f * 3.14159f);
            p.setPen(QPen(QColor(0xda, 0x36, 0x33, 180), 2));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(QPointF(sx, sy), pulseR, pulseR);
        }

        // 目标圆
        p.setPen(QPen(borderColor, 2));
        p.setBrush(fillColor);
        p.drawEllipse(QPointF(sx, sy), kTargetRadius, kTargetRadius);

        // 速度箭头
        float velMag = qSqrt(t.velX * t.velX + t.velY * t.velY);
        if (velMag > 0.05f) {
            float arrowLen = qMin(velMag * 4.0f, kMaxArrowLen);
            float vx = t.velY / mPerPx;   // LiDAR Y → screen X
            float vy = -t.velX / mPerPx;  // LiDAR X → screen Y (inverted)
            float vn = qSqrt(vx * vx + vy * vy);
            if (vn > 0.001f) {
                vx = vx / vn * arrowLen;
                vy = vy / vn * arrowLen;
            }
            p.setPen(QPen(QColor(0xff, 0xff, 0xff, 200), 2));
            p.drawLine(QPointF(sx, sy), QPointF(sx + vx, sy + vy));

            // 箭头尖端小三角
            float tipX = sx + vx;
            float tipY = sy + vy;
            float ang = qAtan2(vy, vx);
            float arrowSize = 6.0f;
            QPointF tri[3] = {
                QPointF(tipX, tipY),
                QPointF(tipX - arrowSize * qCos(ang - 0.6f),
                        tipY - arrowSize * qSin(ang - 0.6f)),
                QPointF(tipX - arrowSize * qCos(ang + 0.6f),
                        tipY - arrowSize * qSin(ang + 0.6f))
            };
            p.setBrush(QColor(0xff, 0xff, 0xff, 200));
            p.drawPolygon(tri, 3);
        }

        // ID 标注
        p.setPen(Qt::white);
        p.setFont(idFont);
        QString label = QString("#%1").arg(t.id);
        p.drawText(QPointF(sx - 10.0f, sy - kTargetRadius - 4.0f), label);
    }
}

// ============================================================================
// 图例
// ============================================================================

void TopDownView::draw_legend_(QPainter& p)
{
    float x = static_cast<float>(width()) - 120.0f;
    float y = static_cast<float>(height()) - 95.0f;

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0x1a, 0x1a, 0x2e, 200));
    p.drawRoundedRect(QRectF(x - 6.0f, y - 6.0f, 120.0f, 95.0f), 6.0f, 6.0f);

    QFont font("Arial", 8);
    p.setFont(font);

    struct LegendItem {
        QColor color;
        QString label;
    };
    LegendItem items[] = {
        { QColor(0x3f, 0xb9, 0x50), QString::fromUtf8("已确认") },
        { QColor(0xd2, 0x99, 0x22), QString::fromUtf8("待确认") },
        { QColor(0x8b, 0x94, 0x9e), QString::fromUtf8("外推中") },
        { QColor(0xda, 0x36, 0x33), QString::fromUtf8("告警")   },
    };

    for (int i = 0; i < 4; ++i) {
        float iy = y + static_cast<float>(i) * 20.0f;
        p.setPen(Qt::NoPen);
        p.setBrush(items[i].color);
        p.drawEllipse(QPointF(x + 6.0f, iy + 5.0f), 5.0f, 5.0f);
        p.setPen(QColor(0xe6, 0xed, 0xf3));
        p.drawText(QPointF(x + 16.0f, iy + 10.0f), items[i].label);
    }
}
