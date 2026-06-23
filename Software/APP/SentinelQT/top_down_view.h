#ifndef TOP_DOWN_VIEW_H
#define TOP_DOWN_VIEW_H

#include <QWidget>
#include <QVector>
#include <cstdint>

struct TrackedTarget;

class TopDownView : public QWidget {
    Q_OBJECT

public:
    explicit TopDownView(QWidget* parent = nullptr);

    void set_targets(const QVector<TrackedTarget>& targets);
    void set_range_meters(float range);
    void set_show_grid(bool show);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void draw_grid_(QPainter& p, const QRectF& rect, float mPerPx);
    void draw_targets_(QPainter& p, float mPerPx);
    void draw_legend_(QPainter& p);

    QVector<TrackedTarget> targets_;
    float rangeMeters_ = 10.0f;
    bool  showGrid_    = true;
    uint32_t frameCount_ = 0;

    static constexpr float kTargetRadius  = 7.0f;
    static constexpr float kMaxArrowLen   = 22.0f;
    static constexpr float kWarningRadius = 12.0f;
};

#endif // TOP_DOWN_VIEW_H
