#include "pcl_qt_viewer/coverage_planner.h"
#include <QDebug>
#include <QElapsedTimer>

CoveragePlanner::CoveragePlanner(QObject* parent)
    : QObject(parent), current_yaw_(0.0f), increment_(0.1f), is_active_(false),
      width_(0.0), height_(0.0), resolution_(0.1) // Varsayılan çözünürlük
{
    // Sinyal-slot bağlantısı: activeStatusChanged -> calculateBoustrophedon
    connect(this, &CoveragePlanner::activeStatusChanged, this, [this](bool active) {
        if (active) {
            calculateBoustrophedon(width_, height_, resolution_);
        } else {
            m_path.clear();
            emit pathCalculated(m_path); // Boş yolu yayınla
        }
    });
}

void CoveragePlanner::setActive(bool active) {
    if (is_active_ != active) {
        is_active_ = active;
        emit activeStatusChanged(is_active_); // Durum değiştiğinde sinyal yayınla
    }
}

void CoveragePlanner::calculateBoustrophedon(double width, double height, double resolution) {
    qDebug() <<  "coverage calculate!";
    m_path.clear();
    width_ = width;    // Parametreleri sakla
    height_ = height;
    resolution_ = resolution;

    int rows = static_cast<int>(height / resolution);
    int cols = static_cast<int>(width / resolution);

    bool leftToRight = true;
    QElapsedTimer timer;
    timer.start();

    for (int r = 0; r <= rows; ++r) {
        if (!is_active_) break; // Dışarıdan durdurulduysa döngüyü kır

        if (leftToRight) {
            for (int c = 0; c <= cols; ++c) {
                m_path.append(QPointF(c * resolution, r * resolution));
            }
        } else {
            for (int c = cols; c >= 0; --c) {
                m_path.append(QPointF(c * resolution, r * resolution));
            }
        }
        leftToRight = !leftToRight;
    }

    emit pathCalculated(m_path); // Yolu yayınla
}

void CoveragePlanner::stopPathPlanning() {
    setActive(false); // Sinyal otomatik yayınlanır
}

void CoveragePlanner::startPathPlanning() {
    setActive(true); // Sinyal otomatik yayınlanır
}

bool CoveragePlanner::isActive() const {
    return is_active_;
}

float CoveragePlanner::getNextYaw() {
    current_yaw_ += increment_;
    if (current_yaw_ > 6.2831853f) { // 2 * pi
        current_yaw_ = 0.0f;
    }
    return current_yaw_;
}