#include "pcl_qt_viewer/vfh_planner.h"
#include <cmath>
#include <QDebug>
#include <limits>

VFHPlanner::VFHPlanner(QObject* parent)
    : QObject(parent)
{
}

void VFHPlanner::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    if (!cloud || cloud->empty()) {
        emit obstaclesDetected(false);
        return;
    }
    const float front_angle = 60.0f;           // Ön taraftaki açı aralığı (derece)
    const float min_safe_distance = 1.0f;     // Güvenli mesafe sınırı (metre)
    const float critical_distance = 0.5f;    // Kritik mesafe

    bool obstacle_detected = false;
    int obstacle_count = 0;
    float total_angle = 0.0f;
    float closest_distance = std::numeric_limits<float>::max();
    pcl::PointXYZRGB closest_point;

    for (const auto& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        float angle = std::atan2(point.y, point.x) * 180.0f / M_PI; // Derece cinsinden

        if (std::abs(angle) <= front_angle) 
        {
            if (distance < min_safe_distance) 
            {
                obstacle_detected = true;
                obstacle_count++;
                total_angle += angle;
                //qDebug() << "engel sayısı:" << obstacle_count;
                if (distance < closest_distance) 
                {
                    closest_distance = distance;
                    closest_point = point;
                }
                if (distance < critical_distance) 
                {
                    emit criticalObstacleDetected();
                    return;
                }
            }
        }
    }

    emit obstaclesDetected(obstacle_detected);

    if (obstacle_detected && obstacle_count > 0) 
    {
        float avg_angle = total_angle / obstacle_count;
        while (avg_angle > 180) avg_angle -= 360;
        while (avg_angle < -180) avg_angle += 360;
        qDebug() << "KAÇINMA AÇISI: " <<avg_angle;
        emit avoidanceDirection(avg_angle);
    }
}
