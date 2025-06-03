#include "pcl_qt_viewer/navigation_manager.h"
#include <cmath>
#include <QDebug>

NavigationManager::NavigationManager(ros::NodeHandle& nh)
    : nh_(nh),
      current_yaw_(0.0),
      obstacle_detected_(false),
      avoidance_angle_(0.0),
      navigation_active_(false)
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // VFH sinyallerini alıyoruz
    QObject::connect(&vfh_, &VFHPlanner::obstaclesDetected,
                     [&](bool detected) {
                         obstacle_detected_ = detected;
                     });

    QObject::connect(&vfh_, &VFHPlanner::avoidanceDirection,
                     [&](float angle) {
                         avoidance_angle_ = angle;
                     });
    QObject::connect(&vfh_, &VFHPlanner::criticalObstacleDetected,
                     [&]() {
                         sendVelocity(0.0, 0.0);
                     });
}

void NavigationManager::update()
{
    if (!navigation_active_)
    {
        //qDebug() << "Navigasyon aktif değil" << navigation_active_;
        return; 
    } // Navigasyon aktif değilse hiçbir şey yapma
    //qDebug() << "Navigasyon aktif" << navigation_active_;
    if (obstacle_detected_) {
        qDebug() << "engel!!!";
        coverage_.stopPathPlanning(); 
        float turn_speed = 0.35; // radyan/saniye
        float linear_speed = 0.0;

        if (std::abs(avoidance_angle_) > 5.0) {
            float angular = (avoidance_angle_ > 0) ? turn_speed : -turn_speed;
            sendVelocity(linear_speed, angular);
        } else {
            sendVelocity(0.1, 0.0);
        }
    } else {
        qDebug() << "dewamke!!!";
        coverage_.startPathPlanning();
        float next_yaw = coverage_.getNextYaw();
        float yaw_error = next_yaw - current_yaw_;

        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        float angular_speed = 0.2 * yaw_error;
        float linear_speed = 0.15;

        sendVelocity(linear_speed, angular_speed);
    }
}


void NavigationManager::sendVelocity(float linear, float angular)
{
    //qDebug() << "[SEND VELOCITY] Linear:" << linear << "Angular:" << angular;

    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_.publish(cmd);
}
void NavigationManager::setCurrentYaw(float yaw) {
    //qDebug() << "[YAW SET] Yaw güncellendi:" << yaw;
    current_yaw_ = yaw;
}
void NavigationManager::setNavigationActive(bool active) {
    navigation_active_ = active;
    //qDebug() << "Navigation active set to:" << navigation_active_;
}
bool NavigationManager::isObstacleDetected() const {
    return obstacle_detected_;
}
void NavigationManager::updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    vfh_.processCloud(cloud);
}
