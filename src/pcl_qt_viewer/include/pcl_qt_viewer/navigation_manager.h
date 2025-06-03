#ifndef NAVIGATION_MANAGER_H
#define NAVIGATION_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "pcl_qt_viewer/vfh_planner.h"
#include "pcl_qt_viewer/coverage_planner.h"
#include <QMessageBox>
#include <QDebug>
#include <tf/tf.h>  // navigation_manager.cpp veya pcl_qt_viewer_node.cpp başında
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
class NavigationManager
{
public:
    NavigationManager(ros::NodeHandle& nh);

    void update();
    void setCurrentYaw(float yaw);
    bool isObstacleDetected() const;
    void setNavigationActive(bool active);
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    VFHPlanner vfh_;
    CoveragePlanner coverage_;

    float current_yaw_;
    bool obstacle_detected_;
    float avoidance_angle_;        
    bool navigation_active_;       


    
    void sendVelocity(float linear, float angular);
};
#endif // NAVIGATION_MANAGER_H

