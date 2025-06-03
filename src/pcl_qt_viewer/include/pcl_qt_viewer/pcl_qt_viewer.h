#ifndef PCL_QT_VIEWER_H
#define PCL_QT_VIEWER_H

#include <QMainWindow>
#include <QTimer>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "pcl_qt_viewer/subscribe_pointcloud2message.h"
#include "pcl_qt_viewer/vfh_planner.h"
#include "pcl_qt_viewer/coverage_planner.h"
#include "pcl_qt_viewer/ui_main_window.h"
#include "pcl_qt_viewer/mapping_manager.h"
#include "pcl_qt_viewer/navigation_manager.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer(QWidget *parent = nullptr);
    ~PCLViewer();

private slots:
    void onStartClicked();
    void onStopClicked();
    void moveRobot();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void updateObstacles(bool detected);
    void updateAvoidanceDirection(double angle);

private:
    void initializeNavigationPlanners();
    void emergencyStop();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void setCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
private:
    Ui::MainWindow *ui;

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    std::unique_ptr<SubscribePointCloud2Message> pc_subscriber_;
    std::unique_ptr<VFHPlanner> vfh_planner_;
    std::unique_ptr<CoveragePlanner> coverage_planner_;
    std::unique_ptr<NavigationManager> navigation_manager_;
    std::unique_ptr<MappingManager> mapping_manager_;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr current_cloud_;

    ros::Subscriber image_sub_;
    QImage convertSensorImageToQImage(const sensor_msgs::ImageConstPtr& msg);
    QImage current_image_;
    QTimer* move_timer_;


    bool mapping_active_;
    bool obstacle_detected_;
    double avoidance_angle_;
    
};

#endif // PCL_QT_VIEWER_H
