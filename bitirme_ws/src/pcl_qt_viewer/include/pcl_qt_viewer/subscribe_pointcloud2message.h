#ifndef SUBSCRIBE_POINTCLOUD2MESSAGE_H
#define SUBSCRIBE_POINTCLOUD2MESSAGE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <QObject>
#include <QColor>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <QDebug>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_qt_viewer/mapping_manager.h"

class PointCloudGLWidget;

class SubscribePointCloud2Message : public QObject
{
    Q_OBJECT
public:
    SubscribePointCloud2Message(PointCloudGLWidget* widget, QObject* parent = nullptr);
    void setPointSize(int size);
    void start();
    void stop();
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getLatestCloud() const;
signals:
    void newPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

private slots:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    PointCloudGLWidget* gl_widget_;
    int point_size_;
    bool running_;
    ros::Subscriber sub_;
    std::shared_ptr<MappingManager> mapping_manager_;
    tf2_ros::Buffer tf_buffer_; 
    tf2_ros::TransformListener tf_listener_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr latest_cloud_;
};

#endif // SUBSCRIBE_POINTCLOUD2MESSAGE_H