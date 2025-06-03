#include "pcl_qt_viewer/subscribe_pointcloud2message.h"
#include "pcl_qt_viewer/pointcloudglwidget.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "pcl_qt_viewer/mapping_manager.h"

SubscribePointCloud2Message::SubscribePointCloud2Message(PointCloudGLWidget* widget, QObject* parent)
    : QObject(parent),
      gl_widget_(widget),
      point_size_(5),
      running_(false),
      tf_listener_(tf_buffer_),
      latest_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    if (!gl_widget_) {
        ROS_ERROR("[PCL] OpenGL Widget is null!");
        return;
    }

    ros::NodeHandle nh;
    std::string topic = "/camera/depth/points";
    
    try {
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            topic, 
            1, 
            &SubscribePointCloud2Message::cloudCallback, 
            this
        );
        if (sub_) {
            ROS_INFO("[PCL] Successfully subscribed to topic: %s", topic.c_str());
            qDebug() << "Point cloud topic'a abone olundu:" << topic.c_str();
        } else {
            ROS_ERROR("[PCL] Failed to subscribe to topic: %s", topic.c_str());
            qDebug() << "Topic'a abone olunamadı:" << topic.c_str();
        }
        mapping_manager_ = std::make_shared<MappingManager>(); // 5cm voxel

    } catch (const std::exception& e) {
        ROS_ERROR("[PCL] Exception while subscribing to topic %s: %s", topic.c_str(), e.what());
        qDebug() << "Topic'a abone olunurken hata oluştu:" << topic.c_str() << "Hata:" << e.what();
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr SubscribePointCloud2Message::getLatestCloud() const
{
    return latest_cloud_;
}

void SubscribePointCloud2Message::setPointSize(int size) {
    point_size_ = std::max(1, size);
    if (gl_widget_) gl_widget_->setPointSize(point_size_);
}

void SubscribePointCloud2Message::start() {
    running_ = true;
}

void SubscribePointCloud2Message::stop() {
    running_ = false;
}

void SubscribePointCloud2Message::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!running_ || !gl_widget_) return;

    std::string target_frame = "base_footprint";
    std::string source_frame = msg->header.frame_id;  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    ros::Time now = ros::Time::now();

    try 
    {
        if (!tf_buffer_.canTransform(target_frame, source_frame, now, ros::Duration(0.1))) {
            ROS_WARN_THROTTLE(1.0, "[PCL] Waiting for transform from %s to %s", source_frame.c_str(), target_frame.c_str());
            qDebug() << "Transform bekleniyor:" << source_frame.c_str() << "->" << target_frame.c_str();
            
            // List available transforms
            std::vector<std::string> frames;
            tf_buffer_._getFrameStrings(frames);
            qDebug() << "Mevcut frame'ler:";
            for (const auto& frame : frames) {
                qDebug() << "-" << frame.c_str();
            }            
            latest_cloud_ = cloud;
            QMetaObject::invokeMethod(gl_widget_, [this, cloud]() {
                gl_widget_->updateCloud(cloud);
            }, Qt::QueuedConnection);
            return;
        }

        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, now);
        for (auto& point : *cloud) {
            geometry_msgs::PointStamped point_in;
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;
            point_in.header.frame_id = source_frame;
            point_in.header.stamp = now;

            geometry_msgs::PointStamped point_out;
            tf_buffer_.transform(point_in, point_out, target_frame);

            point.x = point_out.point.x;
            point.y = point_out.point.y;
            point.z = point_out.point.z;
        }
        
        latest_cloud_ = cloud;
        
        //qDebug() << "Point cloud dönüştürüldü, yeni boyut:" << cloud->size();
        
        QMetaObject::invokeMethod(gl_widget_, [this, cloud]() {
            gl_widget_->updateCloud(cloud);
        }, Qt::QueuedConnection);

        if (mapping_manager_ && latest_cloud_) {
            if (mapping_manager_->isNewCloud(latest_cloud_)) {
                mapping_manager_->addCloud(latest_cloud_);
            }
        }
        emit newPointCloud(cloud);
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PCL] Transform error: %s", ex.what());
        qDebug() << "Transform hatası:" << ex.what();
        
        std::vector<std::string> frames;
        tf_buffer_._getFrameStrings(frames);
        qDebug() << "Mevcut frame'ler:";
        for (const auto& frame : frames) {
            qDebug() << "-" << frame.c_str();
        }

        latest_cloud_ = cloud;
        QMetaObject::invokeMethod(gl_widget_, [this, cloud]() {
            gl_widget_->updateCloud(cloud);
        }, Qt::QueuedConnection);
    }
}