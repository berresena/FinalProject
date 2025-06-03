#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <QDir>
#include <QDateTime>
#include <QDebug>
#include <QCoreApplication>
#include <pcl/io/pcd_io.h>

class MappingManager {
public:
    MappingManager(float voxel_leaf_size = 0.05f);
    
    void addCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
    void saveToFile(const std::string& filename);
    bool isNewCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_cloud);

private:
    float voxel_leaf_size_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud_history_;
    float cloud_similarity_threshold_;

    bool areCloudsSimilar(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& c1,
                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& c2);
};
