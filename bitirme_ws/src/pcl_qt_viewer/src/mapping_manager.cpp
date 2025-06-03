#include "pcl_qt_viewer/mapping_manager.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h> // removeNaNFromPointCloud için
#include <pcl/common/common.h> 

MappingManager::MappingManager(float voxel_leaf_size)
    : voxel_leaf_size_(voxel_leaf_size), cloud_similarity_threshold_(0.95f) {}

void MappingManager::addCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_const_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    *non_const_cloud = *cloud;

    // NaN ve Inf noktaları temizle
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*non_const_cloud, *non_const_cloud, indices);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(non_const_cloud);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.filter(*filtered);

    cloud_history_.push_back(filtered);
}

bool MappingManager::isNewCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_cloud) {
    if (cloud_history_.empty()) return true;

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr last_cloud = cloud_history_.back();
    return !areCloudsSimilar(last_cloud, new_cloud);
}

bool MappingManager::areCloudsSimilar(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& c1,
                                      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& c2) {
    if (c1->empty() || c2->empty()) return false;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(c1);

    float sum_distances = 0.0f;
    int count = 0;
    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    for (const auto& point : c2->points) {
        if (!pcl::isFinite(point)) continue; // geçerli değilse atla

        if (kdtree.nearestKSearch(point, 1, indices, sqr_distances) > 0) {
            sum_distances += std::sqrt(sqr_distances[0]);
            count++;
        }
    }

    if (count == 0) return false;

    float avg_distance = sum_distances / count;
    const float similarity_threshold_distance = 0.1f;

    return avg_distance < similarity_threshold_distance;
}

void MappingManager::saveToFile(const std::string& filename) {
    // Tüm point cloud'ları birleştir
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& c : cloud_history_) {
        if (c && !c->empty()) {
            *all += *c;
        }
    }

    if (all->empty()) {
        qWarning() << "[MappingManager] Uyarı: Kaydedilecek veri bulunamadı.";
        return;
    }

    // ROS package içindeki maps klasörünü belirle
    QString baseDir = QDir::cleanPath(QDir(QCoreApplication::applicationDirPath()).filePath("../../src/pcl_qt_viewer/maps"));
    QDir dir(baseDir);
    if (!dir.exists()) {
        dir.mkpath(".");
    }

    // Dosya adını oluştur
    QString finalFileName = QString::fromStdString(filename);
    if (!finalFileName.endsWith(".pcd")) {
        finalFileName += ".pcd";
    }

    QString fullPath = dir.filePath(finalFileName);

    // Kaydet
    if (pcl::io::savePCDFileBinary(fullPath.toStdString(), *all) == 0) {
        qDebug() << "[MappingManager] Harita başarıyla kaydedildi:" << fullPath;
    } else {
        qCritical() << "[MappingManager] HATA: .pcd dosyası kaydedilemedi!";
    }
}