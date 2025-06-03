#ifndef VFH_PLANNER_H
#define VFH_PLANNER_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VFHPlanner : public QObject
{
    Q_OBJECT
public:
    explicit VFHPlanner(QObject* parent = nullptr);    
    void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);// Nokta bulutu işleme fonksiyonu

signals:    
    void obstaclesDetected(bool detected);// Engel algılandı mı?    
    void criticalObstacleDetected();// Kritik mesafe içindeki engel için acil duruş sinyali    
    void avoidanceDirection(float angle);// Engelden kaçınmak için önerilen yön (derece cinsinden)
};

#endif // VFH_PLANNER_H

