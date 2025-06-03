#ifndef POINTCLOUDGLWIDGET_H
#define POINTCLOUDGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit PointCloudGLWidget(QWidget *parent = nullptr);
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void setPointSize(float size);


protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

private:
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
    float point_size_;
};

#endif // POINTCLOUDGLWIDGET_H
