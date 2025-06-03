#ifndef PCL_QT_VIEWER_NODE_H
#define PCL_QT_VIEWER_NODE_H

#include <QMainWindow>
#include <QDebug>
#include "pcl_qt_viewer/pointcloudglwidget.h"
#include "pcl_qt_viewer/subscribe_pointcloud2message.h"
#include "pcl_qt_viewer/vfh_planner.h"
#include <pcl/io/pcd_io.h>
#include <QDir>
#include <QDateTime>

class PCLQtViewerNode : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLQtViewerNode(QWidget *parent = nullptr);
    ~PCLQtViewerNode();

private:
    PointCloudGLWidget* gl_widget_;, 
    SubscribePointCloud2Message* subscriber_;
    VFHPlanner* vfh_planner_;
};

#endif // PCL_QT_VIEWER_NODE_H 