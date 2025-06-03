#include "pcl_qt_viewer/pcl_qt_viewer.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_qt_viewer");
    QApplication a(argc, argv);
    PCLViewer w;
    w.show();
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return a.exec();
}