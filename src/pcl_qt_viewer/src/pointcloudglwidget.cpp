#include "pcl_qt_viewer/pointcloudglwidget.h"
#include <QOpenGLShaderProgram>
#include <QDebug>

PointCloudGLWidget::PointCloudGLWidget(QWidget *parent) 
    : QOpenGLWidget(parent) {
    setMinimumSize(640, 480);
    QSizePolicy policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    policy.setHorizontalStretch(1);
    policy.setVerticalStretch(1);
    setSizePolicy(policy);
}

void PointCloudGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void PointCloudGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = float(width()) / float(height());
    glOrtho(-5.0 * aspect, 5.0 * aspect, 5.0, -5.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPointSize(point_size_);
    glBegin(GL_POINTS);
    if (cloud_) {
        for(const auto& point : *cloud_) {
            uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(&point.rgb);
            float r = ((rgb_val >> 16) & 0xff) / 255.0f;
            float g = ((rgb_val >> 8) & 0xff) / 255.0f;
            float b = (rgb_val & 0xff) / 255.0f;
            glColor3f(r, g, b);
            glVertex2f(point.x, point.y);
        }
    }
    glEnd();
}

void PointCloudGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void PointCloudGLWidget::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {    
    cloud_ = cloud;
    update();
}

void PointCloudGLWidget::setPointSize(float size) {
    point_size_ = size;
    update();
}