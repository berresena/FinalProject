#include "pcl_qt_viewer/pcl_qt_viewer.h"
#include "pcl_qt_viewer/subscribe_pointcloud2message.h"
#include "pcl_qt_viewer/pointcloudglwidget.h"
#include "pcl_qt_viewer/vfh_planner.h"
#include "pcl_qt_viewer/navigation_manager.h"
#include <QMessageBox>
#include <QMetaObject>
#include <QMetaType>
#include <QDebug>

PCLViewer::PCLViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    nh_(""),
    mapping_active_(false),
    obstacle_detected_(false),
    avoidance_angle_(0.0)
{
    //qDebug() << "PCLViewer başlatılıyor...";

    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>("pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr");

    ui->setupUi(this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/odom", 10, &PCLViewer::odomCallback, this);

    ui->pointSizeSlider->setRange(1, 20);
    ui->pointSizeSlider->setValue(5);
    ui->statusLabel->setText("Sistem Hazır");

    PointCloudGLWidget* pointCloudGLWidget = new PointCloudGLWidget(this);
    pc_subscriber_ = std::make_unique<SubscribePointCloud2Message>(pointCloudGLWidget, this);
    vfh_planner_ = std::make_unique<VFHPlanner>();
    coverage_planner_ = std::make_unique<CoveragePlanner>();
    navigation_manager_ = std::make_unique<NavigationManager>(nh_);
    mapping_manager_ = std::make_unique<MappingManager>(0.05f);

    initializeNavigationPlanners();

    move_timer_ = new QTimer(this);
    connect(move_timer_, &QTimer::timeout, this, &PCLViewer::moveRobot);

    connect(ui->startButton, &QPushButton::clicked, this, &PCLViewer::onStartClicked);
    connect(ui->stopButton, &QPushButton::clicked, this, &PCLViewer::onStopClicked);
    connect(pc_subscriber_.get(), &SubscribePointCloud2Message::newPointCloud, this, &PCLViewer::setCurrentCloud);
    connect(ui->pointSizeSlider, &QSlider::valueChanged, this, [this](int value){
        pc_subscriber_->setPointSize(value);
        ui->pointSizeLabel->setText(QString("Nokta Boyutu: %1").arg(value));
    });

    image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &PCLViewer::imageCallback, this);
    ui->quickWidget->setScaledContents(true);

    //qDebug() << "PCLViewer başarıyla başlatıldı.";
}

void PCLViewer::initializeNavigationPlanners() {
    //qDebug() << "Navigasyon planlayıcıları başlatılıyor...";
    vfh_planner_ = std::make_unique<VFHPlanner>();

    connect(vfh_planner_.get(), &VFHPlanner::obstaclesDetected,
            this, &PCLViewer::updateObstacles);
    connect(vfh_planner_.get(), &VFHPlanner::avoidanceDirection,
            this, &PCLViewer::updateAvoidanceDirection);

    //qDebug() << "VFH bağlantıları yapıldı.";
}

void PCLViewer::setCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    current_cloud_ = cloud;
    //qDebug() << "Yeni point cloud alındı. Nokta sayısı:" << cloud->size();

    if (mapping_active_ && mapping_manager_) {
        //qDebug() << "Haritalama aktif, cloud eklendi.";
        mapping_manager_->addCloud(cloud);
    }

    if (navigation_manager_) {
        //qDebug() << "Navigasyon yöneticisine cloud gönderiliyor.";
        navigation_manager_->updatePointCloud(cloud);
    }
}

void PCLViewer::onStartClicked() {
    //qDebug() << "[START] Başlat butonuna basıldı.";
    pc_subscriber_->start();
    mapping_active_ = true;
    navigation_manager_->setNavigationActive(true);
    move_timer_->start(100);
    ui->statusLabel->setText("Haritalama Aktif | Engel Tanıma Çalışıyor");
}

void PCLViewer::onStopClicked() {
    //qDebug() << "[STOP] Durdur butonuna basıldı.";
    pc_subscriber_->stop();
    mapping_active_ = false;
    navigation_manager_->setNavigationActive(false);
    move_timer_->stop();
    emergencyStop();
    ui->statusLabel->setText("Sistem Durduruldu");

    //qDebug() << "Harita kaydediliyor...";
    mapping_manager_->saveToFile("final_map.pcd");
}

void PCLViewer::moveRobot()
{
    //qDebug() << "[MOVE] moveRobot() çağrıldı.";

    if (!navigation_manager_) {
        //qDebug() << "navigation_manager_ geçersiz (nullptr).";
        return;
    }

    navigation_manager_->update();
}

void PCLViewer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //qDebug() << "[ODOM] Odom verisi geldi.";

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    navigation_manager_->setCurrentYaw(static_cast<float>(yaw));

    QString positionText = QString("X: %1, Y: %2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    QMetaObject::invokeMethod(ui->konumLabel, "setText", Qt::QueuedConnection, Q_ARG(QString, positionText));
}

void PCLViewer::updateObstacles(bool detected) {
    obstacle_detected_ = detected;
    //qDebug() << "[OBSTACLE] Engel durumu güncellendi:" << detected;
}

void PCLViewer::updateAvoidanceDirection(double angle) {
    avoidance_angle_ = angle;
    //qDebug() << "[AVOID] Kaçınma açısı güncellendi:" << angle;
}

void PCLViewer::emergencyStop() {
    //qDebug() << "[EMERGENCY STOP] Robot durduruluyor.";
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.angular.z = 0;
    cmd_vel_pub_.publish(cmd_msg);
}

QImage PCLViewer::convertSensorImageToQImage(const sensor_msgs::ImageConstPtr& msg)
{
    return QImage(&msg->data[0], msg->width, msg->height, msg->step, QImage::Format_RGB888).copy();
}

void PCLViewer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    QImage qimage = convertSensorImageToQImage(msg);
    if (!qimage.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(qimage);
        QMetaObject::invokeMethod(this, [this, pixmap]() {
            ui->quickWidget->setPixmap(pixmap.scaled(
                ui->quickWidget->width(),
                ui->quickWidget->height(),
                Qt::KeepAspectRatio
            ));
        }, Qt::QueuedConnection);
    } else {
        //qDebug() << "QImage dönüşümü başarısız.";
    }
}

PCLViewer::~PCLViewer() {
    delete ui;
}
