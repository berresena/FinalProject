/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *mainGridLayout;
    QOpenGLWidget *openGLWidget;
    QGroupBox *controlGroupBox;
    QVBoxLayout *verticalLayout;
    QPushButton *startButton;
    QPushButton *stopButton;
    QLabel *statusLabel;
    QLabel *konumLabel;
    QLabel *pointSizeLabel;
    QSlider *pointSizeSlider;
    QGroupBox *cameraGroupBox;
    QVBoxLayout *verticalLayout_2;
    QLabel *quickWidget;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(960, 810);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        mainGridLayout = new QGridLayout(centralwidget);
        mainGridLayout->setObjectName(QString::fromUtf8("mainGridLayout"));
        openGLWidget = new QOpenGLWidget(centralwidget);
        openGLWidget->setObjectName(QString::fromUtf8("openGLWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(openGLWidget->sizePolicy().hasHeightForWidth());
        openGLWidget->setSizePolicy(sizePolicy);
        openGLWidget->setMinimumSize(QSize(640, 480));
        openGLWidget->setStyleSheet(QString::fromUtf8("background-color: black;"));

        mainGridLayout->addWidget(openGLWidget, 0, 0, 1, 2);

        controlGroupBox = new QGroupBox(centralwidget);
        controlGroupBox->setObjectName(QString::fromUtf8("controlGroupBox"));
        verticalLayout = new QVBoxLayout(controlGroupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        startButton = new QPushButton(controlGroupBox);
        startButton->setObjectName(QString::fromUtf8("startButton"));

        verticalLayout->addWidget(startButton);

        stopButton = new QPushButton(controlGroupBox);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));

        verticalLayout->addWidget(stopButton);

        statusLabel = new QLabel(controlGroupBox);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));

        verticalLayout->addWidget(statusLabel);

        konumLabel = new QLabel(controlGroupBox);
        konumLabel->setObjectName(QString::fromUtf8("konumLabel"));

        verticalLayout->addWidget(konumLabel);

        pointSizeLabel = new QLabel(controlGroupBox);
        pointSizeLabel->setObjectName(QString::fromUtf8("pointSizeLabel"));

        verticalLayout->addWidget(pointSizeLabel);

        pointSizeSlider = new QSlider(controlGroupBox);
        pointSizeSlider->setObjectName(QString::fromUtf8("pointSizeSlider"));
        pointSizeSlider->setMinimum(1);
        pointSizeSlider->setMaximum(20);
        pointSizeSlider->setValue(5);
        pointSizeSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(pointSizeSlider);


        mainGridLayout->addWidget(controlGroupBox, 1, 0, 1, 1);

        cameraGroupBox = new QGroupBox(centralwidget);
        cameraGroupBox->setObjectName(QString::fromUtf8("cameraGroupBox"));
        verticalLayout_2 = new QVBoxLayout(cameraGroupBox);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        quickWidget = new QLabel(cameraGroupBox);
        quickWidget->setObjectName(QString::fromUtf8("quickWidget"));

        verticalLayout_2->addWidget(quickWidget);


        mainGridLayout->addWidget(cameraGroupBox, 1, 1, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "QT PCL Haritalama Aray\303\274z\303\274", nullptr));
        startButton->setText(QApplication::translate("MainWindow", "Haritalamay\304\261 Ba\305\237lat", nullptr));
        stopButton->setText(QApplication::translate("MainWindow", "Haritalamay\304\261 Durdur", nullptr));
        statusLabel->setText(QApplication::translate("MainWindow", "Haz\304\261r | Robot Modeli: Waffle Pi | Haritalama: Pasif", nullptr));
        konumLabel->setText(QApplication::translate("MainWindow", "Robotunuzun koordinatlar\304\261: ", nullptr));
        pointSizeLabel->setText(QApplication::translate("MainWindow", "Nokta Boyutu: 5", nullptr));
        cameraGroupBox->setTitle(QApplication::translate("MainWindow", "Kamera G\303\266r\303\274nt\303\274s\303\274", nullptr));
        quickWidget->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
