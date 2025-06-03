#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include <QObject>
#include <QPointF>
#include <QList>

class CoveragePlanner : public QObject
{
    Q_OBJECT

public:
    explicit CoveragePlanner(QObject* parent = nullptr);
    
    void calculateBoustrophedon(double width, double height, double resolution);
    void stopPathPlanning();
    void startPathPlanning();
    bool isActive() const;
    float getNextYaw();

    void setActive(bool active); // Yeni eklenen fonksiyon

signals:
    void pathCalculated(const QList<QPointF>& path);
    void activeStatusChanged(bool is_active); // Yeni sinyal

private:
    QList<QPointF> m_path;
    float current_yaw_;
    float increment_;
    bool is_active_;
    double width_;       // Yeni: Parametreleri saklamak için
    double height_;
    double resolution_;
};

#endif // COVERAGE_PLANNER_H