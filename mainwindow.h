#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "robot.h"
#include <thread>
#include <condition_variable>
#include <mutex>
#include "QRos.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    struct info_obs
    {
        int num_cobs;
        int num_lobs;
        int num_quark;
        int num_path;
        int num_optimized;
        int num_optimized_path;
    };
    struct info_sensor
    {
        int    num_sensors;
        double max_dist;
    };
    struct sensor_data
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> q;
        std::vector<double> dist;
        std::vector<double> vx;
        std::vector<double> vy;
    };
    struct cobs_data
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> radius;
    };
    struct lobs_data
    {
        std::vector<double> x1;
        std::vector<double> y1;
        std::vector<double> x2;
        std::vector<double> y2;
    };
    struct quark_data
    {
        std::vector<double> x;
        std::vector<double> y;
    };
    struct path_data
    {
        std::vector<double> px;
        std::vector<double> py;
    };
    struct optimized_data
    {
        std::vector<double> x;
        std::vector<double> y;
    };

    struct robot_data
    {
        double x;
        double y;
        double q;
        double goal_x;
        double goal_y;
        double radius;
        double dt;
    };

    struct control_data
    {
        double v;
        double w;
    };

    struct localmap_data
    {
        int w,h;
        double lPos[3];
        std::vector<BYTE> lData;
    };

    struct robot_pah_data
    {
        int iSize;
        std::vector<double> x;
        std::vector<double> y;
    };

    struct particle_data
    {
        int iSize;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> w;
    };

    MainWindow(int argc, char** argv,QWidget *parent = nullptr);
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void startRobot(Robot&);
    void start();

    QImage *image;
    QRos<geometry_msgs::Twist>  tout;    // out
    static void onROSVel(void*);
private slots:
    void slotTimeout();
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer timer;
    Robot r;
    std::thread m_worker;
    std::mutex  m_mutex;
    bool m_bRunning;
    bool bReady;

    void worker(Robot&);
    void stopWorker();

    void updateRobotData(std::vector<int> iData, std::vector<double> dData);
    void updateUi();

    info_obs m_obs;
    info_sensor m_sensor;
    sensor_data m_sensor_data;
    path_data m_path_data;
    optimized_data m_optimized_data;
    robot_data m_robot_data;
    control_data m_control_data;
    localmap_data m_localmap_data;
    robot_pah_data m_robot_path;
    particle_data m_particle;

    int m_iArrived;
    double x_range;
};
#endif // MAINWINDOW_H
