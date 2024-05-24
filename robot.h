#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include "controller.h"
#include "QRos.h"
#include "QTF2.h"
#include "3d/dhVector.h"
class Robot
{
public:
    Robot();
    ~Robot();

    void initRos(int argc,char **argv);
    void start();
    void run();
    void stop();

    std::vector<int>    getiData();
    std::vector<double> getdData();
    std::vector<BYTE>   getbData();
    //get robot state
    bool isDataUpdated();
    void setDataUpdated(bool updated);

    Controller con;
    std::condition_variable m_condition;

    QTFListen<geometry_msgs::TransformStamped> tflisten;
    double gPos[3];
    QRos<nav_msgs::OccupancyGrid> lmap;
    static void onROSLocalMap(nav_msgs::OccupancyGrid &msg, void *);
    double lPos[3];
    int l_width, l_height;

private:
    void updateLoop();

    void updateData();

    std::thread m_worker;
    std::mutex m_mutex;
    bool m_running;
    bool m_update;

    std::vector<int> m_iData;
    std::vector<double> m_dData;
    std::vector<BYTE> m_bData;
    std::vector<Generator::path> m_bPath;
};

#endif // ROBOT_H
