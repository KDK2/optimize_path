#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include "controller.h"
class Robot
{
public:
    Robot();
    ~Robot();

    void start();
    void run();
    void stop();
};

#endif // ROBOT_H
