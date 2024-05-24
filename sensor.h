#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#define SIZE_STATE 3

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2

#define SIZE_RECT  4

#define LEFT_TOP_X 0
#define LEFT_TOP_Y 1
#define RIGHT_BOTTOM_X 2
#define RIGHT_BOTTOM_Y 3

//#define BYTE unsigned char
#include "stdafx.h"
class Sensor
{
public:
    struct info_param
    {
        struct sensor_param
        {
            int num_sensors;
            double max_dist;
            double radius;//robot radius
        };
        struct quark_param
        {
            int max_quark;
        };
        sensor_param sparam;
        quark_param qparam;
    };
    struct info_quark
    {
        struct quark_pos
        {
            double x,y;
        };
        struct quark_sense
        {
            double dist;
            double vx,vy;
        };
        quark_pos pos;
        quark_sense sense;
    };
    struct info_sensor
    {
        struct sensor_pos
        {
            double x,y,q;
        };
        struct sensor_sense
        {
            double dist;
            double vx,vy;
        };
        sensor_pos pos;
        sensor_sense sense;
    };
    Sensor(int num_sensors, double max_dist, double radius, int max_quark);
    Sensor(const Sensor& sen);
    ~Sensor();

    void normalizeAngle(double angle, double &dst);
    void updateSensorPos();
    void setMap(std::vector<BYTE> map,double *src,int w, int h, double resolution);
    void M2C(std::vector<BYTE> map);//change map data for calculate
    void M2P(double x, double y, int& px, int& py);
    void P2M(double x, double y, double& mx, double& my);
    bool senseCell(int x, int y);
    void senseMap(int sensor);
    bool senseMap();

    void sense(double *pos);

    double m_rPos[SIZE_STATE];
    double m_mPos[SIZE_STATE];

    info_param  ip;
    //info_sensor *is;
    std::vector<info_sensor> is;
protected:
    std::vector<BYTE> m_map;
    double m_limit[SIZE_RECT];
    double m_resolution;
    int m_w,m_h;
};

#endif // SENSOR_H
