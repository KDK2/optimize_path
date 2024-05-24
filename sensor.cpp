#include "sensor.h"
#include "math.h"

std::vector<double> linspace(double start_in, double end_in, int num_in)
{
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0)
    {
        return linspaced;
    }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }
    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
        // are exactly the same as the input
    return linspaced;
}
Sensor::Sensor(int num_sensors, double max_dist, double radius, int max_quark)
{
    ip.sparam.num_sensors=num_sensors;
    ip.sparam.max_dist=max_dist;
    ip.sparam.radius=radius;

    ip.qparam.max_quark=max_quark;

    //is=new info_sensor[num_sensors];
    std::vector<info_sensor>temp(ip.sparam.num_sensors);
    is=temp;
}

Sensor::Sensor(const Sensor &sen)
{
    ip=sen.ip;
    //is=new info_sensor[ip.sparam.num_sensors];
    std::vector<info_sensor>temp(ip.sparam.num_sensors);
    is=temp;
    m_mPos[INDEX_X]=sen.m_mPos[INDEX_X];
    m_mPos[INDEX_Y]=sen.m_mPos[INDEX_Y];
    m_mPos[INDEX_Q]=sen.m_mPos[INDEX_Q];
    //copy obs vector
    setMap(sen.m_map,m_mPos,sen.m_w,sen.m_h,sen.m_resolution);
}

Sensor::~Sensor()
{
    //delete[] is;
}

void Sensor::normalizeAngle(double angle, double &dst)
{
    angle = fmod(angle, 2.0 * M_PI);

    if(angle>M_PI)
        angle -= 2.0 * M_PI;
    else if(angle<-M_PI)
        angle += 2.0 * M_PI;

    dst=angle;
}

void Sensor::updateSensorPos()
{
    int sensor=ip.sparam.num_sensors;
    double radius=ip.sparam.radius;
    std::vector<double> v_sensor = linspace(0, 2.0*M_PI, sensor+1);
    for(int i=0;i<sensor;i++)
    {
        double q;
        //normalizeAngle(v_sensor[i],q);
        q=v_sensor[i];
        double rx,ry,rq;

        rx=radius*cos(q);
        ry=radius*sin(q);
        rq=q;
        q=m_rPos[INDEX_Q];

        is[i].pos.x=rx*cos(q)-ry*sin(q)+m_rPos[INDEX_X];
        is[i].pos.y=rx*sin(q)+ry*cos(q)+m_rPos[INDEX_Y];
        is[i].pos.q=rq+q;
        //normalizeAngle(rq+q,is[i].pos.q);
    }
}

void Sensor::setMap(std::vector<BYTE> map, double *src, int w, int h, double resolution)
{
    double max_dist=ip.sparam.max_dist;
    m_w=w;
    m_h=h;
    m_resolution=resolution;

    m_mPos[INDEX_X]=src[INDEX_X];
    m_mPos[INDEX_Y]=src[INDEX_Y];
    m_mPos[INDEX_Q]=src[INDEX_Q];

    m_limit[0]=-max_dist;//limit left top x
    m_limit[1]=max_dist;//limit left top y
    m_limit[2]=max_dist;//limit right bottom x
    m_limit[3]=-max_dist;//limit right bottom y

    M2C(map);
}

void Sensor::M2C(std::vector<BYTE> map)//convert pixel map to cartesian map
{
    if(map.empty())
        return;
    int w = m_w;
    int h = m_h;
    int index=w*(h-1);
    std::vector<BYTE> temp(w*h);
    for(int i =0;i<h;i++)
    {
        std::copy(&map[w*i],&map[w*i+w],&temp[index]);
        index-=w;
    }
    m_map=map;
}

void Sensor::M2P(double x, double y, int &px, int &py)//meter to pixel
{
    //x, y is global pos
    int w=m_w;
    int h=m_h;
    double resolution=m_resolution;
    double mx=x/resolution;
    double my=y/resolution;

    px=w/2+mx;
    py=h/2+my;
}

void Sensor::P2M(double x, double y, double &mx, double &my)
{
    int w=m_w;
    int h=m_h;
    double resolution=m_resolution;

    mx=(x-(double)(w/2.0))*resolution;
    my=((double)(h/2.0)-y)*resolution;
}

bool Sensor::senseCell(int x, int y)
{
    if(m_map.empty())
        return false;
    int w=m_w;
    int h=m_h;
    int index=y*w+x;
    if(index>=w*h)
        return false;
    if(index<0)
        return false;

    return m_map[index]>50;
}
#include <iostream>
void Sensor::senseMap(int sensor)
{
    int sx,sy;
    int mx,my;
    int lx,ly;
    double sq=is[sensor].pos.q;
//    double rx=is[sensor].pos.x;
//    double ry=is[sensor].pos.y;
    double rx=m_rPos[INDEX_X]-m_mPos[INDEX_X];
    double ry=m_rPos[INDEX_Y]-m_mPos[INDEX_Y];

    double max_dist=ip.sparam.max_dist;
    M2P(rx,ry,sx,sy);
    M2P(rx+max_dist*cos(sq),ry+max_dist*sin(sq),lx,ly);
    M2P(m_mPos[INDEX_X],m_mPos[INDEX_Y],mx,my);

    int dx=     abs(lx-sx);
    int dy=    -abs(ly-sy);
    int step_x= sx<lx?1:-1;
    int step_y= sy<ly?1:-1;
    int err=    dx+dy;

    int limit_lx,limit_ly;
    int limit_rx,limit_ry;

    M2P(m_limit[LEFT_TOP_X],    m_limit[LEFT_TOP_Y],    limit_lx,limit_ly);
    M2P(m_limit[RIGHT_BOTTOM_X],m_limit[RIGHT_BOTTOM_Y],limit_rx,limit_ry);

    bool bRun=true;
    while(true)
    {
        if(!bRun)
        {
            is[sensor].sense.dist=-1.0;
            is[sensor].sense.vx=0.0;
            is[sensor].sense.vy=0.0;
            break;
        }
        if(sx<(limit_lx)||sx>(limit_rx)||sy>(limit_ly)||sy<(limit_ry))
        {
            bRun=false;
            continue;
        }
        if(senseCell(sx,sy))
        {
            double sen_x,sen_y;
            double weight=1.0;
            P2M(sx,sy,sen_x,sen_y);
            sen_y=-sen_y;
            double dist=sqrt(pow(sen_x-rx,2)+pow(sen_y-ry,2));//dist = robot_c to sen(x,y)
            double dist2=dist;
            dist-=0.35;
            if(dist<0.0)
            {
                dist=0.05;
                //dist=-dist;
            }
            else if(dist<0.1)
            {
                dist=0.05;
            }
            is[sensor].sense.dist=weight*dist;
            is[sensor].sense.vx=(sen_x-rx)/(dist2);
            is[sensor].sense.vy=(sen_y-ry)/(dist2);
            break;
        }
        if(sx==lx&&sy==ly)
        {
            bRun=false;
            continue;
        }
        int error2=2*err;
        if(error2>=dy)
        {
            if(sx==lx)
            {
                bRun=false;
                continue;
            }
            err+=dy;
            sx+=step_x;
        }
        if(error2<=dx)
        {
            if(sy==ly)
            {
                bRun=false;
                continue;
            }
            err+=dx;
            sy+=step_y;
        }
    }
}

bool Sensor::senseMap()
{
    int sx,sy;
    int mx,my;
    int lx,ly;
    double rx=m_rPos[INDEX_X]-m_mPos[INDEX_X];
    double ry=m_rPos[INDEX_Y]-m_mPos[INDEX_Y];

    M2P(rx,ry,sx,sy);
    M2P(m_mPos[INDEX_X],m_mPos[INDEX_Y],lx,ly);
    M2P(m_mPos[INDEX_X],m_mPos[INDEX_Y],mx,my);
    int dx=     abs(lx-sx);
    int dy=    -abs(ly-sy);
    int step_x= sx<lx?1:-1;
    int step_y= sy<ly?1:-1;
    int err=    dx+dy;

    int limit_lx,limit_ly;
    int limit_rx,limit_ry;

    M2P(m_limit[LEFT_TOP_X],    m_limit[LEFT_TOP_Y],    limit_lx,limit_ly);
    M2P(m_limit[RIGHT_BOTTOM_X],m_limit[RIGHT_BOTTOM_Y],limit_rx,limit_ry);

    bool bRun=true;
    while(true)
    {
        if(!bRun)
        {
            return false;
        }
        if(sx<(limit_lx)||sx>(limit_rx)||sy>(limit_ly)||sy<(limit_ry))
        {
            return true;
        }
        if(senseCell(sx,sy))
        {
            return true;
        }
        if(sx==lx&&sy==ly)
        {
            bRun=false;
            continue;
        }
        int error2=2*err;
        if(error2>=dy)
        {
            if(sx==lx)
            {
                bRun=false;
                continue;
            }
            err+=dy;
            sx+=step_x;
        }
        if(error2<=dx)
        {
            if(sy==ly)
            {
                bRun=false;
                continue;
            }
            err+=dx;
            sy+=step_y;
        }
    }
}

void Sensor::sense(double *pos)
{
    //global robot pos
    int sensor=ip.sparam.num_sensors;
    m_rPos[INDEX_X]=pos[INDEX_X];
    m_rPos[INDEX_Y]=pos[INDEX_Y];
    m_rPos[INDEX_Q]=pos[INDEX_Q];

    updateSensorPos();
    for(int i=0;i<sensor;i++)
    {
        senseMap(i);
    }
}
