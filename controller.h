#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "sensor.h"
#include "generator.h"
class Controller
{
public:
    Controller();
    ~Controller();
    struct goal
    {
        double x;
        double y;
        double theta;
        double d;
        bool   arrived;
    };
    struct optimized_data
    {
        double x;
        double y;
        double cost1[4];
        double cost2[4];
        double loss;
    };

    enum con_state
    {
        idle,
        localminimum,
        optimized
    };
    void addGoal(double x, double y, double theta);
    void setTGoal(double x, double y, double theta, double d);
    void checkMaxVelocity(double vel, double vel_max, double &dst);
    void velocity(double* src, double& v, double& w);

    void control(double* src);//input global position(map -> base_footprint)
    void detectLocalminimum(bool& bLocalminimum);
    void setState(bool bLobalminimum);
    void planing();
    void moveGoal();

    void getPos(double* dst);
    void getGoal(double* dst,bool bGlobal);
    void checkGoal();
    bool checkGoal(std::vector<Generator::path> path,bool bGlobal);

    bool isArrived();

    void optimize(const double *pos, double *param,double* dst, double* cst1, double* cst2, double& loss);
    double cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double* cst, double& loss);
    double normalizeCross(std::vector<Generator::path> path, int& argFeature);
    Sensor* s;
    Generator* g;

    std::vector<optimized_data> o;
    std::vector<Generator::path> optimized_path;
    std::vector<Generator::path> b_path;

    double stag_pos[2];
    double con_vel[2];
    bool   bArrived;

protected:
    void initConState(double* pos);
    void setOutput(double* v);

private:
    goal temporary;
    std::vector<goal> goals;

    double m_rPos[SIZE_STATE];
    double eold;

    double kp,kd;
    double minLoss;
    con_state state;
};

#endif // CONTROLLER_H
