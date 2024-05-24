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
    struct t_polygon
    {
        double p1[SIZE_STATE-1];
        double p2[SIZE_STATE-1];
        double p3[SIZE_STATE-1];
    };
    struct particle
    {
        std::vector<Generator::path> p;
        std::vector<double> c1;
        std::vector<double> c2;
        std::vector<double> c3;
        std::vector<double> c4;
        std::vector<double> c5;
        std::vector<double> c6;
        std::vector<double> w;
    };

    enum con_state
    {
        idle,
        test,
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
    void createTPolygon();
    void createPointInTriangle(t_polygon polygon,double* dst);
    std::vector<Generator::path> createParticle(int num);
    void moveParticle();
    void calcCost();
    double cost1(std::vector<Generator::path> path);
    double cost2(std::vector<Generator::path> path);
    double cost3(std::vector<Generator::path> path);
    double cost4(double* pos);
    double cost5(double* pos);
    double cost6(double* pos);
    std::vector<double> calculateWeight(double w1,double w2, double w3, double w4, double w5, double w6);
    double calcESS();
    double calcEntropy(std::vector<double> &weights);
    void resample();
    bool updateParticle();

    bool isArrived();

    void optimize(const double *pos, double *param,double* dst, double* cst1, double* cst2, double& loss);
    double cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double* cst, double& loss);
    double normalizeCross(std::vector<Generator::path> path, int& argFeature);
    Sensor* s;
    Generator* g;

    std::vector<optimized_data> o;
    std::vector<Generator::path> optimized_path;
    std::vector<Generator::path> origin_path;
    std::vector<t_polygon> tPolygon;
    particle p;

    double stag_pos[2];
    double con_vel[2];
    bool   bArrived;
    bool   bFirst=true;

protected:
    void initConState(double* pos);
    void setOutput(double* v);
    void selectBeam(int& dst, bool direction,int src=-1);
    void genSample(double *point1, double *point2,double *dst);
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
