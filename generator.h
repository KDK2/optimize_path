#ifndef GENERATOR_H
#define GENERATOR_H
#include <random>
#include "sensor.h"
#include <vector>

class Generator
{
public:
    struct info
    {
        struct force_param
        {
            struct mobile_param
            {
                double q_v;
            };
            struct att_param
            {
                double k_vg, q_g;
            };
            struct rep_param
            {
                double k_vo, q_o, d_o, d_oq;
            };
            struct quark_param
            {
                double k_vq, q_q;
            };
            mobile_param mparam;
            att_param aparam;
            rep_param rparam;
            quark_param qparam;
        };
        struct predict_param
        {
            struct length_param
            {
                double lam,delta,lam_stagnation,radius;
            };
            length_param lparam;
        };
        struct move_param
        {
            struct velocity_param
            {
                double v_max, w_max;
            };
            struct error_param
            {
                double tolorance;
                double theta_max;
            };
            velocity_param vparam;
            error_param eparam;
        };
        force_param f_param;//force
        predict_param p_param;//predict
        move_param m_param;//move
    };
    struct path
    {
        double px,py,pq;
    };
    enum genmode{reference, prediction, stagnation};
    Generator(const info in, const Sensor& sen, const double* pos, const double* gpos);
    Generator(const Generator& gen, Sensor& sen, const double* pos);
    Generator(const Generator& gen, const double *pos);
    ~Generator();

    void setSensor(Sensor &sen);
    void normalizeAngle(double src, double &dst);
    void setGoal(double *goal);
    void setPos(double *pos);
    void getTgoal(double *tgoal);

    void gen(genmode mode);
    void getRef(double &v, double &q);
    void getStagPos(double* pos);

    double calcTgoal();
    double addNoise(double src, double sigma);
    double addRealNoise(double src, double sigma);

    int    rndInt(int range);
    double rndDouble(double min,double max);
    void   setDD(const std::vector<double>& weights);
    int    rndDD();

    bool isLocalMin();

    std::vector<path> getPath();

    info ip;
    std::vector<path> m_rPath;
    Sensor* s;
    bool isArrived=false;
    void ref();
protected:
    void attForce(double* pos, double* f);
    void repForce(int i, double* f);

    void force(double* pos, double* f);
    void checkMaxRef(double ref, double& dst);
    void checkMaxRef(double ref, double* src, double& dst);
    void predict(bool bStag);
    void detLocalmin();

private:
    double m_rPos[SIZE_STATE];
    double m_gGoal[SIZE_STATE];
    double m_tGoal[SIZE_STATE];
    double v_ref,q_ref;

    bool m_bLocalMin;
    std::mt19937 rand_gen;
    std::discrete_distribution<> *dd=nullptr;
};

#endif // GENERATOR_H
