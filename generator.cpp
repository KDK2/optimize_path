#include "generator.h"
#include "math.h"

#define RAD(X) ((X)*(M_PI/180.0))
Generator::Generator(const info in, const Sensor &sen, const double *pos, const double *gpos)
{
    ip=in;
    s=new Sensor(sen);

    m_rPos[INDEX_X]=pos[INDEX_X];
    m_rPos[INDEX_Y]=pos[INDEX_Y];
    normalizeAngle(pos[INDEX_Q],m_rPos[INDEX_Q]);

    m_gGoal[INDEX_X]=gpos[INDEX_X];
    m_gGoal[INDEX_Y]=gpos[INDEX_Y];
    normalizeAngle(gpos[INDEX_Q],m_gGoal[INDEX_Q]);

    m_rPath.clear();
    m_rPath.push_back({m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]});
    m_bLocalMin=false;
}

Generator::Generator(const Generator &gen)
{
    ip=gen.ip;
    s=new Sensor(*gen.s);

    m_rPos[INDEX_X]=gen.m_rPos[INDEX_X];
    m_rPos[INDEX_Y]=gen.m_rPos[INDEX_Y];
    normalizeAngle(gen.m_rPos[INDEX_Q],m_rPos[INDEX_Q]);

    m_gGoal[INDEX_X]=gen.m_gGoal[INDEX_X];
    m_gGoal[INDEX_Y]=gen.m_gGoal[INDEX_Y];
    normalizeAngle(gen.m_gGoal[INDEX_Q],m_gGoal[INDEX_Q]);

    m_rPath.clear();
    m_rPath.push_back({m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]});
    m_bLocalMin=false;
}

Generator::Generator(const Generator &gen, const double *pos)
{
    ip=gen.ip;
    s=new Sensor(*gen.s);

    m_rPos[INDEX_X]=pos[INDEX_X];
    m_rPos[INDEX_Y]=pos[INDEX_Y];
    normalizeAngle(pos[INDEX_Q], m_rPos[INDEX_Q]);

    m_gGoal[INDEX_X]=gen.m_gGoal[INDEX_X];
    m_gGoal[INDEX_Y]=gen.m_gGoal[INDEX_Y];
    normalizeAngle(gen.m_gGoal[INDEX_Q],m_gGoal[INDEX_Q]);

    m_rPath.clear();
    m_rPath.push_back({m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]});
    m_bLocalMin=false;
}

Generator::~Generator()
{
    delete s;
}

void Generator::setSensor(Sensor &sen)
{
    if(s!=nullptr)
    {
        delete s;
    }
    s=new Sensor(sen);
}

void Generator::normalizeAngle(double angle, double &dst)
{
    angle = fmod(angle, 2.0 * M_PI);

    if(angle>M_PI)
        angle -= 2.0 * M_PI;
    else if(angle<-M_PI)
        angle += 2.0 * M_PI;

    dst=angle;
}

void Generator::setGoal(double *goal)
{
    m_gGoal[INDEX_X]=goal[INDEX_X];
    m_gGoal[INDEX_Y]=goal[INDEX_Y];
    normalizeAngle(goal[INDEX_Q],m_gGoal[INDEX_Q]);
}

void Generator::setPos(double *pos)
{
    m_rPos[INDEX_X]=pos[INDEX_X];
    m_rPos[INDEX_Y]=pos[INDEX_Y];
    normalizeAngle(pos[INDEX_Q],m_rPos[INDEX_Q]);

    m_rPath.clear();
    m_rPath.push_back({m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]});
}

void Generator::getTgoal(double *tgoal)
{
    tgoal[INDEX_X]=m_tGoal[INDEX_X];
    tgoal[INDEX_Y]=m_tGoal[INDEX_Y];
    tgoal[INDEX_Q]=m_tGoal[INDEX_Q];
}

void Generator::gen(genmode mode)
{
    genmode m=mode;
    if(genmode::prediction==m)
    {
        predict(false);
    }
    if(genmode::reference==m)
    {
        ref();
    }
    if(genmode::stagnation==m)
    {
        predict(true);
        detLocalmin();
    }
}

void Generator::getRef(double &v, double &q)
{
    v=v_ref;
    q=q_ref;
}

void Generator::getStagPos(double *pos)
{
    double mean_x, mean_y;
    mean_x=std::accumulate(m_rPath.begin(),m_rPath.end(),0.0,[](double sum, path p){return sum+p.px;});
    mean_x/=m_rPath.size();
    mean_y=std::accumulate(m_rPath.begin(),m_rPath.end(),0.0,[](double sum, path p){return sum+p.py;});
    mean_y/=m_rPath.size();
    pos[INDEX_X]=mean_x;
    pos[INDEX_Y]=mean_y;
}

double Generator::calcTgoal()
{
    double rg_x,rg_y;
    double a,b,c;
    double dMax=0.0;
    int    index=0;
    int    iSize=m_rPath.size();

    rg_x=m_gGoal[INDEX_X]-m_rPos[INDEX_X];
    rg_y=m_gGoal[INDEX_Y]-m_rPos[INDEX_Y];

    a=(rg_y/rg_x);
    b=-1.0;
    c=(-a*m_rPos[INDEX_X]+m_rPos[INDEX_Y]);
    for(int i=0;i<iSize;i++)
    {
        double path[2]={m_rPath.at(i).px,m_rPath.at(i).py};
        double d=abs(a*path[INDEX_X]+b*path[INDEX_Y]+c)/sqrt(a*a+b*b);
        if(d>dMax)
        {
            dMax=d;
            index=i;
        }
    }
    m_tGoal[INDEX_X]=m_rPath.at(index).px;
    m_tGoal[INDEX_Y]=m_rPath.at(index).py;
    m_tGoal[INDEX_Q]=m_rPath.at(index).pq;
    return dMax;
}

double Generator::addNoise(double src, double sigma)
{
    std::normal_distribution<double> d(0.0,sigma);
    return src+d(rand_gen);
}

bool Generator::isLocalMin()
{
    return m_bLocalMin;
}

std::vector<Generator::path> Generator::getPath()
{
    return m_rPath;
}

void Generator::attForce(double *pos, double *f)
{
    double x=pos[INDEX_X];
    double y=pos[INDEX_Y];
    double gx=m_gGoal[INDEX_X];
    double gy=m_gGoal[INDEX_Y];

    double dx=gx-x;
    double dy=gy-y;
    double dist=sqrt(pow(dx,2)+pow(dy,2));
    double cutoff=1.0;

    double k_vg=ip.f_param.aparam.k_vg;
    double q_v=ip.f_param.mparam.q_v;
    double q_g=ip.f_param.aparam.q_g;

    if(dist>cutoff)
    {
        f[INDEX_X]=-((k_vg*q_v*q_g))*(1.0/(dist*dist))*(dx/dist);
        f[INDEX_Y]=-((k_vg*q_v*q_g))*(1.0/(dist*dist))*(dy/dist);
    }

    else
    {
        f[INDEX_X]=-((k_vg*q_v*q_g))*(dx);
        f[INDEX_Y]=-((k_vg*q_v*q_g))*(dy);
    }
}

void Generator::repForce(int i, double *f)
{
    double d_o=ip.f_param.rparam.d_o;
    double k_vo=ip.f_param.rparam.k_vo;
    double q_v=ip.f_param.mparam.q_v;
    double q_o=ip.f_param.rparam.q_o;
    double dist=s->is[i].sense.dist;
    double v_sp[2]={s->is[i].sense.vx,s->is[i].sense.vy};
    if(dist<d_o && 0<dist)
    {
        f[INDEX_X]=((k_vo*q_v*q_o)/(dist*dist))*(v_sp[INDEX_X]);
        f[INDEX_Y]=((k_vo*q_v*q_o)/(dist*dist))*(v_sp[INDEX_Y]);
    }
    else
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
    if(dist<0)
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
}

void Generator::force(double *pos, double *f)
{
    double aForce[2]={0.0,0.0};
    double rForce[2]={0.0,0.0};

    int s_num=s->ip.sparam.num_sensors;

    attForce(pos, aForce);
    for(int i=0;i<s_num;i++)
    {
        double rForce_temp[2]={0.0,0.0};
        repForce(i, rForce_temp);
        rForce[INDEX_X]+=rForce_temp[INDEX_X];
        rForce[INDEX_Y]+=rForce_temp[INDEX_Y];
    }
    rForce[INDEX_X]=-rForce[INDEX_X];
    rForce[INDEX_Y]=-rForce[INDEX_Y];

    f[INDEX_X]=aForce[INDEX_X]+rForce[INDEX_X];
    f[INDEX_Y]=aForce[INDEX_Y]+rForce[INDEX_Y];
}

void Generator::ref()
{
    double norm,ref,alpha;
    s->sense(m_rPos);
    double tForce[2]={0.0,0.0};
    force(m_rPos,tForce);
    norm=sqrt(pow(tForce[INDEX_X],2)+pow(tForce[INDEX_Y],2));
    ref=atan2(tForce[INDEX_Y],tForce[INDEX_X]);
    checkMaxRef(ref,alpha);
    v_ref=norm*alpha;
    q_ref=ref;
}

void Generator::checkMaxRef(double ref, double &dst)
{
    double ret,error,max_error;
    double q=m_rPos[INDEX_Q];

    max_error=ip.m_param.eparam.theta_max;
    normalizeAngle(abs(ref-q),error);
    if(error<=max_error)
    {
        ret=(max_error-error)/max_error;
    }
    else
    {
        ret=0.0;
    }
    dst=ret;
}

void Generator::predict(bool bStag)
{
    double lam=ip.p_param.lparam.lam;
    double lam_stagnation=ip.p_param.lparam.lam_stagnation;
    double delta=ip.p_param.lparam.delta;
    double iter_max=bStag?lam_stagnation/delta:lam/delta;

    for(int i=1;i<iter_max;i++)
    {
        double px=m_rPath[i-1].px;
        double py=m_rPath[i-1].py;
        double pq=m_rPath[i-1].pq;
        double pos[SIZE_STATE]={px,py,pq};
        double tForce[2]={0.0,0.0};
        double ref=0.0;
        double x,y,q;
        s->sense(pos);
        force(pos,tForce);

        ref=atan2(tForce[INDEX_X],tForce[INDEX_Y]);
        normalizeAngle(ref,q);
        x=px+delta*cos(q);
        y=py+delta*sin(q);
        m_rPath.push_back({x,y,q});
    }
}

void Generator::detLocalmin()
{
    double mean_x,mean_y;
    mean_x=std::accumulate(m_rPath.begin(),m_rPath.end(),0.0,[](double sum, path p){return sum+p.px;});
    mean_x/=m_rPath.size();
    mean_y=std::accumulate(m_rPath.begin(),m_rPath.end(),0.0,[](double sum, path p){return sum+p.py;});
    mean_y/=m_rPath.size();


    if(m_rPath.empty()) return;

    bool ret=true;
    for(int i=0;i<m_rPath.size();i++)
    {
        double diff_x = m_rPath.at(i).px-mean_x;
        double diff_y = m_rPath.at(i).py-mean_y;
        double norm   = sqrt(pow(diff_x,2)+pow(diff_y,2));
        ret&=norm<ip.p_param.lparam.radius;
    }
    m_bLocalMin=ret;

    double varianceX=0.0;
    double varianceY=0.0;
    for(int i=0;i<m_rPath.size();i++)
    {
        varianceX+=pow(m_rPath.at(i).px-mean_x,2);
        varianceY+=pow(m_rPath.at(i).py-mean_y,2);
    }
    // varianceX/=m_rPath.size();
    // varianceY/=m_rPath.size();
}
