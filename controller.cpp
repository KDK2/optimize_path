#include "controller.h"
#include "math.h"
#include <iostream>
#include <algorithm>
#define INDEX_REF_V 0
#define INDEX_REF_Q 1
#define INDEX_LINEAR 0
#define INDEX_ANGULAR 1
Controller::Controller():
    s(nullptr),
    g(nullptr),
    bArrived(false),
    temporary({0,0,0,0,true}),
    m_rPos({0.0,0.0,0}),
    eold(0.0),
    kp(2.2),
    kd(0.8),
    minLoss(0.0),
    state(idle)
{

}

Controller::~Controller()
{
    delete g;
    delete s;
}

void Controller::addGoal(double x, double y, double theta)
{
    goal g;
    g.x=x;
    g.y=y;
    g.theta=theta;
    g.arrived=false;
    goals.push_back(g);
}

void Controller::setTGoal(double x, double y, double theta, double d)
{
    temporary.x=x;
    temporary.y=y;
    temporary.theta=theta;
    temporary.arrived=false;
    temporary.d=d;
    eold=0.0;
}

void Controller::checkMaxVelocity(double vel, double vel_max, double &dst)
{
    if(abs(vel)<=vel_max)
    {
        dst=vel;
    }
    else
    {
        dst=vel_max*vel/abs(vel);
    }
}

void Controller::velocity(double *src, double &v, double &w)
{
    double v_max=g->ip.m_param.vparam.v_max;
    double w_max=g->ip.m_param.vparam.w_max;
    double v_ref=src[INDEX_REF_V];
    double q_ref=src[INDEX_REF_Q];
    double e;
    g->normalizeAngle(q_ref-m_rPos[INDEX_Q],e);

    checkMaxVelocity(v_ref,v_max,v);
    checkMaxVelocity(kp*e+kd*(e-eold),w_max,w);
    eold=e;
}

void Controller::control(double *src)
{
    bool bDetect=false;
    if(isArrived())
    {
        double v[2]={0.0,0.0};
        setOutput(v);
        bArrived=true;
        return;
    }
    initConState(src);//set global pos
    s->sense(src);
    detectLocalminimum(bDetect);
    setState(bDetect);
    planing();
    moveGoal();
    checkGoal();
}

void Controller::detectLocalminimum(bool &bLocalminimum)
{
    Generator* pRef=nullptr;
    Generator* pLocal=nullptr;
    double target[SIZE_STATE];
    double lastPredict[SIZE_STATE];
    double pos[SIZE_STATE]={m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]};//global pos

    getGoal(target,true);
    pRef=new Generator(*g,*s,pos);
    pRef->setGoal(target);
    pRef->gen(Generator::prediction);
    origin_path=pRef->getPath();
    lastPredict[INDEX_X]=pRef->getPath().back().px;
    lastPredict[INDEX_Y]=pRef->getPath().back().py;
    lastPredict[INDEX_Q]=pRef->getPath().back().pq;

    pLocal=new Generator(*g,*s,lastPredict);
    pLocal->setGoal(target);
    pLocal->gen(Generator::stagnation);
    pLocal->getStagPos(stag_pos);
    if(!checkGoal(pLocal->getPath(),true))
    {
        if(pLocal->isLocalMin())
        {
            bLocalminimum=true;
        }
        else
        {
            bLocalminimum=false;
        }
    }
    else
    {
        bLocalminimum=false;
    }
}

void Controller::setState(bool bLocalminimum)
{
    if(bLocalminimum)
    {
        if     (idle==state)         state=test;
        else if(localminimum==state) state=localminimum;
        else if(optimized==state)    state=optimized;
    }
    else
    {
        if     (idle==state)         state=idle;
        //else if(localminimum==state) state=optimized;
        else if(optimized==state)
        {
            if(temporary.arrived)
                state=idle;
        }
    }
}

void Controller::planing()
{
    Generator* ref=nullptr;
    double d=0.0;
    double tg[3];

    if(idle==state)
    {
        double go[SIZE_STATE];
        getGoal(go,true);
        ref=new Generator(*g,*s,m_rPos);
        ref->setGoal(go);
        ref->gen(Generator::prediction);
        d=ref->calcTgoal();
        ref->getTgoal(tg);
        setTGoal(tg[INDEX_X],tg[INDEX_Y],tg[INDEX_Q],d);
        g->m_rPath=ref->getPath();
        return;
    }
    else if(test==state)
    {
        if(p.p.empty())
        {
            int num=200;
            p.p=createParticle(num);
            std::vector<double> temp(num,1.0/num);
            p.c1=temp;
            p.c2=temp;
            p.c3=temp;
            p.c4=temp;
            p.c5=temp;
            p.c6=temp;
            p.w =temp;
        }
        else
        {
            bool ret=false;
            ret=updateParticle();
            if(ret)
            {
                state=optimized;
            }
        }
    }
    else if(localminimum==state)
    {
        o.clear();
        optimized_path.clear();

        int sgd_iter=200;
        double oPos[SIZE_STATE]={m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]};
        double param[2]={0.0,0.0};
        std::vector<optimized_data>temp_o(sgd_iter);

        bool bHeading=true;
        int iBeam=0;
        int newBeam=-1;
        int num_s=s->ip.sparam.num_sensors;
        bool bNext=false;
        while(true)
        {
            if(bNext)
            {
                o.clear();
                optimized_path.clear();
                if(iBeam>0&&bHeading)
                {
                    newBeam=iBeam-1;
                }
                else if(iBeam==0&&bHeading)
                {
                    newBeam=-1;
                    bHeading=false;
                }
                else if(iBeam<num_s&&(!bHeading))
                {
                    newBeam=iBeam+1;
                }
                else if(iBeam==num_s&&(!bHeading))
                {
                    newBeam=-1;
                    bHeading=true;
                }
            }
            selectBeam(iBeam,bHeading,newBeam);
            double d=s->is[iBeam].sense.dist;
            double theta=s->is[iBeam].pos.q;
            double radius=s->ip.sparam.radius;
            double d_o=g->ip.f_param.rparam.d_o;
            double d_max=s->ip.sparam.max_dist;
            double dist=d<0.0?d_max-d_o-2.0*radius:d-d_o-2.0*radius;
            double p1x=2.0*radius*cos(theta);
            double p1y=2.0*radius*sin(theta);
            double p2x=dist*cos(theta);
            double p2y=dist*sin(theta);
            double p1[2]={p1x,p1y};
            double p2[2]={p2x,p2y};
            double sample[2]={0.0,0.0};
            param[INDEX_X]=(p1x+p2x)/2.0;
            param[INDEX_Y]=(p1y+p2y)/2.0;
            for(int i=0;i<sgd_iter;i++)
            {
                double dst[2];
                genSample(p1,p2,sample);
                std::cout<<"sample ="<<sample[INDEX_X]<<", "<<sample[INDEX_Y]<<std::endl;
                optimize(sample,param,dst,temp_o[i].cost1,temp_o[i].cost2,temp_o[i].loss);
                oPos[INDEX_X]=dst[INDEX_X];
                oPos[INDEX_Y]=dst[INDEX_Y];
                param[INDEX_X]=dst[INDEX_X];
                param[INDEX_Y]=dst[INDEX_Y];
                temp_o[i].x=oPos[INDEX_X];
                temp_o[i].y=oPos[INDEX_Y];
                o.push_back(temp_o[i]);
            }
            if(o.size()>0)
            {
                std::vector<optimized_data> to;
                for(int i=0;i<o.size();i++)
                {
                    if(o[i].loss<-1.4)
                        to.push_back(o[i]);
                }
                if(to.size()==0)
                {
                    std::cout<<"no optimized data"<<std::endl;
                    bNext=true;
                    continue;
                }
                auto minIt = std::min_element(to.begin(), to.end(),
                                              [](const optimized_data& a, const optimized_data& b){return a.loss < b.loss;});
                int minIndex = std::distance(to.begin(), minIt);
                if(minLoss>to[minIndex].loss)
                {
                    minLoss=to[minIndex].loss;
                    //o.clear();
                    o.push_back(to[minIndex]);
                    bNext=false;
                }
                else
                {
                    bNext=true;
                    continue;
                }
                //minLoss=0.0;
                double gGoal[SIZE_STATE];
                getGoal(gGoal,true);
                double theta=atan2(gGoal[INDEX_Y]-to[minIndex].y,gGoal[INDEX_X]-to[minIndex].x);
                double op_pos[SIZE_STATE]={to[minIndex].x,to[minIndex].y,theta};
                double tGoal[SIZE_STATE];
                ref=new Generator(*g,*s,op_pos);
                ref->setGoal(gGoal);
                ref->gen(Generator::prediction);
                optimized_path=ref->getPath();
                //std::cout<<"optimized first : "<<optimized_path.at(0).px<<", "<<optimized_path.at(0).py<<std::endl;
//                int argValid=-1;
//                normalizeCross(optimized_path,argValid);
//                if(argValid==-1)
//                {
//                    minLoss=0.0;
//                    //o.clear();
//                    bNext=true;
//                    continue;
//                }
                //optimized_path.erase(optimized_path.begin(),optimized_path.begin()+argValid);
//                double mean_x=std::accumulate(optimized_path.begin(),optimized_path.begin()+argValid,0.0,[](double sum, Generator::path p){return sum+p.px;});
//                mean_x/=argValid;
//                double mean_y=std::accumulate(optimized_path.begin(),optimized_path.begin()+argValid,0.0,[](double sum, Generator::path p){return sum+p.py;});
//                mean_y/=argValid;
                ref->setPos(m_rPos);
                ref->m_rPath=optimized_path;
                d=ref->calcTgoal();
                ref->getTgoal(tGoal);
                state=optimized;
                setTGoal(tGoal[INDEX_X],tGoal[INDEX_Y],tGoal[INDEX_Q],d);
                //setTGoal(mean_x,mean_y,0.0,0.0);
                std::cout<<"loss : "<<to[minIndex].loss<<std::endl;
                std::cout<<"beam : "<<iBeam<<std::endl;
                break;
            }
            else
            {
                bNext=true;
                continue;
            }
        }
    }
}

void Controller::moveGoal()
{
    if(localminimum==state)
        return;

    double tg[3];
    double v_ref,q_ref,v,w;
    double ref[2];
    getGoal(tg,false);
    g->setGoal(tg);
    g->s=s;
    g->setPos(m_rPos);
    g->gen(Generator::reference);
    g->getRef(v_ref,q_ref);
    ref[INDEX_LINEAR]=v_ref;
    ref[INDEX_ANGULAR]=q_ref;
    velocity(ref,v,w);
    con_vel[INDEX_LINEAR]=v;
    con_vel[INDEX_ANGULAR]=w;
}

void Controller::getPos(double *dst)
{
    dst[INDEX_X]=m_rPos[INDEX_X];
    dst[INDEX_Y]=m_rPos[INDEX_Y];
    dst[INDEX_Q]=m_rPos[INDEX_Q];
}

void Controller::getGoal(double *dst, bool bGlobal)
{
    if(bGlobal)
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
    if(!temporary.arrived)
    {
        dst[INDEX_X]=temporary.x;
        dst[INDEX_Y]=temporary.y;
        dst[INDEX_Q]=temporary.theta;
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
}

void Controller::checkGoal()
{
    double goal[3];
    getGoal(goal,false);

    double dx=goal[INDEX_X]-m_rPos[INDEX_X];
    double dy=goal[INDEX_Y]-m_rPos[INDEX_Y];
    double d=sqrt(dx*dx+dy*dy);
    double tolorance=g->ip.m_param.eparam.tolorance;
    if(!temporary.arrived)
    {
        if(d<tolorance)
        {
            temporary.arrived=true;
            temporary.d=0.0;
            //s->vq.clear();
        }
    }
    for(int i=0;i<goals.size();i++)
    {
        if(goals[i].arrived)
            continue;

        dx=goals[i].x-m_rPos[INDEX_X];
        dy=goals[i].y-m_rPos[INDEX_Y];
        d =sqrt(dx*dx+dy*dy);

        if(d<tolorance)
            goals[i].arrived=true;
    }
}

bool Controller::checkGoal(std::vector<Generator::path> path, bool bGlobal)
{
    double goal[SIZE_STATE];
    getGoal(goal,bGlobal);
    for(int i=0;i<path.size();i++)
    {
        double dx=path[i].px-goal[INDEX_X];
        double dy=path[i].py-goal[INDEX_Y];
        double d=sqrt(dx*dx+dy*dy);
        if(d<g->ip.m_param.eparam.tolorance) return true;
    }
    return false;
}

void Controller::createTPolygon()
{
    int s_num=s->ip.sparam.num_sensors;
    double robot[SIZE_STATE-1]={m_rPos[INDEX_X],m_rPos[INDEX_Y]};
    double r_radius=s->ip.sparam.radius;
    double d_max=s->ip.sparam.max_dist;
    double d_safe=g->ip.f_param.rparam.d_o;
    double offset=1.0*d_safe+r_radius;
    tPolygon.clear();
    for(int i=0;i<s_num-1;i++)
    {
        double d1=s->is[i].sense.dist;
        double d2=s->is[i+1].sense.dist;
        double q1=s->is[i].pos.q;
        double q2=s->is[i+1].pos.q;
        d1=d1<0.0?d_max:d1;
        d2=d2<0.0?d_max:d2;
        d1-=offset;
        d2-=offset;
        if(!(d1<=0.0||d2<=0.0))
        {
            t_polygon temp;
            double p1[SIZE_STATE-1];
            double p2[SIZE_STATE-1];
            p1[INDEX_X]=robot[INDEX_X]+d1*cos(q1);
            p2[INDEX_X]=robot[INDEX_X]+d2*cos(q2);
            p1[INDEX_Y]=robot[INDEX_Y]+d1*sin(q1);
            p2[INDEX_Y]=robot[INDEX_Y]+d2*sin(q2);

            memcpy(&temp.p1,&p1,sizeof(double)*2);
            memcpy(&temp.p2,&p2,sizeof(double)*2);
            memcpy(&temp.p3,&robot,sizeof(double)*2);
            tPolygon.push_back(temp);
        }
    }
}

void Controller::createPointInTriangle(Controller::t_polygon polygon, double* dst)
{
    double r1=g->addRealNoise(0.0,1.0);
    double r2=g->addRealNoise(0.0,1.0);

    double sqrt_r1=std::sqrt(r1);
    double lambda1=1.0-sqrt_r1;
    double lambda2=sqrt_r1*(1.0-r2);
    double lambda3=sqrt_r1*r2;

    dst[INDEX_X]=lambda1*polygon.p1[INDEX_X]+lambda2*polygon.p2[INDEX_X]+lambda3*polygon.p3[INDEX_X];
    dst[INDEX_Y]=lambda1*polygon.p1[INDEX_Y]+lambda2*polygon.p2[INDEX_Y]+lambda3*polygon.p3[INDEX_Y];
}

std::vector<Generator::path> Controller::createParticle(int num)
{
    createTPolygon();
    if(!tPolygon.empty())
    {
        std::vector<Generator::path> temp_particle(num);
        for(int i=0;i<num;i++)
        {
            Generator::path temp_p;
            int index=g->rndInt(tPolygon.size());
            double particle[SIZE_STATE-1];
            double q=g->rndDouble(-M_PI,M_PI);
            createPointInTriangle(tPolygon[index],particle);
            temp_p.px=particle[INDEX_X];
            temp_p.py=particle[INDEX_Y];
            temp_p.pq=q;
            temp_particle[i]=temp_p;
        }
        return temp_particle;
    }
    else
    {
        std::cout<<"empty polygon!!"<<std::endl;
    }
}

void Controller::moveParticle()
{
    for(int i=0;i<p.p.size();i++)
    {
        double pos[SIZE_STATE]={p.p[i].px,p.p[i].py,p.p[i].pq};
        Generator* g_temp=new Generator(*g,*s,pos);
        double v_ref,q_ref;
        double v,w;
        g_temp->ref();
        g_temp->getRef(v_ref,q_ref);
//        v=g->addNoise(v_ref,0.01);
//        w=g->addNoise(q_ref,0.01);
        v=v_ref;
        w=q_ref;
        double dx,dy,dq;

        double x=p.p[i].px;
        double y=p.p[i].py;
        double q=p.p[i].pq;

        double r=s->ip.sparam.radius;
        double dt=0.1;

        if(w==0)
        {
            dx=0.5*r*v*cos(q)*dt;
            dy=0.5*r*v*sin(q)*dt;
        }
        else
        {
            dx=-v/w*sin(q)+v/w*sin(q+w*dt);
            dy=v/w*cos(q)-v/w*cos(q+w*dt);
        }
        dq=0.5*r*w*dt;

        p.p[i].px=x+dx;
        p.p[i].py=y+dy;
        p.p[i].pq=q+dq;
    }
}

void Controller::calcCost()
{
    int p_num=p.p.size();
    std::vector<Generator::path> pPath;
    std::vector<Generator::path> aPath;
    for(int i=0;i<p_num;i++)
    {
        Generator* g_ptemp;
        Generator* g_atemp;

        double p_pos[SIZE_STATE]={p.p[i].px,p.p[i].py,p.p[i].pq};

        g_ptemp=new Generator(*g,*s,p_pos);
        g_ptemp->gen(Generator::prediction);
        pPath=g_ptemp->getPath();
        double g_lpos[SIZE_STATE]={pPath.back().px,pPath.back().py,pPath.back().pq};
        g_atemp=new Generator(*g,*s,g_lpos);
        g_atemp->gen(Generator::stagnation);
        aPath=g_atemp->getPath();

        p.c1[i]=cost1(pPath);
        p.c2[i]=cost2(aPath);
        p.c3[i]=cost3(aPath);
        p.c4[i]=cost4(p_pos);
        p.c5[i]=cost5(p_pos);
        p.c6[i]=cost6(p_pos);
    }
}

double Controller::cost1(std::vector<Generator::path> path)
{
    int iIndex=0;
    double c1=normalizeCross(path,iIndex);
    if(c1<0.00001)
        c1=-0.1;
    return c1;
}

double Controller::cost2(std::vector<Generator::path> path)
{
    double mean_x,mean_y;
    double var_x,var_y;
    double gGoal[SIZE_STATE];
    getGoal(gGoal,true);
    int offset=10;
    mean_x=std::accumulate(path.end()-offset,path.end(),0.0,[](double sum, Generator::path p){return sum+p.px;});
    mean_x/=offset;
    mean_y=std::accumulate(path.end()-offset,path.end(),0.0,[](double sum, Generator::path p){return sum+p.py;});
    mean_y/=offset;
    double d=0.0;
    for(int i=path.size()-offset;i<path.size();i++)
    {
        var_x+=pow(path[i].px-mean_x,2);
        var_y+=pow(path[i].py-mean_y,2);
        d=sqrt(pow(path[i].px-gGoal[INDEX_X],2)+pow(path[i].py-gGoal[INDEX_Y],2));
        if(d<0.1)
            return 0.001;
    }
    var_x/=offset;
    var_y/=offset;
    if(sqrt(pow(path.back().px-stag_pos[INDEX_X],2)+pow(path.back().py-stag_pos[INDEX_Y],2))<0.1)
        return -0.05;
    return sqrt(var_x*var_x+var_y*var_y);
}

double Controller::cost3(std::vector<Generator::path> path)
{
    double gGoal[SIZE_STATE];
    int offset=10;
    std::vector<double> dist(offset);
    getGoal(gGoal,true);
    for(int i=path.size()-offset;i<path.size();i++)
    {
        double dx=gGoal[INDEX_X]-path[i].px;
        double dy=gGoal[INDEX_Y]-path[i].py;
        double d=sqrt(dx*dx+dy*dy);
        dist[i-(path.size()-offset)]=d;
    }
    auto min_itr=std::min_element(dist.begin(),dist.end());
    double d_min=*min_itr;
    double d_o=sqrt(pow(gGoal[INDEX_X]-m_rPos[INDEX_X],2)+pow(gGoal[INDEX_Y]-m_rPos[INDEX_Y],2));
    return d_min/(d_min+d_o);//d_min=0.0->1.0 d_min=1.0-> <1.0
}

double Controller::cost4(double *pos)
{
    double dr=sqrt(pow(stag_pos[INDEX_X]-m_rPos[INDEX_X],2)+pow(stag_pos[INDEX_Y]-m_rPos[INDEX_Y],2));
    double mpos[SIZE_STATE]={m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]};
    double dx=pos[INDEX_X]-mpos[INDEX_X];
    double dy=pos[INDEX_Y]-mpos[INDEX_Y];
    double d=sqrt(dx*dx+dy*dy);
    //d=1.0/(1.0+exp(-d));
    d=-(1.0/(pow(d-dr,2)+dr*dr))+1.0/(dr*dr);
    return d;
}
bool compare(double a, double b)
{
    if(a<0.0)
        a=4.5;
    if(b<0.0)
        b=4.5;
    return (std::abs(a)<std::abs(b));
}
double Controller::cost5(double* pos)
{
    Generator* g_temp=new Generator(*g,*s,m_rPos);
    g_temp->setGoal(pos);
    g_temp->ip.p_param.lparam.delta=0.2;
    g_temp->gen(Generator::prediction);
    bool arv=false;
    arv=g_temp->isArrived;
    double c5=arv==false?0.5:0.0;
    return c5;
}

double Controller::cost6(double *pos)
{
    Generator* g_temp=new Generator(*g,*s,pos);
    g_temp->ref();
    auto min_itr=std::min_element(g_temp->s->is.begin(),g_temp->s->is.end(),
                     [](const Sensor::info_sensor& a, const Sensor::info_sensor& b){return compare(a.sense.dist,b.sense.dist);});
    int index=std::distance(g_temp->s->is.begin(),min_itr);
    double min_dist=g_temp->s->is[index].sense.dist;
    double cost=g->ip.f_param.rparam.d_o/min_dist;
    if(min_dist<0.01)
        cost=1.0;
    return cost;
}
double median(std::vector<double> vec) {
    std::sort(vec.begin(), vec.end());
    size_t n = vec.size();
    if (n % 2 == 0) {
        return (vec[n / 2 - 1] + vec[n / 2]) / 2.0;
    } else {
        return vec[n / 2];
    }
}
std::pair<double, double> calculate_iqr(const std::vector<double>& vec) {
    std::vector<double> sorted_vec = vec;
    std::sort(sorted_vec.begin(), sorted_vec.end());

    size_t n = sorted_vec.size();
    double q1 = sorted_vec[n / 4];
    double q3 = sorted_vec[3 * n / 4];

    return {q1, q3};
}

std::vector<double> robust_scale(const std::vector<double>& data) {
    std::vector<double> scaled_data(data.size());

    double med = median(data);
    auto [q1, q3] = calculate_iqr(data);
    double iqr = q3 - q1;

    for (size_t i = 0; i < data.size(); ++i) {
        scaled_data[i] = (data[i] - med) / iqr;
    }

    return scaled_data;
}
std::vector<double> invert_and_normalize_weights(const std::vector<double>& weights) {
    std::vector<double> inverted_weights(weights.size());

    // 가중치의 역수 계산
    for (size_t i = 0; i < weights.size(); ++i) {
        inverted_weights[i] = 1.0 / weights[i];
    }

    // 전체 가중치의 합 계산
    double sum_weight = std::accumulate(inverted_weights.begin(), inverted_weights.end(), 0.0);

    // 가중치 정규화
    for (auto& weight : inverted_weights) {
        weight /= sum_weight;
    }

    return inverted_weights;
}
std::vector<double> Controller::calculateWeight(double w1, double w2, double w3, double w4, double w5, double w6)
{
    //2.8 10.0 0.8
    std::vector<double> weights;
    auto min_itr_c1 = std::min_element(p.c1.begin(), p.c1.end());
    auto min_itr_c2 = std::min_element(p.c2.begin(), p.c2.end());
    auto min_itr_c3 = std::min_element(p.c3.begin(), p.c3.end());
    auto min_itr_c4 = std::min_element(p.c4.begin(), p.c4.end());
    auto min_itr_c5 = std::min_element(p.c5.begin(), p.c5.end());
    auto min_itr_c6 = std::min_element(p.c6.begin(), p.c6.end());
    double min_c1=*min_itr_c1;
    double min_c2=*min_itr_c2;
    double min_c3=*min_itr_c3;
    double min_c4=*min_itr_c4;
    double min_c5=*min_itr_c5;
    double min_c6=*min_itr_c6;
    for(int i=0;i<p.p.size();i++)
    {
        double normalized_c1=p.c1[i]-min_c1;//small
        double normalized_c2=p.c2[i]-min_c2;//small
        double normalized_c3=p.c3[i]-min_c3;//
        double normalized_c4=p.c4[i]-min_c4;
        double normalized_c5=p.c5[i]-min_c5;
        double normalized_c6=p.c6[i]-min_c6;
        double tw1=exp(-(w1*normalized_c1+w2*normalized_c2));
        double tw2=exp(w3*normalized_c3+w4*normalized_c4+w5*normalized_c5+w6*normalized_c6);
//        tw1=1.0/(1.0+exp(tw1));
//        tw2=1.0/(1.0+exp(tw2));
        double weight=tw1+tw2;
        weights.push_back(exp(weight));
    }
    weights = invert_and_normalize_weights(weights);
//    double min_weight = *std::min_element(weights.begin(), weights.end());
//    double max_weight = *std::max_element(weights.begin(), weights.end());;
//    if (min_weight<0)
//    {
//        for (auto& weight:weights)
//        {
//            weight+=std::abs(min_weight);
//        }
//    }

//    double sum_weight=0.0;
//    for(auto& weight : weights)
//    {
//        weight=max_weight-weight+min_weight;
//        sum_weight+=weight;
//    }
//    for(auto& weight : weights)
//    {
//        weight/=sum_weight;
//    }
//    min_weight = *std::min_element(weights.begin(), weights.end());
//    max_weight = *std::max_element(weights.begin(), weights.end());
//    std::cout<<"min weight : "<<min_weight<<std::endl;
//    std::cout<<"max weight : "<<max_weight<<std::endl;
    return weights;
}

double Controller::calcESS()
{
    double sum_of_weights_squared = std::accumulate(p.w.begin(), p.w.end(), 0.0,
        [](double sum, const double& a) {
            return sum+a*a;
        });
    return 1.0/sum_of_weights_squared;
}

double Controller::calcEntropy(std::vector<double> &weights)
{
    double entropy=0.0;
    for(double p : weights)
    {
        if(p>0)
        {
            entropy+=p*log2(p);
        }
    }
    return -entropy;
}
#include <numeric>
void Controller::resample()
{
    std::vector<Generator::path> rep;
    rep.reserve(p.p.size());
    std::vector<double> rw(p.w.size(),1.0/p.w.size());

    ///discrete_distribution
//  g->setDD(p.w);
//    for(size_t i=0;i<p.p.size();++i)
//    {
//        int index=g->rndDD();
//        rep.push_back(p.p[index]);
//    }


    std::vector<double> cm(p.p.size());
    std::partial_sum(p.w.begin(),p.w.end(),cm.begin());
    double sum_weights = cm.back();
    for (double& val : cm)
    {
        val /= sum_weights;
    }
    double u1=g->rndDouble(0.0,1.0/p.w.size());
    double step=1.0/p.w.size();
    double u =u1;
//    for(size_t i=0;i<p.p.size();++i)
//    {
//        double r=g->rndDouble(0.0,1.0);
//        auto it = std::lower_bound(cm.begin(), cm.end(), r);
//        int index = std::distance(cm.begin(), it);
//        rep.push_back(p.p[index]);
//    }
    int i=0;
    for (int j=0;j<p.w.size();++j)
    {
        while(u>cm[i])
        {
            ++i;
        }
        rep.push_back(p.p[i]);
        u+=step;
    }
    p.p=rep;
    p.w=rw;
    p.c1=rw;
    p.c2=rw;
    p.c3=rw;
    p.c4=rw;
    p.c5=rw;
    p.c6=rw;
    return;
}

bool Controller::updateParticle()
{
    double gGoal[SIZE_STATE];
    getGoal(gGoal,true);
    g->setGoal(gGoal);
    optimized_path.clear();
    double w1=1.5;
    double w2=10.0;
    double w3=2.0;
    double w4=3.0;
    double w5=1.0;
    double w6=0.5;
    //cros product, variance, close goal pos, to robot pos, to cp, sen
    double ess=calcESS();
    std::cout<<"ess   :"<<ess<<std::endl;
    g->setDD(p.w);
    int iiIndex=g->rndDD();
    auto min_itr=std::max_element(p.w.begin(),p.w.end());
    int iIndex=std::distance(p.w.begin(),min_itr);
//    double min_weight=p.w[min_index];
//    double sum_weights = std::accumulate(p.w.begin(), p.w.end(), 0.0,
//        [](double sum, const double& w) {
//            return sum + w;
//        });

    std::cout<<"cost1 : "<<w1*p.c1[iIndex]<<std::endl;
    std::cout<<"cost2 : "<<w2*p.c2[iIndex]<<std::endl;
    std::cout<<"cost3 : "<<w3*p.c3[iIndex]<<std::endl;
    std::cout<<"cost4 : "<<w4*p.c4[iIndex]<<std::endl;
    std::cout<<"cost5 : "<<w5*p.c5[iIndex]<<std::endl;
    std::cout<<"cost6 : "<<w6*p.c6[iIndex]<<std::endl;
    std::cout<<"c1~2  : "<<exp(-(w1*p.c1[iIndex]+w2*p.c2[iIndex]))<<std::endl;
    std::cout<<"c3~6  : "<<exp(w3*p.c3[iIndex]+w4*p.c4[iIndex]+w5*p.c5[iIndex]+w6*p.c6[iIndex])<<std::endl;
    std::cout<<"total : "<<exp(exp(-(w1*p.c1[iIndex]+w2*p.c2[iIndex]))+exp(w3*p.c3[iIndex]+w4*p.c4[iIndex]+w5*p.c5[iIndex]+w6*p.c6[iIndex]))<<std::endl;
    std::cout<<"px : "<<p.p[iIndex].px<<", py : "<<p.p[iIndex].py<<std::endl;
    double tw=exp(exp(-(w1*p.c1[iiIndex]+w2*p.c2[iiIndex]))+exp(w3*p.c3[iiIndex]+w4*p.c4[iiIndex]+w5*p.c5[iiIndex]+w6*p.c6[iiIndex]));
    if(!bFirst)
    if(tw<10.0)
    {
        double pos[SIZE_STATE]={p.p[iiIndex].px,p.p[iiIndex].py,p.p[iiIndex].pq};
        Generator* g_temp=new Generator(*g,*s,pos);
        g_temp->gen(Generator::prediction);
        optimized_path=g_temp->getPath();
        int arg=-1;
        normalizeCross(optimized_path,arg);
        if(arg!=-1)
        {
            double tg[SIZE_STATE];
            optimized_path.erase(optimized_path.begin(),optimized_path.begin()+arg);
            Generator* g_tg=new Generator(*g,*s,m_rPos);

            g_tg->m_rPath=optimized_path;
            double d =g_tg->calcTgoal();
            g_tg->getTgoal(tg);
            setTGoal(tg[INDEX_X],tg[INDEX_Y],tg[INDEX_Q],d);
            return true;
        }
    }
    tw=exp(exp(-(w1*p.c1[iiIndex]+w2*p.c2[iiIndex]))+exp(w3*p.c3[iiIndex]+w4*p.c4[iiIndex]+w5*p.c5[iiIndex]+w6*p.c6[iiIndex]));
    if(tw>10.0)
    {
        resample();
        moveParticle();
        int n_p=1;
        std::vector<Generator::path>n_particle= createParticle(n_p);
        for(int i=0;i<n_p;i++)
        {
            int sindex=0;
            int dindex=0;
            dindex=g->rndInt(p.p.size());
            sindex=g->rndInt(n_particle.size());
            p.p[dindex].px=n_particle[sindex].px;
            p.p[dindex].py=n_particle[sindex].py;
            p.p[dindex].pq=n_particle[sindex].pq;
            p.c1[dindex]=(1.0/p.p.size());
            p.c2[dindex]=(1.0/p.p.size());
            p.c3[dindex]=(1.0/p.p.size());
            p.c4[dindex]=(1.0/p.p.size());
            p.c5[dindex]=(1.0/p.p.size());
            p.c6[dindex]=(1.0/p.p.size());
            p.w[dindex] =(1.0/p.p.size());
        }
        g->setDD(p.w);
        for(int i=0;i<n_p;i++)
        {
            int dindex=0;
            dindex=g->rndDD();
            p.p[dindex].px+=g->rndDouble(-0.05,0.05);
            p.p[dindex].py+=g->rndDouble(-0.05,0.05);
            p.p[dindex].pq+=g->rndDouble(-0.5,0.5);
            p.c1[dindex]=(1.0/p.p.size());
            p.c2[dindex]=(1.0/p.p.size());
            p.c3[dindex]=(1.0/p.p.size());
            p.c4[dindex]=(1.0/p.p.size());
            p.c5[dindex]=(1.0/p.p.size());
            p.c6[dindex]=(1.0/p.p.size());
            p.w[dindex] =(1.0/p.p.size());
        }
    }
    calcCost();
    p.w=calculateWeight(w1,w2,w3,w4,w5,w6);
    bFirst=false;
    return false;
}

bool Controller::isArrived()
{
    bool ret=true;
    for(int i=0;i<goals.size();i++)
    {
        ret&=goals[i].arrived;
    }
    return ret;
}

void Controller::optimize(const double *pos, double *param, double *dst, double *cst1, double *cst2, double &loss)
{
    double delta=0.05;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.01;

    std::vector<double> cost1;
    std::vector<double> cost2;
    std::vector<double> l;

    double gGoal[SIZE_STATE];
    getGoal(gGoal,true);
    for(int i=0;i<2;i++)
    {
        Generator *pFuture;
        Generator *pAdd;
        Generator *mFuture;
        Generator *mAdd;

        double pPos[3]={pos[0],pos[1],pos[2]};
        double mPos[3]={pos[0],pos[1],pos[2]};

        pPos[i]+=delta;
        mPos[i]-=delta;

        pFuture=new Generator(*g,*s,pPos);
        mFuture=new Generator(*g,*s,mPos);
        pFuture->gen(Generator::prediction);
        mFuture->gen(Generator::prediction);

        pfPath=pFuture->getPath();
        mfPath=mFuture->getPath();

        double paPos[3]={pfPath.back().px,pfPath.back().py,pfPath.back().pq};
        double maPos[3]={mfPath.back().px,mfPath.back().py,mfPath.back().pq};

        pAdd=new Generator(*g,*s,paPos);
        mAdd=new Generator(*g,*s,maPos);

        pAdd->gen(Generator::stagnation);
        mAdd->gen(Generator::stagnation);

        paPath=pAdd->getPath();
        maPath=mAdd->getPath();

        double pc[2];//plus delta cost1,2
        double mc[2];//minus delta cost1,2
        double ploss;
        double mloss;
        double pl=cost(pfPath,paPath,pc,ploss);
        double ml=cost(mfPath,maPath,mc,mloss);
        gradient[i]=(pl-ml)/(2.0*delta);
        cost1.push_back(pc[0]);
        cost1.push_back(mc[0]);
        cost2.push_back(pc[1]);
        cost2.push_back(mc[1]);
        l.push_back(ploss);
        l.push_back(mloss);
    }
    for(int i=0;i<cost1.size();i++)
    {
        cst1[i]=cost1[i];
        cst2[i]=cost2[i];
    }
    auto min_itr=std::min_element(l.begin(),l.end());
    loss=*min_itr;
    dst[0]=param[0]-learning_rate*gradient[0];
    dst[1]=param[1]-learning_rate*gradient[1];
//    param[0]=dst[0];
//    param[1]=dst[1];
}

double Controller::cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double *cst, double &loss)
{
    double w1,w2;//weight
    double cost1,cost2;
    double mean_x,mean_y;
    double varianceX=0.0;
    double varianceY=0.0;

    w1=2.8;
    w2=10.0;

    //normalization
    int arg=0;
    cost1=normalizeCross(path,arg);

    mean_x=std::accumulate(aPath.end()-10,aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.px;});
    mean_x/=10;
    mean_y=std::accumulate(aPath.end()-10,aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.py;});
    mean_y/=10;

    double cost3=0.0;
    double cost4=0.0;
    double cost5=0.0;
    double gGoal[SIZE_STATE];
    getGoal(gGoal,true);
    std::vector<double> vdist(10);
    double d_o=sqrt(pow(gGoal[INDEX_X]-stag_pos[INDEX_X],2)+pow(gGoal[INDEX_Y]-stag_pos[INDEX_Y],2));
    double d_s=sqrt(pow(stag_pos[INDEX_X]-m_rPos[INDEX_X],2)+pow(stag_pos[INDEX_Y]-m_rPos[INDEX_Y],2))+g->ip.f_param.rparam.d_o;
    double d_a=sqrt(pow(path[0].px-m_rPos[INDEX_X],2)+pow(path[0].py-m_rPos[INDEX_Y],2));
    for(int i=aPath.size()-10;i<aPath.size();i++)
    {
        varianceX+=pow(aPath.at(i).px-mean_x,2);
        varianceY+=pow(aPath.at(i).py-mean_y,2);
        double dx=stag_pos[0]-aPath.at(i).px;
        double dy=stag_pos[1]-aPath.at(i).py;
        double dist=sqrt(dx*dx+dy*dy);
        if(dist<0.5)
            cost3-=0.0;
        dx=gGoal[INDEX_X]-aPath[i].px;
        dy=gGoal[INDEX_Y]-aPath[i].py;
        dist=sqrt(dx*dx+dy*dy);
        vdist[i-(aPath.size()-10)]=dist;
    }
    auto min_itr=std::min_element(vdist.begin(),vdist.end());
    double d_min=*min_itr;
    varianceX/=10.0;
    varianceY/=10.0;
    cost2=varianceX+varianceY;
    cost4=d_min/d_o;
    cost5=d_a/d_s;
    cst[0]=w1*cost1;
    cst[1]=w2*cost2+0.8*cost4;//-0.5*cost5;
    double px,py;
    px=path.front().px;
    py=path.front().py;
//    if(-(cst[0]+cst[1])<-0.6)
    //std::cout<<px<<", "<<py<<" : "<<w1*cost1<<", "<<w2*cost2+cost3-0.5*cost5<<", "<<-(cst[0]+cst[1])<<std::endl;
    loss=-(cst[0]+cst[1]);
    return -(cst[0]+cst[1]);
}

double Controller::normalizeCross(std::vector<Generator::path> path, int &argFeature)
{
    std::vector<double> x(path.size());
    std::vector<double> y(path.size());
    std::vector<double> diff_x(path.size()-1);
    std::vector<double> diff_y(path.size()-1);
    std::vector<double> distance(path.size()-1);
    std::vector<double> normalized_distance(path.size()-1);
    std::vector<double> normalized_x(path.size()-1);
    std::vector<double> normalized_y(path.size()-1);
    double total_distance=0.0;
    for(int i=0;i<path.size();i++)
    {
        x[i]=path.at(i).px;
        y[i]=path.at(i).py;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        diff_x[i]=x[i+1]-x[i];
        diff_y[i]=y[i+1]-y[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        distance[i]=sqrt(pow(x[i+1]-x[i],2)+pow(y[i+1]-y[i],2));
    }
    for(int i=0;i<path.size()-1;i++)
    {
        total_distance+=distance[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_distance[i]=distance[i]/total_distance;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_x[i]=normalized_distance[i]*diff_x[i]/distance[i];
        normalized_y[i]=normalized_distance[i]*diff_y[i]/distance[i];
    }
    double cum_sum_x=0.0;
    double cum_sum_y=0.0;
    double ret=0.0;
    for(int i=0;i<path.size()-1;i++)
    {
        cum_sum_x+=normalized_x[i];
        cum_sum_y+=normalized_y[i];
        x[i]=cum_sum_x;
        y[i]=cum_sum_y;
    }
    bool bFirst=true;
    for (int i=0;i<path.size()-2;i++)
    {
        double cross_product =x[i]*y[i+1]-x[i+1]*y[i];
        ret+=abs(cross_product);
        if(bFirst)
        {
            if(abs(cross_product)>0.001)
            {
                argFeature=i;
                bFirst=false;
            }
        }
    }
    return ret;
}

void Controller::initConState(double *pos)
{
    m_rPos[INDEX_X]=pos[INDEX_X];
    m_rPos[INDEX_Y]=pos[INDEX_Y];
    m_rPos[INDEX_Q]=pos[INDEX_Q];
    g->setPos(m_rPos);
}

void Controller::setOutput(double *v)
{
    con_vel[INDEX_LINEAR]=v[INDEX_LINEAR];
    con_vel[INDEX_ANGULAR]=v[INDEX_ANGULAR];
}

void Controller::selectBeam(int& dst, bool direction, int src)
{
    int num_s=s->ip.sparam.num_sensors;
    double dx=stag_pos[INDEX_X]-m_rPos[INDEX_X];
    double dy=stag_pos[INDEX_Y]-m_rPos[INDEX_Y];
    double radius=sqrt(dx*dx+dy*dy)+1.2*g->ip.f_param.rparam.d_o;

    std::vector<double> ds(num_s);//stag dist from sensor line

    for(int i=0;i<num_s;i++)
    {
        double sx,sy;
        double q;
        double a,b,c,d;
        double vx,vy;
        sx=s->is[i].sense.vx-m_rPos[INDEX_X];
        sy=s->is[i].sense.vy-m_rPos[INDEX_Y];
        q=s->is[i].pos.q;
        vx=cos(q);
        vy=sin(q);
        if(stag_pos[INDEX_X]*vx+stag_pos[INDEX_Y]*vy<0)
        {
            ds[i]=10.0;
            continue;
        }
        a=s->is[i].pos.q;
        b=-1.0;
        c=(-a*m_rPos[INDEX_X]+m_rPos[INDEX_Y]);
        d=abs(a*stag_pos[INDEX_X]+b*stag_pos[INDEX_Y]+c)/sqrt(a*a+b*b);
        ds[i]=d;
    }
    auto min_itr=std::min_element(ds.begin(),ds.end());
    int min_index = std::distance(ds.begin(), min_itr);
    if(src!=-1)
    {
        min_index=src;
    }
    if(direction)//min_index to index of heading beam
    {
        for(int i=min_index;i>=0;i--)
        {
            double sen=s->is[i].sense.dist;
            if(sen>radius)
            {
                dst=i;
                break;
            }
            if(sen<0.0)
            {
                dst=i;
                break;
            }
        }
    }
    else
    {
        for(int i=min_index;i<num_s;i++)
        {
            double sen=s->is[i].sense.dist;
            if(sen>radius)
            {
                dst=i;
                break;
            }
            if(sen<0.0)
            {
                dst=i;
                break;
            }
        }
    }
}

void Controller::genSample(double *point1, double *point2, double *dst)
{
    double t=g->addRealNoise(0,1);
    double x=point1[INDEX_X]+t*(point2[INDEX_X]-point1[INDEX_X]);
    double y=point1[INDEX_Y]+t*(point2[INDEX_Y]-point1[INDEX_Y]);
    dst[INDEX_X]=g->addNoise(x,0.05);
    dst[INDEX_Y]=g->addNoise(y,0.05);
}





















