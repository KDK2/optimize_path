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
    b_path.push_back({m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]});
    detectLocalminimum(bDetect);
    setState(bDetect);
    planing();
    moveGoal();
}

void Controller::detectLocalminimum(bool &bLocalminimum)
{
    Generator* pRef=nullptr;
    Generator* pLocal=nullptr;
    double target[SIZE_STATE];
    double lastPredict[SIZE_STATE];
    double pos[SIZE_STATE]={m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]};//global pos

    getGoal(target,true);
    pRef=new Generator(*g,pos);
    pRef->setGoal(target);
    pRef->gen(Generator::prediction);

    lastPredict[INDEX_X]=pRef->getPath().back().px;
    lastPredict[INDEX_Y]=pRef->getPath().back().py;
    lastPredict[INDEX_Q]=pRef->getPath().back().pq;

    pLocal=new Generator(*g,lastPredict);
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
        if     (idle==state)         state=localminimum;
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
        ref=new Generator(*g,m_rPos);
        ref->gen(Generator::prediction);
        d=ref->calcTgoal();
        ref->getTgoal(tg);
        setTGoal(tg[INDEX_X],tg[INDEX_Y],tg[INDEX_Q],d);
        return;
    }
    else if(localminimum==state)
    {
        o.clear();
        optimized_path.clear();

        int sgd_iter=200;
        double oPos[SIZE_STATE]={m_rPos[INDEX_X],m_rPos[INDEX_Y],m_rPos[INDEX_Q]};
        double param[2]={0.0,0.0};
        std::vector<optimized_data>temp_o(sgd_iter);
        for(int i=0;i<sgd_iter;i++)
        {
            double temp[2];
            double dst[2];
            temp[INDEX_X]=g->addNoise(oPos[INDEX_X],0.1);
            temp[INDEX_Y]=g->addNoise(oPos[INDEX_Y],0.1);
            optimize(temp,param,dst,temp_o[i].cost1,temp_o[i].cost2,temp_o[i].loss);
            oPos[INDEX_X]=dst[INDEX_X];
            oPos[INDEX_Y]=dst[INDEX_Y];
            temp_o[i].x=oPos[INDEX_X];
            temp_o[i].y=oPos[INDEX_Y];
            o.push_back(temp_o[i]);
        }
        if(o.size()>0)
        {
            std::vector<optimized_data> to;
            for(int i=0;i<o.size();i++)
            {
                if(o[i].loss<-1.2)
                    to.push_back(o[i]);
            }
            if(to.size()==0)
            {
                std::cout<<"no optimized data"<<std::endl;
                o.clear();
                return;
            }
            auto minIt = std::min_element(to.begin(), to.end(),
                                          [](const optimized_data& a, const optimized_data& b){return a.loss < b.loss;});
            int minIndex = std::distance(to.begin(), minIt);
            if(minLoss>to[minIndex].loss)
            {
                minLoss=to[minIndex].loss;
                o.clear();
                o.push_back(to[minIndex]);
            }
            else
            {
                o.clear();
                return;
            }
            double op_pos[SIZE_STATE]={to[minIndex].x,to[minIndex].y,0.0};
            double gGoal[SIZE_STATE];
            double tGoal[SIZE_STATE];
            getGoal(gGoal,true);
            ref=new Generator(*g,op_pos);
            ref->setGoal(gGoal);
            ref->gen(Generator::prediction);
            optimized_path=ref->getPath();
            std::cout<<"optimized first : "<<optimized_path.at(0).px<<", "<<optimized_path.at(0).py<<std::endl;
            int argValid=-1;
            normalizeCross(optimized_path,argValid);
            if(argValid==-1)
            {
                minLoss=0.0;
                o.clear();
                return;
            }
            optimized_path.erase(optimized_path.begin(),optimized_path.begin()+argValid);
            ref->m_rPath=optimized_path;
            d=ref->calcTgoal();
            ref->getTgoal(tGoal);
            setTGoal(tGoal[INDEX_X],tGoal[INDEX_Y],tGoal[INDEX_Q],d);
            std::cout<<"rPos x : "<<m_rPos[0]<<", rPos y : "<<m_rPos[1]<<std::endl;
            std::cout<<"loss : "<<to[minIndex].loss<<std::endl;
            std::cout<<"goal: "<<tg[0]<<", "<<tg[1]<<std::endl;
            std::cout<<"min_opos: "<<to[minIndex].x<<", "<<to[minIndex].y<<std::endl;
            std::cout<<"last_opos: "<<param[0]<<", "<<param[1]<<std::endl;
            std::cout<<"last_loss: "<<temp_o[sgd_iter-1].loss<<std::endl;
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
    double delta=0.1;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.05;

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

        pFuture=new Generator(*g,pPos);
        mFuture=new Generator(*g,mPos);
        pFuture->gen(Generator::prediction);
        mFuture->gen(Generator::prediction);

        pfPath=pFuture->getPath();
        mfPath=mFuture->getPath();

        double paPos[3]={pfPath.back().px,pfPath.back().py,pfPath.back().pq};
        double maPos[3]={mfPath.back().px,mfPath.back().py,mfPath.back().pq};

        pAdd=new Generator(*g,paPos);
        mAdd=new Generator(*g,maPos);

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
    dst[0]=pos[0]-learning_rate*gradient[0];
    dst[1]=pos[1]-learning_rate*gradient[1];
    param[0]=dst[0];
    param[1]=dst[1];
}

double Controller::cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double *cst, double &loss)
{
    double w1,w2;//weight
    double cost1,cost2;
    double mean_x,mean_y;
    double varianceX=0.0;
    double varianceY=0.0;

    w1=0.8;
    w2=10.0;

    //normalization
    int arg=0;
    cost1=normalizeCross(path,arg);

    mean_x=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.px;});
    mean_x/=aPath.size();
    mean_y=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.py;});
    mean_y/=aPath.size();

    double cost3=0.0;
    for(int i=0;i<aPath.size();i++)
    {
        varianceX+=pow(aPath.at(i).px-mean_x,2);
        varianceY+=pow(aPath.at(i).py-mean_y,2);
        double dx=stag_pos[0]-aPath.at(i).px;
        double dy=stag_pos[1]-aPath.at(i).py;
        double dist=sqrt(dx*dx+dy*dy);
        if(dist<0.5)
            cost3-=0.5;
    }
    varianceX/=aPath.size();
    varianceY/=aPath.size();
    cost2=varianceX+varianceY;
    cst[0]=w1*cost1;
    cst[1]=w2*cost2+cost3;
    double px,py;
    px=path.front().px;
    py=path.front().py;
    if(-(cst[0]+cst[1])<-0.6)
        std::cout<<px<<", "<<py<<" : "<<w1*cost1<<", "<<w2*cost2+cost3<<", "<<-(cst[0]+cst[1])<<std::endl;
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
            if(abs(cross_product)>0.01)
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
}

void Controller::setOutput(double *v)
{
    con_vel[INDEX_LINEAR]=v[INDEX_LINEAR];
    con_vel[INDEX_ANGULAR]=v[INDEX_ANGULAR];
}
