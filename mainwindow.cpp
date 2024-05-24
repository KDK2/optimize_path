#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qcustomplot.h"
#include "ros/ros.h"
MainWindow::MainWindow(int argc, char **argv,QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_bRunning(false)
    , bReady(false)
{
    ui->setupUi(this);
    this->resize(1000,600);
    //ui->centralwidget->resize(800,500);
    ui->gridLayoutWidget->resize(1000,500);
    ui->widget->resize(800,400);
    ui->widget_2->resize(200,200);

    connect(&timer, &QTimer::timeout, this, &MainWindow::slotTimeout);
    timer.start(1);

    tout.SetFPS(100);
    tout.SetSender("cmd_vel");
    tout.onLoop=onROSVel;
    tout.Init(argc,argv,"cm",this);
    r.initRos(argc,argv);
}

MainWindow::~MainWindow()
{
    stopWorker();
    delete ui;
}

void MainWindow::startRobot(Robot &robot)
{
    if(!m_bRunning)
    {
        m_bRunning=true;
        m_worker=std::thread(&MainWindow::worker,this,std::ref(robot));
    }
}

void MainWindow::start()
{
    r.start();
    startRobot(r);
}

void MainWindow::onROSVel(void *pArg)
{
    MainWindow* pMF = (MainWindow*)pArg;
    if(!pMF->m_bRunning)
        return;

    geometry_msgs::Twist cmd;
    cmd.linear.x=pMF->m_control_data.v;
    cmd.angular.z=pMF->m_control_data.w;
    pMF->tout.Send(cmd);
}

void MainWindow::slotTimeout()
{
    updateUi();
}

void MainWindow::worker(Robot &robot)
{
    while (m_bRunning)
    {
        std::vector<int> idata;
        std::vector<double> ddata;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            robot.m_condition.wait(lock, [&](){return robot.isDataUpdated();});

            idata = robot.getiData();
            ddata = robot.getdData();
            m_localmap_data.lData=robot.getbData();
        }
        updateRobotData(idata, ddata);
        bReady=true;
        robot.setDataUpdated(false);
    }//
}

void MainWindow::stopWorker()
{
    m_bRunning=false;
    if(m_worker.joinable())
    {
        m_worker.join();
    }
}

void MainWindow::updateRobotData(std::vector<int> iData, std::vector<double> dData)
{
    m_obs.num_path=iData[0];
    m_sensor.num_sensors=iData[1];
    m_obs.num_optimized=iData[2];
    m_obs.num_optimized_path=iData[3];
    m_localmap_data.h=iData[4];
    m_localmap_data.w=iData[5];
    m_iArrived=iData[6];
    m_particle.iSize=iData[7];

    int index=0;
    m_sensor_data.x.clear();
    m_sensor_data.y.clear();
    m_sensor_data.q.clear();
    m_sensor_data.dist.clear();
    m_sensor_data.vx.clear();
    m_sensor_data.vy.clear();
    for(int i=0;i<m_sensor.num_sensors;i++)
    {
        m_sensor_data.x.push_back(dData[index++]);
        m_sensor_data.y.push_back(dData[index++]);
        m_sensor_data.q.push_back(dData[index++]);
        m_sensor_data.dist.push_back(dData[index++]);
        m_sensor_data.vx.push_back(dData[index++]);
        m_sensor_data.vy.push_back(dData[index++]);
    }
    m_path_data.px.clear();
    m_path_data.py.clear();
    for(int i=0;i<m_obs.num_path;i++)
    {
        m_path_data.px.push_back(dData[index++]);
        m_path_data.py.push_back(dData[index++]);
    }
    for(int i=0;i<m_obs.num_optimized_path;i++)
    {
        m_path_data.px.push_back(dData[index++]);
        m_path_data.py.push_back(dData[index++]);
    }
    m_optimized_data.x.clear();
    m_optimized_data.y.clear();
    for(int i=0;i<m_obs.num_optimized;i++)
    {
        m_optimized_data.x.push_back(dData[index++]);
        m_optimized_data.y.push_back(dData[index++]);
    }
    m_particle.x.clear();
    m_particle.y.clear();
    for(int i=0;i<m_particle.iSize;i++)
    {
        m_particle.x.push_back(dData[index++]);
        m_particle.y.push_back(dData[index++]);
    }
    m_particle.w.clear();
    for(int i=0;i<m_particle.iSize;i++)
    {
        m_particle.w.push_back(dData[index++]);
    }
    m_sensor.max_dist=dData[index++];
    m_robot_data.x=dData[index++];
    m_robot_data.y=dData[index++];
    m_robot_data.q=dData[index++];
    m_robot_data.goal_x=dData[index++];
    m_robot_data.goal_y=dData[index++];
    m_robot_data.radius=dData[index++];
    m_robot_data.dt=dData[index++];

    m_control_data.v=dData[index++];
    m_control_data.w=dData[index++];

    m_localmap_data.lPos[0]=dData[index++];
    m_localmap_data.lPos[1]=dData[index++];
    m_localmap_data.lPos[2]=dData[index++];

//    delete image;
    image = new QImage(m_localmap_data.h,m_localmap_data.w,QImage::Format_Grayscale8);
    std::vector<BYTE> bData=m_localmap_data.lData;
    memcpy(image->bits(), bData.data(), bData.size() * sizeof(BYTE));
}

double calcEntropy(QVector<double> &weights)
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
void MainWindow::updateUi()
{
    if(!bReady) return;
    bReady=false;

    int num_sensor=m_sensor.num_sensors;
    int iPath=m_obs.num_path+m_obs.num_optimized_path;
    int iOptimized=m_obs.num_optimized;
    int iParticle=m_particle.iSize;
    double max_dist=m_sensor.max_dist;
    QVector<QCPCurveData> robot_sensor(num_sensor+1);
    QVector<QVector<QCPCurveData>> pth(iPath);
    QVector<QVector<QCPCurveData>> optimized(iOptimized);
    QVector<QVector<QCPCurveData>> particle(iParticle);
    QVector<QCPCurveData> goal;
    QVector<QCPCurve*> fermatPath;
    QVector<QCPCurve*> fermatrobot;
    QVector<QCPCurve*> fermatOptimized;
    //QVector<QCPCurve*> fermatParticle;
    QVector<QCPGraph*> fermatParticle;
    QCPCurve* fermatGoal;
    QVector<QCPItemLine*> sensor_line;
    QCustomPlot *customPlot = ui->widget;
    QCustomPlot *customPlot2 = ui->widget_2;
    QCPCurve *fermatCircle = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    QCPItemPixmap *_pixmap = new QCPItemPixmap(customPlot);
    QPixmap pixmap = QPixmap::fromImage(image->scaled(customPlot->width(),customPlot->height(),Qt::IgnoreAspectRatio, Qt::FastTransformation));
    _pixmap->setVisible(true);
    _pixmap->setScaled(true);
    _pixmap->setPixmap(pixmap);

    _pixmap->topLeft->setCoords(m_robot_data.x-4.5,m_robot_data.y+4.5);
    _pixmap->bottomRight->setCoords(m_robot_data.x+4.5,m_robot_data.y-4.5);

    for(int i=0;i<1;i++)
    {
        double r =m_robot_data.radius;
        double d=m_sensor_data.dist[i];
        double x=m_sensor_data.x[i];
        double y=m_sensor_data.y[i];
        double q=m_sensor_data.q[i];
        if(d<0) d=max_dist;
        QCPItemLine* line = new QCPItemLine(customPlot);
        line->start->setCoords(x,y);
        line->end->setCoords(x-r*cos(q),y-r*sin(q));
        sensor_line.append(line);
        if(i==0)
            sensor_line[i]->setPen(QPen(Qt::red));
        else
            sensor_line[i]->setPen(QPen(Qt::blue));
        robot_sensor[i]=QCPCurveData(i, x, y);
    }
    robot_sensor[num_sensor]=QCPCurveData(num_sensor, m_sensor_data.x[0], m_sensor_data.y[0]);
    sensor_line[0]->setPen(QPen(Qt::red));
    fermatCircle->data()->clear();
    fermatCircle->data()->add(robot_sensor,true);
    fermatCircle->setPen(QPen(Qt::blue));

    for(int i=0; i<iPath;i++)//iPath
    {
        QCPCurve* path_curve = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        fermatPath.append(path_curve);
        double px=m_path_data.px[i];
        double py=m_path_data.py[i];
        double r =m_robot_data.radius;
        for(int j=0;j<num_sensor;j++)
        {
            pth[0].append(QCPCurveData(j, px+r*cos(2.0*M_PI*j/(num_sensor-1)), py+r*sin(2*M_PI*j/(num_sensor-1))));
        }
        QVector<QCPCurveData> path_data=pth.takeFirst();
        fermatPath.at(i)->data()->add(path_data,true);
        fermatPath.at(i)->setPen(QPen(Qt::yellow));
//        m_robot_path.x.push_back(px);
//        m_robot_path.y.push_back(py);
    }
    if(m_iArrived==1)
    {
        QVector<QVector<QCPCurveData>> rp(m_robot_path.x.size());
        for(int i=0;i<m_robot_path.x.size();i++)
        {
            QCPCurve* path_robot = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
            fermatrobot.append(path_robot);
            double px=m_robot_path.x[i];
            double py=m_robot_path.y[i];
            double r =m_robot_data.radius;
            for(int j=0;j<num_sensor;j++)
            {
                rp[0].append(QCPCurveData(j, px+r*cos(2.0*M_PI*j/(num_sensor-1)), py+r*sin(2*M_PI*j/(num_sensor-1))));
            }
            QVector<QCPCurveData> robot_data=rp.takeFirst();
            fermatrobot.at(i)->data()->add(robot_data,true);
            fermatrobot.at(i)->setPen(QPen(Qt::red));
        }
    }
    for(int i=0;i<iOptimized;i++)
    {
        QCPCurve* optimized_curve = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        fermatOptimized.append(optimized_curve);
        double x=m_optimized_data.x[i];
        double y=m_optimized_data.y[i];
        double r=m_robot_data.radius-0.2;
        for(int j=0;j<num_sensor;j++)
        {
            optimized[0].append(QCPCurveData(j, x+r*cos(2.0*M_PI*j/(num_sensor-1)), y+r*sin(2*M_PI*j/(num_sensor-1))));
        }
        QVector<QCPCurveData> optimized_data=optimized.takeFirst();
        fermatOptimized.at(i)->data()->add(optimized_data,true);
        fermatOptimized.at(i)->setPen(QPen(Qt::magenta));
    }
    QVector<double> qx = QVector<double>::fromStdVector(m_particle.x);
    QVector<double> qy = QVector<double>::fromStdVector(m_particle.y);
    QPen ppen;
    ppen.setColor(Qt::green);
    ppen.setWidth(0.5);
    QCPGraph* particle_scatter=new QCPGraph(customPlot->xAxis,customPlot->yAxis);
    particle_scatter->setData(qx,qy);
    particle_scatter->setPen(ppen);
    particle_scatter->setLineStyle(QCPGraph::lsNone);
    particle_scatter->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ScatterShape::ssStar));


    fermatGoal = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    double gx=m_robot_data.goal_x;
    double gy=m_robot_data.goal_y;
    double gr=m_robot_data.radius;
    for(int j=0;j<num_sensor;j++)
    {
        goal.append(QCPCurveData(j, gx+gr*cos(2.0*M_PI*j/(num_sensor-1)), gy+gr*sin(2*M_PI*j/(num_sensor-1))));
    }
    fermatGoal->data()->add(goal,true);
    fermatGoal->setPen(QPen(Qt::red));

    int bins=m_particle.iSize;
    double min_w;
    double max_w;
    min_w=*std::min_element(m_particle.w.begin(), m_particle.w.end());
    max_w=*std::max_element(m_particle.w.begin(), m_particle.w.end());
    double binWidth=(max_w-min_w)/(double)bins;
    QVector<double> vBins(bins,0);
    if(!(binWidth<0.0000001))
    {
        for(double weight:m_particle.w)
        {
            int binIndex = std::min(bins-1, static_cast<int>((weight-min_w) / binWidth));
            vBins[binIndex]++;
        }
        QVector<double> probabilities = vBins;
        for(double& value : probabilities)
        {
            value/=m_particle.iSize;
        }
        std::cout<<"entropy!! : "<<calcEntropy(probabilities)<<std::endl;
        QVector<double> x(bins);
        QVector<double> y = probabilities;
        for(int i=0;i<bins;++i)
        {
            x[i]=min_w +i*binWidth+binWidth/2; // Center of the bin
        }

        QCPBars *bars = new QCPBars(customPlot2->xAxis, customPlot2->yAxis);
        bars->setWidth(0.0003);
        bars->setData(x, y);
        //double min_x=*std::min_element(x.begin(),x.end());
        double max_x=*std::max_element(x.begin(),x.end());
        if(x_range<max_x)
            x_range=max_x;
        customPlot2->xAxis->setRange(0.0,x_range);
        customPlot2->yAxis->setRange(0,1.0);
        customPlot2->replot();
        customPlot2->clearPlottables();
        customPlot2->clearItems();
    }

    customPlot->addGraph();
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    customPlot->xAxis->setRange(m_robot_data.x-5.0, m_robot_data.x+5.0);
    customPlot->yAxis->setRange(m_robot_data.y-5.0, m_robot_data.y+5.0);
    // customPlot->xAxis->setRange(0.0, 10.0);
    // customPlot->yAxis->setRange(0.0, 10.0);

    customPlot->replot();
    customPlot->clearPlottables();
    customPlot->clearItems();
}

void MainWindow::on_pushButton_clicked()
{
    start();
}
