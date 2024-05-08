#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
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

void MainWindow::worker(Robot &robot)
{
    //
}

void MainWindow::stopWorker()
{
    m_bRunning=false;
    if(m_worker.joinable())
    {
        m_worker.join();
    }
}
