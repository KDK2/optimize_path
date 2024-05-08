#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "robot.h"
#include <thread>
#include <condition_variable>
#include <mutex>
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void startRobot(Robot&);
    void start();

private:
    Ui::MainWindow *ui;
    Robot r;
    std::thread m_worker;
    std::mutex  m_mutex;
    bool m_bRunning;
    bool bReady;

    void worker(Robot&);
    void stopWorker();
};
#endif // MAINWINDOW_H
