QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH+=../../lib
LIBS+= -lrt
# ROS
INCLUDEPATH+=/opt/ros/noetic/include    # --> QRos.h
LIBS+=-L/opt/ros/noetic/lib
LIBS+= -lrostime -lroscpp -ltf2_ros -lrosconsole -lroscpp_serialization
LIBS+= -lxmlrpcpp -lactionlib -ltf2 -lrosconsole_backend_interface
LIBS+= -lrosconsole_log4cxx -lcpp_common
LIBS+= -lactionlib -lboost_thread

SOURCES += \
    ../../lib/3d/dhMat.cpp \
    ../../lib/3d/dhQuat.cpp \
    ../../lib/3d/dhVector.cpp \
    ../../lib/Elapsed.cpp \
    ../../lib/GTimer.cpp \
    ../../lib/KDTp.cpp \
    ../../lib/PTRList.cpp \
    ../../lib/SMMgr.cpp \
    ../../lib/ShMem.cpp \
    ../../lib/afx.cpp \
    ../../lib/vMutex.cpp \
    ../../lib/vParser.cpp \
    ../../lib/vSerial.cpp \
    ../../lib/vShell.cpp \
    ../../lib/vString.cpp \
    ../../lib/vToken.cpp \
    controller.cpp \
    generator.cpp \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp\
    robot.cpp \
    sensor.cpp

HEADERS += \
    ../../lib/3d/dhMat.h \
    ../../lib/3d/dhQuat.h \
    ../../lib/3d/dhVector.h \
    ../../lib/Elapsed.h \
    ../../lib/GTimer.h \
    ../../lib/KDTp.h \
    ../../lib/NetInfo.h \
    ../../lib/PTRList.h \
    ../../lib/QAct.h \
    ../../lib/QRos.h \
    ../../lib/QTF.h \
    ../../lib/QTF2.h \
    ../../lib/SMMgr.h \
    ../../lib/ShMem.h \
    ../../lib/stdafx.h \
    ../../lib/vArray.h \
    ../../lib/vMutex.h \
    ../../lib/vParser.h \
    ../../lib/vSerial.h \
    ../../lib/vShell.h \
    ../../lib/vString.h \
    ../../lib/vToken.h \
    ../../lib/xRGB.h \
    controller.h \
    generator.h \
    mainwindow.h \
    qcustomplot.h\
    robot.h \
    sensor.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
