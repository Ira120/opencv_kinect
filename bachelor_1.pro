#-------------------------------------------------
#
# Project created by QtCreator 2014-08-20T13:31:40
#
#-------------------------------------------------

QT       += core gui
QT       += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = bachelor_1
TEMPLATE = app


SOURCES += main.cpp \
    Tools.cpp \
    EdgeDetection.cpp \
    BackProjection.cpp \
    Line3D.cpp \
    Application.cpp


macx: LIBS += -L/Users/irina/Develop/Kinect/OpenNI-Bin-Dev-MacOSX-v1.5.7.10/Lib -lnimCodecs -lnimMockNodes -lnimRecorder -lOpenNI
macx: LIBS += -L/Users/irina/Develop/Kinect/SensorKinect-unstable/Bin/Sensor-Bin-MacOSX-v5.1.2.1/Lib -lXnCore -lXnDDK -lXnDeviceFile -lXnDeviceSensorV2KM -lXnFormats
INCLUDEPATH += /usr/include/ni
INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include/ni

macx: LIBS += -L/usr/local/lib -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab
INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include

INCLUDEPATH += /Users/irina/Develop/Kinect/glm/glm

HEADERS += \
    Tools.h \
    EdgeDetection.h \
    BackProjection.h \
    Line3D.h \
    Application.h
