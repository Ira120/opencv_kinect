#ifndef APPLICATION_H
#define APPLICATION_H

#include "EdgeDetection.h"
#include "BackProjection.h"

class Application
{
public:
    Application();
    ~Application();

    int frame_nr;

    int initKinect();
    void showModel();
    int frameLoop(VideoCapture);

private:
    EdgeDetection detector;
    BackProjection projection;
};

#endif // APPLICATION_H
