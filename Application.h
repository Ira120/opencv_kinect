#ifndef APPLICATION_H
#define APPLICATION_H

#include "EdgeDetection.h"
#include "BackProjection.h"
#include "cameraparams.h"
#include "patterndetector.h"
#include "Line3D.h"
#include "EdgeModel.h"

class Application {
public:
    int frame_nr;
    vector<Mat> patternLibrary;
    PatternDetector myDetector;
    Mat rotMat;
    Vec3f transVec;


    Application();
    ~Application();
    void initPattern();
    int initKinect();
    int frameLoop(VideoCapture);
    void detectPattern(Mat rgbImage);
    void calculate3DLines();

private:
    EdgeDetection edgeDetector;
    BackProjection projection;
    cameraparams patternLoader;
    EdgeModel edgeModel;
};

#endif // APPLICATION_H
