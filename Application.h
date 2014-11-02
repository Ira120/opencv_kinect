#ifndef APPLICATION_H
#define APPLICATION_H

#include "EdgeDetection.h"
#include "BackProjection.h"
#include "MarkerTracker.h"
#include "Patterndetector.h"
#include "Line3D.h"
#include "EdgeModel.h"
#include "Tools.h"
#include "GroupLines3D.h"

class Application {
public:
    //parameters
    string log;
    VideoCapture capture;
    int frame_nr;
    vector<Mat> patternLibrary;
    PatternDetector myDetector;
    Mat rotMat;
    Vec3f transVec;
    Mat transVecMat;
    Mat rotVecMat;
    bool detectedPats;

    //methods
    Application();
    ~Application();
    void initPattern();
    int initKinect();
    int frameLoop();
    void detectPattern(Mat rgbImage);
    void calculate3DLines();
    void showLines3DInFrame(Mat rgbImage);

private:
    //parameters
    EdgeDetection edgeDetector;
    BackProjection projection;
    MarkerTracker patternLoader;
    EdgeModel edgeModel;
    GroupLines3D groupLines;
};

#endif // APPLICATION_H
