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
#include <fstream>

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
    vector<Point2f> pattern_origin;

    //methods
    Application();
    ~Application();
    void initPattern();
    int initKinect();
    int frameLoop();
    void detectPattern(Mat rgbImage);
   // vector<Line3D> showCam3DLines(vector<Point3f> cam_lines3D);
    vector<Line3D> calculate3DLines(vector<Point3f> cam_lines3D);
    void showLines3DInFrame(vector<Line3D> lines3DproFrame,Mat rgbImage);
    Mat smoothDepthMap(Mat depthMapWithoutROI,int innerThreshold);
   // vector<Line3D> calculateExtrinsicNew(vector<Point3f> cam_lines3D);

    //manual input
    int initManualInput();
    void detectPatternManual(Mat rgbImage);
    vector<Line3D> calculate3DLinesManual(Mat modelView,vector<Point3f> cam_lines3D);


private:
    //parameters
    EdgeDetection edgeDetector;
    BackProjection projection;
    MarkerTracker patternLoader;
    EdgeModel edgeModel;
    GroupLines3D groupLines;
};

#endif // APPLICATION_H
