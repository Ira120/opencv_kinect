#ifndef EDGEDETECTION_H
#define EDGEDETECTION_H

#include "Tools.h"
#include "Line3D.h"
#include "Log.h"

class EdgeDetection {
public:
    //parameters
    string log;

    //methods
    EdgeDetection();
    ~EdgeDetection();
    Mat smoothGauss (Mat image, int value);
    Mat smoothMedian (Mat image, int value);
    Mat smoothBilateral (Mat image, int value);
    vector<Vec4i> applyHoughTransformation (Mat imageOriginal, int frame_nr);
    vector<Vec4i> applyLSD(Mat imageOriginal, int frame_nr);

private:
    //parameters
    Mat imageCanny;
    Mat imageGray;
    char filename[200];
    vector<Vec4i> lines;
};

#endif // EDGEDETECTION_H
