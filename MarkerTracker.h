#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H

#define PAT_SIZE 120//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 1// define the number of patterns you want to use

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include "Patterndetector.h"
#include "Log.h"

using namespace cv;
using namespace std;
using namespace ARma;

class MarkerTracker{

public:
    //parameters
    string log;
    Mat cameraMatrix_depth;
    Mat distortions_depth;

    //functions
    MarkerTracker();
    ~MarkerTracker();
    void loadCameraParams();
    int loadPattern(const String& filename, vector<Mat>& library, int& patternCount);
    Mat getCameraMatrix();
    Mat getDistortions();
    vector<Mat> createPatternLib();

private:
    //parameters
    Mat cameraMatrix;
    Mat distortions;
};

#endif


