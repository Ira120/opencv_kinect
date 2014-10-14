#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "patterndetector.h"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace ARma;

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 1// define the number of patterns you want to use

class cameraparams{

public:
    cameraparams();
    ~cameraparams();
    void loadCameraParams();
    int loadPattern(const String& filename, vector<Mat>& library, int& patternCount);
    Mat getCameraMatrix();
    Mat getDistortions();
    vector<Mat> createPatternLib();

private:
    Mat cameraMatrix;
    Mat distortions;
};


#endif


