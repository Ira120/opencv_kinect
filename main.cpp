#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include "Tools.h"
#include "EdgeDetection.h"
#include "BackProjection.h"
#include "Application.h"

#include <vector>
#include <iostream>


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "patterndetector.h"
#include <stdio.h>
#include "cameraparams.h"


using namespace std;
using namespace cv;
using namespace ARma;

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 1// define the number of patterns you want to use

int main() {
/*
    //test inverse
    Mat rotMat = Mat::eye(3, 3, CV_32F);
    rotMat.at<float>(0,0) = 2;
    rotMat.at<float>(1,0) = 56;
    cout<<rotMat<<endl;
    Mat inverse = rotMat.inv();
    cout<<"inverse: "<<inverse<<endl;


    //test Punkt
    Point3f punkt(2.0f,2.0f,2.0f);
    Point3f punkt1(1.0f,-4.0f,2.0f);
    Point3f punkt2 = punkt-punkt1;

    Vec3f neue = Vec<float,3>(punkt1);
    cout << "neue: "<<neue<<endl;

    //Vec3f tex;
   //Mat tex = inverse.mul(neue);
    Mat result = inverse*Mat(neue);

    cout << "neue: "<<result<<endl;
*/

    Application app;

    app.initPattern();
    app.initKinect();





}



