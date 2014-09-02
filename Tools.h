#ifndef TOOLS_H
#define TOOLS_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

/** \brief A collection of useful features in OpenCV with Kinect
 *
 */
namespace Tools {

    int saveFramesFromKinect(VideoCapture &capture);

}

#endif // TOOLS_H
