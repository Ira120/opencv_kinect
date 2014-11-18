#ifndef TOOLS_H
#define TOOLS_H

#include <QFile>
#include <QTextStream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <glm.hpp>
#include <gtc/constants.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <gtc/matrix_inverse.hpp>
#include "Log.h"

#include <iostream>
#include <stdio.h>
#include <vector>

using namespace cv;
using namespace std;
using namespace glm;

/** \brief A collection of useful features in OpenCV with Kinect
 *
 */
namespace Tools {

    int saveFramesFromKinect(VideoCapture &capture);
    float modalValue (vector<float> values);

}

#endif // TOOLS_H
