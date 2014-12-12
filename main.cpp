#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdio.h>
#include "Tools.h"
#include "Application.h"

using namespace std;
using namespace cv;
using namespace ARma;

int main() {

    Application app;


    app.initPattern();
   // app.initManualInput();
    app.initKinect();

    return 0;
}



