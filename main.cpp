/*
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

int main(){
    cout << "opening device(s)" << endl;

    //cout << cv::getBuildInformation() << endl;

    VideoCapture sensor1;



    sensor1.open(CAP_OPENNI);

    sensor1.set(CAP_PROP_OPENNI_REGISTRATION_ON,0);
    if( !sensor1.isOpened() ){
        cout << "Can not open capture object 1." << endl;
        return -1;
    }

    for(;;){
        Mat depth1;

        if( !sensor1.grab() ){
            cout << "Sensor1 can not grab images." << endl;
            return -1;
        }else if( sensor1.retrieve( depth1, CAP_OPENNI_POINT_CLOUD_MAP ) ) imshow("depth1",depth1);

        if( waitKey( 30 ) == 27 )   break;//ESC to exit

   }
}
*/

/*
 * starter_video.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: Ethan Rublee
 *
 *  Modified on: April 17, 2013
 *      Author: Kevin Hughes
 *
 * A starter sample for using OpenCV VideoCapture with capture devices, video files or image sequences
 * easy as CV_PI right?
 */

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include "Tools.h"

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main() {

    VideoCapture capture(CAP_OPENNI);

    if (!capture.isOpened()) {
        cerr << "Failed to open the video device, video file or image sequence!\n" << endl;
        return 1;
    }

    else {

        Tools::saveFramesFromKinect(capture);

        }
}
