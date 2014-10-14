#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cameraparams.h"
#include "patterndetector.h"
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace ARma;

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 1// define the number of patterns you want to use

string filename1 = "/Users/irina/Develop/workspace/bachelor_AR/pattern1.png";//id=1
string filename2 = "/Users/irina/Develop/workspace/bachelor_AR/pattern2.png";//id=2
string filename3 = "/Users/irina/Develop/workspace/bachelor_AR/pattern3.png";//id=3


int main(){

    cameraparams camObj;
    Mat cameraMatrix = camObj.getCameraMatrix();
    Mat distortions = camObj.getDistortions();


	std::vector<cv::Mat> patternLibrary;
	std::vector<Pattern> detectedPattern;
	int patternCount=0;

    //create patterns' library using rotated versions of patterns

    camObj.loadPattern(filename1, patternLibrary, patternCount);
#if (NUM_OF_PATTERNS==2)
    camObj.loadPattern(filename2, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==3)
    camObj.loadPattern(filename2, patternLibrary, patternCount);
    camObj.loadPattern(filename3, patternLibrary, patternCount);
#endif


	cout << patternCount << " patterns are loaded." << endl;
	

	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
    int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode //Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
	double confidenceThreshold = 0.35;
    int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	PatternDetector myDetector( fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

    VideoCapture capture(CAP_OPENNI);
    if (!capture.isOpened()) {
        cerr << "Failed to open the video device, video file or image sequence!\n" << endl;
        return 1;
    }

	Mat imgMat;

    for (;;) {

        capture.grab();
        capture.retrieve(imgMat,CAP_OPENNI_BGR_IMAGE);

     //  char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

     // switch (key) {
     // case 27: //escape key
      //  return 0;
    // case ' ': //Save an image

		//run the detector
        myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern);

        cout << "Detected Patterns: " << detectedPattern.size() << endl;

		//augment the input frame (and print out the properties of pattern if you want)
		for (unsigned int i =0; i<detectedPattern.size(); i++){
            detectedPattern.at(i).showPattern();
            detectedPattern.at(i).draw( imgMat, cameraMatrix, distortions);
		}

        imshow("result", imgMat);
        waitKey(1);
		detectedPattern.clear();

//   default:
  //  break;
//}

}
                    return 0;
}


