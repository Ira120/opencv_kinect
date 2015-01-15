//http://xanthippi.ceid.upatras.gr/people/evangelidis/arma/

#include "MarkerTracker.h"

//=======================================================================================//

MarkerTracker::MarkerTracker() {
    loadCameraParams();
}

//=======================================================================================//

MarkerTracker::~MarkerTracker() {}

//=======================================================================================//

Mat MarkerTracker::getCameraMatrix() {

    return cameraMatrix;
}

//=======================================================================================//

Mat MarkerTracker::getDistortions() {

    return distortions;
}

//=======================================================================================//

void MarkerTracker::loadCameraParams() {
    FileStorage fs ("/Users/irina/Develop/workspace/bachelor_1/out_camera_data.yml", FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >>distortions;
    fs["depth_intrinsics"] >> cameraMatrix_depth;
    fs["depth_distortion"] >>distortions_depth;
}

//=======================================================================================//

int MarkerTracker::loadPattern(const String& filename, vector<Mat>& library, int& patternCount) {

    //load the pattern
    Mat img = imread(filename,IMREAD_GRAYSCALE); //graycsale image = 0

    if(img.cols!=img.rows){

        log = SSTR("[ERROR]: Not a square pattern");
        Log(log);

        return -1;
    }

    Mat src(PAT_SIZE, PAT_SIZE, CV_8UC1);
    Point2f center((PAT_SIZE-1)/2.0f,(PAT_SIZE-1)/2.0f);
    Mat rot_mat(2,3,CV_32F);

    //resize pattern to PAT_SIZExPAT_SIZE
    resize(img, src, Size(PAT_SIZE,PAT_SIZE));
    //'cut' the white middle of the pattern and put in the library-vector
    Mat subImg = src(Range(PAT_SIZE/4,3*PAT_SIZE/4), Range(PAT_SIZE/4,3*PAT_SIZE/4));
    library.push_back(subImg);

    //calculates an affine matrix of 2D rotation
   // rot_mat = getRotationMatrix2D(center, 90, 1.0);

    for (int i=1; i<4; i++){
        Mat dst = Mat(PAT_SIZE, PAT_SIZE, CV_8UC1);
        //calculates an affine matrix of 2D rotation
        rot_mat = getRotationMatrix2D( center, -i*90, 1.0);
        //applies an affine transformation to an image
        warpAffine(src, dst,rot_mat, Size(PAT_SIZE,PAT_SIZE));
        //'cut' the white middle of the pattern and put in the library-vector
        Mat subImg = dst(Range(PAT_SIZE/4,3*PAT_SIZE/4), Range(PAT_SIZE/4,3*PAT_SIZE/4));
        library.push_back(subImg);
    }

  //  imshow("Mat1",library.at(0));
  //  imshow("Mat2",library.at(1));
  //  imshow("Mat3",library.at(2));
  //  imshow("Mat4",library.at(3));

    patternCount++;
    return 1;
}

//=======================================================================================//

vector<Mat> MarkerTracker::createPatternLib() {
    string filename1 = "/Users/irina/Develop/workspace/bachelor_1/pattern4.png";//id=1
   // string filename2 = "/Users/irina/Develop/workspace/bachelor_AR/pattern2.png";//id=2
   // string filename3 = "/Users/irina/Develop/workspace/bachelor_AR/pattern3.png";//id=3

    vector<Mat> patternLibrary;
    int patternCount=0;

    //create patterns' library using rotated versions of patterns
    loadPattern(filename1, patternLibrary, patternCount);

    if (patternCount == 1) {
        log = SSTR("[DEBUG]: ..." << patternCount << " pattern is loaded to marker detector...\n");
        Log(log);
    } else {
        log = SSTR("[DEBUG]: ..." << patternCount << " patterns are loaded to marker detector...\n");
        Log(log);
    }


    return patternLibrary;
}
