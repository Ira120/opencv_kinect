#include "cameraparams.h"

//=======================================================================================//

cameraparams::cameraparams() {
    loadCameraParams();
}

//=======================================================================================//

cameraparams::~cameraparams() {}

//=======================================================================================//

Mat cameraparams::getCameraMatrix() {
    return cameraMatrix;
}

//=======================================================================================//

Mat cameraparams::getDistortions() {
    return distortions;
}

//=======================================================================================//

void cameraparams::loadCameraParams() {
    FileStorage fs ("/Users/irina/Develop/workspace/bachelor_AR/out_camera_data.yml", FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >>distortions;

    cout << "camera matrix: " << cameraMatrix << endl
         << "distortion coeffs: " << distortions << endl;
}

//=======================================================================================//

int cameraparams::loadPattern(const String& filename, vector<Mat>& library, int& patternCount){

    Mat img = imread(filename,IMREAD_GRAYSCALE); //graycsale image = 0
    //char filename1[200];

    if(img.cols!=img.rows){
        return -1;
        printf("Not a square pattern");
    }

    int msize = 64;

    Mat src(msize, msize, CV_8UC1);
    Point2f center((msize-1)/2.0f,(msize-1)/2.0f);
    Mat rot_mat(2,3,CV_32F);

    resize(img, src, Size(msize,msize));
    Mat subImg = src(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
    library.push_back(subImg);

    rot_mat = getRotationMatrix2D( center, 90, 1.0);

    for (int i=1; i<4; i++){
        Mat dst= Mat(msize, msize, CV_8UC1);
        rot_mat = getRotationMatrix2D( center, -i*90, 1.0);
        warpAffine( src, dst , rot_mat, Size(msize,msize));
        Mat subImg = dst(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
        library.push_back(subImg);

      //  sprintf(filename1,"test%.1d",i);
      //  imshow(filename1,subImg);
    }

    patternCount++;
    return 1;
}

//=======================================================================================//

vector<Mat> cameraparams::createPatternLib() {
    string filename1 = "/Users/irina/Develop/workspace/bachelor_AR/pattern1.png";//id=1
    string filename2 = "/Users/irina/Develop/workspace/bachelor_AR/pattern2.png";//id=2
    string filename3 = "/Users/irina/Develop/workspace/bachelor_AR/pattern3.png";//id=3

    vector<Mat> patternLibrary;
    int patternCount=0;

    //create patterns' library using rotated versions of patterns

    loadPattern(filename1, patternLibrary, patternCount);
#if (NUM_OF_PATTERNS==2)
    loadPattern(filename2, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==3)
    loadPattern(filename2, patternLibrary, patternCount);
    loadPattern(filename3, patternLibrary, patternCount);
#endif

     cout << patternCount << " patterns are loaded." << endl;

     return patternLibrary;
}
