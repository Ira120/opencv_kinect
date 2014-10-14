#include "Application.h"

//=======================================================================================//

Application::Application() {
    frame_nr = 1;
    edgeDetector = EdgeDetection::EdgeDetection();
    projection = BackProjection::BackProjection();
    patternLoader = cameraparams::cameraparams();
}

//=======================================================================================//

Application::~Application() {}

//=======================================================================================//

void Application::initPattern() {

   patternLibrary = patternLoader.createPatternLib();

   int norm_pattern_size = PAT_SIZE;
   double fixed_thresh = 40;
   double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
   int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode //Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
   double confidenceThreshold = 0.35;
   int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

   myDetector.initPatternDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);
}

//=======================================================================================//

int Application::initKinect() {
    int return_value;
    VideoCapture capture(CAP_OPENNI);

    if (!capture.isOpened()) {
        cerr << "Failed to open the video device, video file or image sequence!\n" << endl;
        return 1;
    } else {
        return_value = Application::frameLoop(capture);
        return return_value;
    }
}

//=======================================================================================//

int Application::frameLoop(VideoCapture capture){
    Mat rgbImage, depthMap;
    vector<Vec4i> lines_hough;
    vector<Vec4i> lines_lsd;
    int return_value;

    for (;;) {
            capture.grab();
            capture.retrieve(rgbImage,CAP_OPENNI_BGR_IMAGE);
            capture.retrieve(depthMap,CAP_OPENNI_DEPTH_MAP);
            capture.set(CAP_PROP_OPENNI_REGISTRATION , 0);
            if (rgbImage.empty() || depthMap.empty())
                break;

            imshow("show RGB input", rgbImage);

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

            switch (key) {
                case 27: //escape key
                    return 0;

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'h': //run with hough
                case 'H': //run the application 'step by step'
                    cout << "...Detection with Hough..." << endl;
                    detectPattern(rgbImage);
                    lines_hough = edgeDetector.applyHoughTransformation(rgbImage,frame_nr);
                    projection.calculateBackProjection(lines_hough,depthMap);
                    calculate3DLines();

                    cout<<"lines number: "<<edgeModel.lines3DproFrame.size()<<endl;
                    edgeModel.createOBJ(frame_nr);
                    frame_nr++;

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'l': //run with LSD
                case 'L':
                    cout << "...Detection with LSD..." << endl;
                    detectPattern(rgbImage);
                    lines_lsd = edgeDetector.applyLSD(rgbImage,frame_nr);
                    projection.calculateBackProjection(lines_lsd,depthMap);
                    calculate3DLines();

                    cout<<"lines number: "<<edgeModel.lines3DproFrame.size()<<endl;
                    edgeModel.createOBJ(frame_nr);
                    frame_nr++;

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                default:
                    break;
            }
    }
    return 0;
}

//=======================================================================================//

void Application::calculate3DLines() {
    Line3D line3D = Line3D::Line3D();
    edgeModel = EdgeModel::EdgeModel();
    Mat inverseRotMat;

    //inverse matrix
    inverseRotMat = rotMat.inv();
    inverseRotMat.convertTo(inverseRotMat,CV_32F);
    cout<<"Inverse Matrix: "<<inverseRotMat<<endl;


    Point3f temp_start_point, temp_end_point;
    Vec3f result_start_point, result_end_point;
    Mat result_start_mat, result_end_mat;

    //TODO: wenn vector mit Linien != leer - Abfrage!
    for (int i=0; i<(int)projection.camera_lines3D.size(); i+=2){
        temp_start_point = projection.camera_lines3D.at(i);
        result_start_point = Vec<float,3>(temp_start_point);
        //back translation
        result_start_point = result_start_point-transVec;
        //back rotation
        result_start_mat = inverseRotMat*Mat(result_start_point);

        //++++++++++++++++++++++++++++++++++++++++++++++++

        temp_end_point = projection.camera_lines3D.at(i+1);
        result_end_point = Vec<float,3>(temp_end_point);
        //back translation
        result_end_point = result_end_point-transVec;
        //back rotation
        result_end_mat = inverseRotMat*Mat(result_end_point);

        //++++++++++++++++++++++++++++++++++++++++++++++++

        line3D.storeLine3D(result_start_mat.at<float>(0,0),result_start_mat.at<float>(0,1),result_start_mat.at<float>(0,2),
                           result_end_mat.at<float>(0,0), result_end_mat.at<float>(0,1), result_end_mat.at<float>(0,2));
        edgeModel.lines3DproFrame.push_back(line3D);
        }
}

//=======================================================================================//

void Application::detectPattern(Mat rgbImage) {

    vector<Pattern> detectedPattern;

    Mat cameraMatrix = patternLoader.getCameraMatrix();
    Mat distortions = patternLoader.getDistortions();

    myDetector.detect(rgbImage,cameraMatrix,distortions,patternLibrary,detectedPattern);

    cout << "Detected Patterns: " << detectedPattern.size() << endl;

    if ((int)detectedPattern.size()==0){
        break;
    }

    //TODO: for more then one patter!!!

    //showPattern() important for RotationMatrix!
    detectedPattern.at(0).showPattern();

    detectedPattern.at(0).rotMat.copyTo(rotMat);
    transVec.val[0] = detectedPattern.at(0).transVec.at<double>(0,0);
    transVec.val[1] = detectedPattern.at(0).transVec.at<double>(0,1);
    transVec.val[2] = detectedPattern.at(0).transVec.at<double>(0,2);

    cout << "--------------------------------------" << endl;
    cout << "TransVec now: " << transVec <<endl;
    cout << "RotMat now: " << rotMat <<endl;
    cout << "--------------------------------------" << endl;

    return 1;
}
