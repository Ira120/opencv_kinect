#include "Application.h"

//=======================================================================================//

Application::Application() {
    frame_nr = 1;
    detectedPats = false;
    edgeDetector = EdgeDetection::EdgeDetection();
    projection = BackProjection::BackProjection();
    patternLoader = MarkerTracker::MarkerTracker();
    edgeModel = EdgeModel::EdgeModel();
    groupLines = GroupLines3D::GroupLines3D();
}

//=======================================================================================//

Application::~Application() {}

//=======================================================================================//

void Application::initPattern() {

    log = SSTR("[DEBUG]: ...INIT PATTERN...\n");
    Log(log);

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

    log = SSTR("[DEBUG]: ...INIT KINECT...\n");
    Log(log);

    capture.open(CAP_OPENNI);

    if (!capture.isOpened()) {

        log = SSTR("[ERROR]: Failed to open the video device, video file or image sequence!\n");
        Log(log);

        return 1;
    } else {
        return_value = Application::frameLoop();
        return return_value;
    }
}

//=======================================================================================//

int Application::frameLoop(){

    log = SSTR("[DEBUG]: ...START APPLICATION...\n");
    Log(log);

    Mat rgbImage, depthMap;
    vector<Vec4i> lines_hough;
    vector<Vec4i> lines_lsd;

    for (;;) {
            capture.grab();
            capture.retrieve(rgbImage,CAP_OPENNI_BGR_IMAGE);
            capture.retrieve(depthMap,CAP_OPENNI_DEPTH_MAP);
            capture.set(CAP_PROP_OPENNI_REGISTRATION , 0);
            if (rgbImage.empty() || depthMap.empty())
                break;

            log = SSTR("[DEBUG]: decode and return the grabbed video frame from KINECT"<<endl);
           // Log(log);

            imshow("show RGB input", rgbImage);

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

            switch (key) {
                case 27: //escape key
                    return 0;

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'h': //run with hough
                case 'H': //run the application 'step by step'
                    log = SSTR("[DEBUG]: ****** calculate 3Dlines for " << frame_nr << ". frame ******" << endl);
                    Log(log);

                    log = SSTR("[DEBUG]: ...edge detection with HOUGH..." << endl);
                    Log(log);

                    detectPattern(rgbImage);
                    if (detectedPats==false){
                        break;
                    }

                    lines_hough = edgeDetector.applyHoughTransformation(rgbImage,frame_nr);
                    projection.calculateBackProjection(lines_hough,depthMap);
                    calculate3DLines();

                    log = SSTR("[DEBUG]: ...3Dlines number in world coordinates: " << edgeModel.lines3DproFrame.size() << "..."<< endl
                               << "========================================================" << endl);
                    Log(log);

                   // edgeModel.createOBJ(frame_nr);
                    edgeModel.line3Dall.push_back(edgeModel.lines3DproFrame);
                    frame_nr++;
                    edgeModel.lines3DproFrame.clear();

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'l': //run with LSD
                case 'L':
                    log = SSTR("[DEBUG]: ****** calculate 3Dlines for " << frame_nr << ". frame ******" << endl);
                    Log(log);

                    log = SSTR("[DEBUG]: ...edge detection with LSD..." << endl);
                    Log(log);

                    detectPattern(rgbImage);
                    if (detectedPats==false){
                    break;
                    }

                    lines_lsd = edgeDetector.applyLSD(rgbImage,frame_nr);
                    projection.calculateBackProjection(lines_lsd,depthMap);
                    calculate3DLines();

                    log = SSTR("[DEBUG]: ...3Dlines number in world coordinates: " << edgeModel.lines3DproFrame.size() << "..."<< endl
                               << "========================================================" << endl);
                    Log(log);

                    //edgeModel.createOBJ(frame_nr);
                    edgeModel.line3Dall.push_back(edgeModel.lines3DproFrame);
                    frame_nr++;
                    edgeModel.lines3DproFrame.clear();

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'q':
                capture.release();
                cout <<"test"<<endl;
                default:
                    break;
            }
    }

    char key = (char)waitKey(0); //delay N millis, usually long enough to display and capture input

    switch (key) {
        case 'o':
        case 'O':
        //cout << edgeModel.line3Dall.size() << endl;
        edgeModel.createOBJfinal();
        return groupLines.findSimilarLines(edgeModel.line3Dall);


        default:
            break;
        case 27: //escape key
            return 0;
    }
    return 0;
}

//=======================================================================================//

void Application::calculate3DLines() {
    Line3D line3D = Line3D::Line3D();
    Mat inverseRotMat;

    //inverse matrix
    inverseRotMat = rotMat.inv();
    inverseRotMat.convertTo(inverseRotMat,CV_32F);

    log = SSTR("[DEBUG]: Inverse Matrix: \n" << inverseRotMat << endl
               << "========================================================" << endl);
    Log(log);

    Point3f temp_start_point, temp_end_point;
    Vec3f result_start_point, result_end_point;
    Mat result_start_mat, result_end_mat;

    if (projection.camera_lines3D.size()==0){
        log = SSTR ("[ERROR]: NO LINES WITH VALID Z DETECTED!\n");
        Log(log);

    } else {

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
    projection.camera_lines3D.clear();
}

//=======================================================================================//

void Application::detectPattern(Mat rgbImage) {
    detectedPats = false;
    vector<Pattern> detectedPattern;

    Mat cameraMatrix = patternLoader.getCameraMatrix();
    Mat distortions = patternLoader.getDistortions();

    myDetector.detect(rgbImage,cameraMatrix,distortions,patternLibrary,detectedPattern);

    log = SSTR("[DEBUG]: ...detected patterns: " << detectedPattern.size() << "..." << endl);
    Log(log);

    if ((int)detectedPattern.size()==0) {

        log = SSTR("[ERROR]: NO MARKER FOUNDED!\n");
        Log(log);

        //capture.release();
        //exit(0);
        detectedPats = false;

    } else {
        detectedPats = true;

        //TODO: for more then one patter!!!

        //showPattern() important for RotationMatrix!
        detectedPattern.at(0).showPattern();
       // detectedPattern.at(0).draw(rgbImage,cameraMatrix,distortions);

        detectedPattern.at(0).rotMat.copyTo(rotMat);
        transVec.val[0] = detectedPattern.at(0).transVec.at<double>(0,0);
        transVec.val[1] = detectedPattern.at(0).transVec.at<double>(0,1);
        transVec.val[2] = detectedPattern.at(0).transVec.at<double>(0,2);


        cout << "--------------------------------------" << endl;
        cout << "TransVec now: " << transVec <<endl;
        cout << "RotMat now: " << rotMat <<endl;
        cout << "--------------------------------------" << endl;

    }
}
