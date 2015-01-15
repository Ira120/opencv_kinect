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
    double adapt_thresh = 5; //non-used with FIXED_THRESHOLD mode
    int adapt_block_size = 45; //non-used with FIXED_THRESHOLD mode -> Size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on.
    double confidenceThreshold = 0.35;
    int mode = 2; //1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

    //initialize pattern detector in Patterndetector.cpp
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

    Mat rgbImage, depthMap, cloudMap;
    Mat rgbImageUndistorted, depthMapUndistorted, depthMapShow;
    Mat rgbImageROI, depthMapROI, cloudMapROIx, cloudMapROIy, cloudMapROIz, cloudMapROI (407,555,CV_32FC3);
    vector<Vec4i> lines_hough, lines_lsd;
    vector<Point3f> cam_lines3D;
    vector<Line3D> lines3DproFrame;


    for (;;) {
            capture.grab();
            capture.retrieve(rgbImage,CAP_OPENNI_BGR_IMAGE); //color image (CV_8UC3)
            capture.retrieve(depthMap,CAP_OPENNI_DEPTH_MAP); //depth values in mm (CV_16UC1)
            capture.retrieve(cloudMap,CAP_OPENNI_POINT_CLOUD_MAP); //XYZ in meters (CV_32FC3)

            /*
            // Print some avalible device settings.
            cout << "\nDepth generator output mode:" << endl <<
                    "FRAME_WIDTH      " << capture.get( CAP_PROP_FRAME_WIDTH ) << endl <<
                    "FRAME_HEIGHT     " << capture.get( CAP_PROP_FRAME_HEIGHT ) << endl <<
                    "FRAME_MAX_DEPTH  " << capture.get( CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
                    "FPS              " << capture.get( CAP_PROP_FPS ) << endl <<
                    "REGISTRATION     " << capture.get( CAP_PROP_OPENNI_REGISTRATION ) << endl;
            if( capture.get( CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
            {
                cout <<
                    "\nImage generator output mode:" << endl <<
                    "FRAME_WIDTH   " << capture.get( CAP_OPENNI_IMAGE_GENERATOR+CAP_PROP_FRAME_WIDTH ) << endl <<
                    "FRAME_HEIGHT  " << capture.get( CAP_OPENNI_IMAGE_GENERATOR+CAP_PROP_FRAME_HEIGHT ) << endl <<
                    "FPS           " << capture.get( CAP_OPENNI_IMAGE_GENERATOR+CAP_PROP_FPS ) << endl;
            }
            else
            {
                cout << "\nDevice doesn't contain image generator." << endl;
            }
*/
            //important for registration between rgb and depth data
            /*
             * Flag that registers the remapping depth map to image map by
             * changing depth generator’s view point (if the flag is "on") or
             * sets this view point to its normal one (if the flag is "off").
             * The registration process’s resulting images are pixel-aligned,
             * which means that every pixel in the image is aligned to a pixel
             * in the depth image.
             * http://docs.opencv.org/trunk/doc/user_guide/ug_highgui.html
             */


         //   cout<<"FOCAL      " << (float)capture.get( CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ) << endl;
            capture.set(CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1);
            if (rgbImage.empty() || depthMap.empty())
                break;

            /*
            //undistrot the rgbImage
            undistort(rgbImage,rgbImageUndistorted,patternLoader.getCameraMatrix(),patternLoader.getDistortions());
            //undistort depth map with rgb distortion coeffs, because of the registration between the two maps
            undistort(depthMap,depthMapUndistorted,patternLoader.getCameraMatrix(),patternLoader.getDistortions());

            //show RGBD data interleaved - undistorted
            Mat rgbdMapUndis(480,640,CV_8UC3);
            Mat depthMapUndistorted2 = depthMapUndistorted.clone();
            depthMapUndistorted2.convertTo(depthMapUndistorted2,CV_8U,0.2f);
            for (int v=0;v<rgbImage.rows;v++) {
                for (int u=0;u<rgbImage.cols;u++) {
                    rgbdMapUndis.at<Vec3b>(v,u)[0] = (rgbImageUndistorted.at<Vec3b>(v,u)[0]+depthMapUndistorted2.at<uchar>(v,u))/2;
                    rgbdMapUndis.at<Vec3b>(v,u)[1] = (rgbImageUndistorted.at<Vec3b>(v,u)[1]+depthMapUndistorted2.at<uchar>(v,u))/2;
                    rgbdMapUndis.at<Vec3b>(v,u)[2] = (rgbImageUndistorted.at<Vec3b>(v,u)[2]+depthMapUndistorted2.at<uchar>(v,u))/2;
                }
            }
            imshow("show overlayed input - undistorted", rgbdMapUndis);
            */

           // cloudshow.convertTo(cloudshow,CV_8UC1,0.25f);
           // imshow("POINT_CLOUD_Z",cloudshow);


            //show RGBD data interleaved - distorted
            Mat rgbdMatDis(480,640,CV_8UC3);
            depthMap.convertTo(depthMapShow, CV_8U,0.2f);
            for (int v=0;v<rgbImage.rows;v++) {
                for (int u=0;u<rgbImage.cols;u++) {
                    rgbdMatDis.at<Vec3b>(v,u)[0] = (rgbImage.at<Vec3b>(v,u)[0]+depthMapShow.at<uchar>(v,u))/2;
                    rgbdMatDis.at<Vec3b>(v,u)[1] = (rgbImage.at<Vec3b>(v,u)[1]+depthMapShow.at<uchar>(v,u))/2;
                    rgbdMatDis.at<Vec3b>(v,u)[2] = (rgbImage.at<Vec3b>(v,u)[2]+depthMapShow.at<uchar>(v,u))/2;
                }
            }
            imshow("show overlayed input - distorted", rgbdMatDis);


            //convert depth map from CV_16UC1 to CV_32F for better handling
            depthMap.convertTo(depthMap,CV_32F);


            //smooth depth map -> 6 = inner bound
            Mat depthMapSmoothed = smoothDepthMap(depthMap,6);

            //apply ROI for registered region in RGBD data
            Rect mask(30,63,555,407);
            rgbImageROI = rgbImage(mask);
            depthMapROI = depthMapSmoothed(mask);
          //  depthMapROI = depthMap(mask);
            cloudMapROI = cloudMap(mask);

            //just show the smoothed depth map
            depthMapSmoothed.convertTo(depthMapSmoothed,CV_8UC1,0.25f);
            imshow("smoothed depth map",depthMapSmoothed);

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input

            switch (key) {
                case 27: //escape key
                    return 0;

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'h': //run application with hough edge detection
                case 'H':
                    //test, if could detect marker
                    detectPattern(rgbImageROI);
                    if (detectedPats==false){
                        break;
                    }

                    log = SSTR("[DEBUG]: ****** CALCULATE 3DLINES FOR " << frame_nr << ". frame ******" << endl);
                    Log(log);

                    log = SSTR("[DEBUG]: ...edge detection with HOUGH..." << endl);
                    Log(log);


                    //fill vector with 2D points
                    lines_hough = edgeDetector.applyHoughTransformation(rgbImageROI,frame_nr);
                    //calculate 3D lines in camera CS
                    cam_lines3D = projection.calculateBackProjection(lines_hough,depthMapROI,pattern_origin,cloudMapROI);

                    /*
                    //TransVec (extr. Parameter) durch den von der Kinect ermittelten Marker-Usprungspunkt ersetzen
                    transVec = projection.pat_origin3D;
                    transVecMat.at<float>(0,0) = projection.pat_origin3D.x;
                    transVecMat.at<float>(1,0) = projection.pat_origin3D.y;
                    transVecMat.at<float>(2,0) = projection.pat_origin3D.z;
                    */

                  //  lines3DproFrame=calculateExtrinsicNew(cam_lines3D);

                    //tranfer the 3DLines into world CS (from camera CS)
                    lines3DproFrame=calculate3DLines(cam_lines3D);

                    log = SSTR("[DEBUG]: ...3Dlines number in world coordinates: " << lines3DproFrame.size() << "..."<< endl
                               << "========================================================" << endl);
                    Log(log);

                    //show projected 3DLines in frame
                    showLines3DInFrame(lines3DproFrame,rgbImageROI);

                    //create .obj-file with 3DLines per frame
                    edgeModel.createOBJproFrame(lines3DproFrame,frame_nr);

                    //store 3DLines per frame in a 'global' vector -> finished edge model
                    edgeModel.line3Dall.push_back(lines3DproFrame);


                    frame_nr++;

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'l': //run with LSD
                case 'L':

                    //test, if could detect marker
                    detectPattern(rgbImageROI);
                    if (detectedPats==false){
                        break;
                    }

                    log = SSTR("[DEBUG]: ****** CALCULATE 3DLINES FOR " << frame_nr << ". frame ******" << endl);
                    Log(log);

                    log = SSTR("[DEBUG]: ...edge detection with LSD..." << endl);
                    Log(log);


                    //fill vector with 2D points
                    lines_lsd = edgeDetector.applyLSD(rgbImageROI,frame_nr);
                    //calculate 3D lines in camera CS
                    cam_lines3D = projection.calculateBackProjection(lines_lsd,depthMapROI,pattern_origin,cloudMapROI);

                    /*
                    //TransVec (extr. Parameter) durch den von der Kinect ermittelten Marker-Usprungspunkt ersetzen
                    transVec = projection.pat_origin3D;
                    transVecMat.at<float>(0,0) = projection.pat_origin3D.x;
                    transVecMat.at<float>(1,0) = projection.pat_origin3D.y;
                    transVecMat.at<float>(2,0) = projection.pat_origin3D.z;
                    */

                    //tranfer the 3DLines into world CS (from camera CS)
                    lines3DproFrame=calculate3DLines(cam_lines3D);

                    log = SSTR("[DEBUG]: ...3Dlines number in world coordinates: " << lines3DproFrame.size() << "..."<< endl
                               << "========================================================" << endl);
                    Log(log);

                    //show projected 3DLines in frame
                    showLines3DInFrame(lines3DproFrame,rgbImageROI);

                    //create .obj-file with 3DLines per frame
                    edgeModel.createOBJproFrame(lines3DproFrame,frame_nr);

                    //store 3DLines per frame in a 'global' vector -> finished edge model
                    edgeModel.line3Dall.push_back(lines3DproFrame);


                    frame_nr++;

                    break;

                //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                case 'q': //press 'q' for the next step
                case 'Q':
                    capture.release();
                    cout<<"EXIT"<<endl;

                default:
                    break;
            }
    }

    char key = (char)waitKey(0); //delay N millis, usually long enough to display and capture input

    switch (key) {
        //group and approximate the lines
        case 'g':
        case 'G':
        edgeModel.createOBJfinal();
        return groupLines.findSimilarLines(edgeModel.line3Dall,frame_nr);

        case 27: //escape key
            return 0;

        default:
            break;
    }
    return 0;
}

//=======================================================================================//

void Application::detectPattern(Mat rgbImage) {
    rotVecMat.release();
    transVecMat.release();
    rotMat.release();

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
        detectedPats = false;

    } else {
        detectedPats = true;

        detectedPattern.at(0).showPattern();
      //  detectedPattern.at(0).draw(rgbImage,cameraMatrix,distortions); // draw a cube
      //  imshow("draw cube", rgbImage);

        detectedPattern.at(0).rotMat.copyTo(rotMat);
        transVec.val[0] = detectedPattern.at(0).transVec.at<double>(0,0);
        transVec.val[1] = detectedPattern.at(0).transVec.at<double>(0,1);
        transVec.val[2] = detectedPattern.at(0).transVec.at<double>(0,2);

        //****************
        //to draw 3D projected lines
        detectedPattern.at(0).rotVec.copyTo(rotVecMat);
        detectedPattern.at(0).transVec.copyTo(transVecMat);
        //*****************

        pattern_origin = detectedPattern.at(0).pattern_origin;

        log = SSTR("[DEBUG]: origin of world/marker coordinate system (uv-values): " <<  pattern_origin.at(3) << endl);
        Log(log);
    }
}

//=======================================================================================//

/*
vector<Line3D> Application::showCam3DLines(vector<Point3f> cam_lines3D) {
    Line3D line3D = Line3D::Line3D();
    vector<Line3D> lines3DproFrame;

    Point3f temp_start_point, temp_end_point;

    if (cam_lines3D.size()==0){
        log = SSTR ("[ERROR]: NO LINES WITH VALID Z DETECTED!\n");
        Log(log);

    } else {

        for (int i=0; i<(int)cam_lines3D.size(); i+=2){

                temp_start_point = cam_lines3D.at(i);

                //++++++++++++++++++++++++++++++++++++++++++++++++

                temp_end_point = cam_lines3D.at(i+1);

                //++++++++++++++++++++++++++++++++++++++++++++++++

                line3D.storeLine3D(temp_start_point,temp_end_point);

                lines3DproFrame.push_back(line3D);
            }
    }
    return lines3DproFrame;
}
*/
//=======================================================================================//

vector<Line3D> Application::calculate3DLines(vector<Point3f> cam_lines3D) {
    Line3D line3D = Line3D::Line3D();
    Mat inverseRotMat;
    Mat inverse_test;
    vector<Line3D> lines3DproFrame;

    //inverse matrix
    inverseRotMat = rotMat.inv();
    //for better handling
    inverseRotMat.convertTo(inverseRotMat,CV_32F);

    log = SSTR("[DEBUG]: Inverse Matrix: \n" << inverseRotMat << endl
               << "========================================================" << endl);
    Log(log);

    Point3f temp_start_point, temp_end_point;
    Vec3f result_start_point, result_end_point;
    Point3f result_start, result_end;

    if (cam_lines3D.size()==0){
        log = SSTR ("[ERROR]: NO LINES WITH VALID Z DETECTED!\n");
        Log(log);

    } else {

        for (int i=0; i<(int)cam_lines3D.size(); i+=2){

                temp_start_point = cam_lines3D.at(i);
                result_start_point = Vec<float,3>(temp_start_point);
                //back translation
                result_start_point = result_start_point-transVec;
                //back rotation
                result_start.x = inverseRotMat.at<float>(0,0)*result_start_point.val[0] +
                                                  inverseRotMat.at<float>(0,1)*result_start_point.val[1] +
                                                  inverseRotMat.at<float>(0,2)*result_start_point.val[2];
                result_start.y = inverseRotMat.at<float>(1,0)*result_start_point.val[0] +
                                                  inverseRotMat.at<float>(1,1)*result_start_point.val[1] +
                                                  inverseRotMat.at<float>(1,2)*result_start_point.val[2];
                result_start.z = inverseRotMat.at<float>(2,0)*result_start_point.val[0] +
                                                  inverseRotMat.at<float>(2,1)*result_start_point.val[1] +
                                                  inverseRotMat.at<float>(2,2)*result_start_point.val[2];

                //++++++++++++++++++++++++++++++++++++++++++++++++

                temp_end_point = cam_lines3D.at(i+1);
                result_end_point = Vec<float,3>(temp_end_point);
                //back translation
                result_end_point = result_end_point-transVec;
                //back rotation
                result_end.x = inverseRotMat.at<float>(0,0)*result_end_point.val[0] +
                                                  inverseRotMat.at<float>(0,1)*result_end_point.val[1] +
                                                  inverseRotMat.at<float>(0,2)*result_end_point.val[2];
                result_end.y = inverseRotMat.at<float>(1,0)*result_end_point.val[0] +
                                                  inverseRotMat.at<float>(1,1)*result_end_point.val[1] +
                                                  inverseRotMat.at<float>(1,2)*result_end_point.val[2];
                result_end.z = inverseRotMat.at<float>(2,0)*result_end_point.val[0] +
                                                  inverseRotMat.at<float>(2,1)*result_end_point.val[1] +
                                                  inverseRotMat.at<float>(2,2)*result_end_point.val[2];

                //++++++++++++++++++++++++++++++++++++++++++++++++

              //  line3D.storeLine3D(result_start_mat.at<float>(0,0),result_start_mat.at<float>(0,1),result_start_mat.at<float>(0,2),
               //                    result_end_mat.at<float>(0,0), result_end_mat.at<float>(0,1), result_end_mat.at<float>(0,2));

                line3D.storeLine3D(result_start,result_end);
                lines3DproFrame.push_back(line3D);
            }
    }
    return lines3DproFrame;
}

//=======================================================================================//

void Application::showLines3DInFrame(vector<Line3D> lines3DproFrame, Mat rgbImage) {
    //show 3Dlines in frame
    Mat modelPts;
    Mat disCoeff_empty;
    vector<Point2f> model2ImagePts;
    char filename[200];

    modelPts = Mat::zeros(lines3DproFrame.size()*2, 3, CV_32F);

    if (lines3DproFrame.size()!=0){
        log = SSTR("[DEBUG]: ...calculate the 3DLines and project them on the frame..." << endl);
        Log(log);

        for (int i=0; i<(int)lines3DproFrame.size();i++) {

            modelPts.at<float>(i*2,0) = lines3DproFrame.at(i).getStartPointOfLine3D().x;
            modelPts.at<float>(i*2,1) = lines3DproFrame.at(i).getStartPointOfLine3D().y;
            modelPts.at<float>(i*2,2) = lines3DproFrame.at(i).getStartPointOfLine3D().z;

            modelPts.at<float>(i*2+1,0) = lines3DproFrame.at(i).getEndPointOfLine3D().x;
            modelPts.at<float>(i*2+1,1) = lines3DproFrame.at(i).getEndPointOfLine3D().y;
            modelPts.at<float>(i*2+1,2) = lines3DproFrame.at(i).getEndPointOfLine3D().z;
        }

        projectPoints(modelPts, rotVecMat, transVecMat, patternLoader.getCameraMatrix(), patternLoader.getDistortions(), model2ImagePts);

        //draw the lines
        for (int i=0; i<(int)model2ImagePts.size(); i+=2){
            line(rgbImage, model2ImagePts.at(i), model2ImagePts.at(i+1), Scalar(255,0,0), 1, LINE_AA);
        }

        imshow("Projected 3DLines",rgbImage);
        sprintf(filename,"/Users/irina/Develop/workspace/bachelor_1/data/projected_lines%.2d.jpg",frame_nr);
        imwrite(filename, rgbImage);
    }
    else {
        log = SSTR("[DEBUG]: ...nothing to project..." << endl);
        Log(log);
    }
}

//=======================================================================================//

Mat Application::smoothDepthMap(Mat depthMapWithoutROI,int innerThreshold) {
    //nach http://www.codeproject.com/Articles/317974/KinectDepthSmoothing

    Mat smoothedMap=depthMapWithoutROI.clone();
    bool foundZero = true;
    while (foundZero) {

        foundZero = false;

        //gehe für alle Pixel in der ROI
        for (int v=63; v<=470; v++) {
            for (int u=30; u<=585; u++) {

                //wenn ein "fehlerhafter" Tiefenwert gefunden wurde
                if (depthMapWithoutROI.at<float>(v,u)==0) {
                    foundZero = true;

                    //wende den Filter für den inneren Ring an
                    vector<float> innerBound;
                    int countThreshold=0;

                    for (int j=-1; j<=1; j++) {
                        for (int i=-1; i<=1; i++) {
                            if (j==0 && i==0) {
                                //Mitte
                            } else {

                                if (depthMapWithoutROI.at<float>(v+j,u+i)==0) {
                                    countThreshold++;
                                } else {
                                    innerBound.push_back(depthMapWithoutROI.at<float>(v+j,u+i));
                                }
                            }
                        }
                    }

                    //wenn gezählte 0-Pixel in der näheren Nachbarschaft den vorgegebenen Schwellwert nicht überschreiten
                    if (countThreshold < innerThreshold) {

                        float result = Tools::modalValue(innerBound);
                        smoothedMap.at<float>(v,u) = result;
                    }

                }

            }
        }
    depthMapWithoutROI = smoothedMap.clone();

    }

    return smoothedMap;
}

//=======================================================================================//
/*
 vector<Line3D> Application::calculateExtrinsicNew(vector<Point3f> cam_lines3D) {
    Mat cameraMatrix = patternLoader.getCameraMatrix();
    Mat distortions = patternLoader.getDistortions();

    Mat rotVect = Mat::zeros( 1, 3, CV_64FC1 );
    Mat transVect = Mat::zeros( 1, 3, CV_64FC1 );
    Mat rotMatr = Mat::eye(3, 3, CV_64F);

    Point2f pat2DPts[4];
    Point3f pat3DPts[4];
    vector<Point3f> pattern_in3D = projection.pat_origin3D;
    vector<Point2f> pattern_in2D = pattern_origin;

    pat2DPts[0].x = pattern_in2D.at(0).x;
    pat2DPts[0].y = pattern_in2D.at(0).y;
    pat2DPts[1].x = pattern_in2D.at(1).x;
    pat2DPts[1].y = pattern_in2D.at(1).y;
    pat2DPts[2].x = pattern_in2D.at(2).x;
    pat2DPts[2].y = pattern_in2D.at(2).y;
    pat2DPts[3].x = pattern_in2D.at(3).x;
    pat2DPts[3].y = pattern_in2D.at(3).y;


    //for left-handed CS (x -> right; y -> up; z -> into the wall)
    pat3DPts[0].x = pattern_in3D.at(3).x;
    pat3DPts[0].y = pattern_in3D.at(3).y;
    pat3DPts[0].z = pattern_in3D.at(3).z;
    pat3DPts[1].x = pattern_in3D.at(2).x;
    pat3DPts[1].y = pattern_in3D.at(2).y;
    pat3DPts[1].z = pattern_in3D.at(2).z;
    pat3DPts[2].x = pattern_in3D.at(1).x;
    pat3DPts[2].y = pattern_in3D.at(1).y;
    pat3DPts[2].z = pattern_in3D.at(1).z;
    pat3DPts[3].x = pattern_in3D.at(0).x;
    pat3DPts[3].y = pattern_in3D.at(0).y;
    pat3DPts[3].z = pattern_in3D.at(0).z;


    Mat objectPts(4, 3, CV_32F, pat3DPts);
    Mat imagePts(4, 2, CV_32F, pat2DPts);
    //find extrinsic parameters
    solvePnPRansac(objectPts, imagePts, cameraMatrix, distortions, rotVect, transVect,false,100,1.2,0.99);
    Rodrigues(rotVect, rotMatr);

    cout<<"RotMatrix in CS: "<<rotMatr<<endl;
    cout<<"TransVec in CS: "<<transVect<<endl;

    //********************************************************

    Line3D line3D = Line3D::Line3D();
    vector<Line3D> lines3DproFrame;

    //for better handling
    rotMatr.convertTo(rotMatr,CV_32F);

    log = SSTR("[DEBUG]: Rotations Matrix in CS: \n" << rotMatr << endl
               << "========================================================" << endl);
    Log(log);

    Point3f temp_start_point, temp_end_point;
    Point3f result_start, result_end;

    if (cam_lines3D.size()==0){
        log = SSTR ("[ERROR]: NO LINES WITH VALID Z DETECTED!\n");
        Log(log);

    } else {

        for (int i=0; i<(int)cam_lines3D.size(); i+=2){

                temp_start_point = cam_lines3D.at(i);

                //back rotation
                result_start.x =    rotMatr.at<float>(0,0)*temp_start_point.x +
                                    rotMatr.at<float>(0,1)*temp_start_point.y +
                                    rotMatr.at<float>(0,2)*temp_start_point.z;
                result_start.y =    rotMatr.at<float>(1,0)*temp_start_point.x +
                                    rotMatr.at<float>(1,1)*temp_start_point.y +
                                    rotMatr.at<float>(1,2)*temp_start_point.z;
                result_start.z =    rotMatr.at<float>(2,0)*temp_start_point.x +
                                    rotMatr.at<float>(2,1)*temp_start_point.y +
                                    rotMatr.at<float>(2,2)*temp_start_point.z;

                //back translation
                result_start.x = result_start.x+transVect.at<float>(0,0);
                result_start.y = result_start.y+transVect.at<float>(1,0);
                result_start.z = result_start.z+transVect.at<float>(2,0);
                //++++++++++++++++++++++++++++++++++++++++++++++++

                temp_end_point = cam_lines3D.at(i+1);

                //back rotation
                result_end.x =    rotMatr.at<float>(0,0)*temp_end_point.x +
                                    rotMatr.at<float>(0,1)*temp_end_point.y +
                                    rotMatr.at<float>(0,2)*temp_end_point.z;
                result_end.y =    rotMatr.at<float>(1,0)*temp_end_point.x +
                                    rotMatr.at<float>(1,1)*temp_end_point.y +
                                    rotMatr.at<float>(1,2)*temp_end_point.z;
                result_end.z =    rotMatr.at<float>(2,0)*temp_end_point.x +
                                    rotMatr.at<float>(2,1)*temp_end_point.y +
                                    rotMatr.at<float>(2,2)*temp_end_point.z;

                //back translation
                result_end.x = result_end.x+transVect.at<float>(0,0);
                result_end.y = result_end.y+transVect.at<float>(1,0);
                result_end.z = result_end.z+transVect.at<float>(2,0);

                //++++++++++++++++++++++++++++++++++++++++++++++++

                line3D.storeLine3D(result_start,result_end);
                lines3DproFrame.push_back(line3D);
            }
    }

    return lines3DproFrame;

}
*/
//=======================================================================================//
// MANUAL INPUT with generated images
//=======================================================================================//

int Application::initManualInput() {
    int return_value = 0;
    char filename_rgb[200];
    char filename_depth[200];
    char filename_modelview[200];
    bool read_input = true;
    vector<Vec4i> lines_hough;
    vector<Point3f> cam_lines3D;
    vector<Line3D> lines3DproFrame;

    Mat rgbImageMan, depthMapMan(480,640,CV_32F), modelView(4,4,CV_32F);
    FileStorage fs;


    while (read_input){


    char key = (char)waitKey(3000); //delay N millis, usually long enough to display and capture input

    cout<<"test4"<<endl;

    imshow("depthtest",depthMapMan);
        switch (key) {
            //case 'm':

            case 'm':

            //load rgb images
            sprintf(filename_rgb,"/Users/irina/Develop/workspace/bachelor_1/input_data/ImgColor%.2d.png",frame_nr-1);
            rgbImageMan = imread(filename_rgb,1);

            sprintf(filename_depth,"/Users/irina/Develop/workspace/bachelor_1/input_data/ImgDepth32F%.2d.xml",frame_nr-1);

            fs.open(filename_depth, FileStorage::READ);
            fs["depth_matrix"] >> depthMapMan;

          //  depthMapMan.convertTo(depthMapMan,CV_8UC1,70.0f);
           // imshow("depthtest",depthMapMan);
           // cout<<depthMapMan<<endl;

            cout<<"frame: "<<frame_nr<<"geladen\n";
          // cout << depthMapMan << endl;
            //test, if could detect marker
            detectPatternManual(rgbImageMan);
            if (detectedPats==false){

                //modelView matrix manuell einlesen, falls kein Marker gefunden wurde
                sprintf(filename_modelview,"/Users/irina/Develop/workspace/bachelor_1/input_data/modelView%.2d.xml",frame_nr-1);

                fs.open(filename_modelview, FileStorage::READ);
                fs["modelView_matrix"] >> modelView;

                cout<<"frame: "<<frame_nr<<"geladen\n";

                //fill vector with 2D points
                lines_hough = edgeDetector.applyHoughTransformation(rgbImageMan,frame_nr);
                //calculate 3D lines in camera CS
                cam_lines3D = projection.calculateBackProjectionManual(lines_hough,depthMapMan);
                //tranfer the 3DLines into world CS (from camera CS)
                lines3DproFrame=calculate3DLinesManual(modelView,cam_lines3D);

            } else {

            //fill vector with 2D points
            lines_hough = edgeDetector.applyHoughTransformation(rgbImageMan,frame_nr);
            //calculate 3D lines in camera CS
            cam_lines3D = projection.calculateBackProjectionManual(lines_hough,depthMapMan);
            //tranfer the 3DLines into world CS (from camera CS)
            lines3DproFrame=calculate3DLines(cam_lines3D);

            log = SSTR("[DEBUG]: ...3Dlines number in world coordinates: " << lines3DproFrame.size() << "..."<< endl
                       << "========================================================" << endl);
            Log(log);

            }
            //create .obj-file with 3DLines per frame
            edgeModel.createOBJproFrame(lines3DproFrame,frame_nr);

            //store 3DLines per frame in a 'global' vector -> finished edge model
            edgeModel.line3Dall.push_back(lines3DproFrame);


            frame_nr++;
            read_input = true;
            break;


            case 27: //escape key
                read_input = false;
                edgeModel.createOBJfinal();
                return groupLines.findSimilarLines(edgeModel.line3Dall,frame_nr);
                break;

            default:
                read_input = true;
                break;
        }
    }

    return return_value;
}

//=======================================================================================//

void Application::detectPatternManual(Mat rgbImage) {
    rotVecMat.release();
    transVecMat.release();
    rotMat.release();

    detectedPats = false;
    vector<Pattern> detectedPattern;

    Mat cameraMatrix = (Mat_<float>(3,3) << 277.128f, 0, 320.0f, 0, 277.128f, 240.0f, 0, 0, 1);
    Mat distortions = (Mat_<float>(5,1) << 0, 0, 0, 0, 0);

    myDetector.detect(rgbImage,cameraMatrix,distortions,patternLibrary,detectedPattern);

    log = SSTR("[DEBUG]: ...detected patterns: " << detectedPattern.size() << "..." << endl);
    Log(log);

    if ((int)detectedPattern.size()==0) {

        log = SSTR("[ERROR]: NO MARKER FOUNDED!\n");
        Log(log);
        detectedPats = false;

    } else {
        detectedPats = true;

        //showPattern() important for RotationMatrix!
        detectedPattern.at(0).showPattern();
        detectedPattern.at(0).draw(rgbImage,cameraMatrix,distortions); // draw a cube
        imshow("draw cube", rgbImage);

        detectedPattern.at(0).rotMat.copyTo(rotMat);
        transVec.val[0] = detectedPattern.at(0).transVec.at<double>(0,0);
        transVec.val[1] = detectedPattern.at(0).transVec.at<double>(0,1);
        transVec.val[2] = detectedPattern.at(0).transVec.at<double>(0,2);

        //****************
        //to draw 3D projected lines
        detectedPattern.at(0).rotVec.copyTo(rotVecMat);
        detectedPattern.at(0).transVec.copyTo(transVecMat);
        //*****************

        pattern_origin = detectedPattern.at(0).pattern_origin;

        log = SSTR("[DEBUG]: origin of world/marker coordinate system (uv-values): " <<  pattern_origin << endl);
        Log(log);
    }
}

//=======================================================================================//

vector<Line3D> Application::calculate3DLinesManual(Mat modelView,vector<Point3f> cam_lines3D) {
    Line3D line3D = Line3D::Line3D();
    Mat inverseRotMat;
    vector<Line3D> lines3DproFrame;

    cout<<modelView<<endl;

    //inverse matrix
    inverseRotMat = modelView.inv();

    log = SSTR("[DEBUG]: Inverse Matrix: \n" << inverseRotMat << endl
               << "========================================================" << endl);
    Log(log);

    Point3f temp_start_point, temp_end_point;
    Vec4f result_start_point, result_end_point;
    Mat result_start_mat, result_end_mat;

    if (cam_lines3D.size()==0){
        log = SSTR ("[ERROR]: NO LINES WITH VALID Z DETECTED!\n");
        Log(log);

    } else {

        for (int i=0; i<(int)cam_lines3D.size(); i+=2){

                temp_start_point = cam_lines3D.at(i);
                //result_start_point = Vec<float,4>(temp_start_point);
                result_start_point.val[0] = temp_start_point.x;
                result_start_point.val[1] = temp_start_point.y;
                result_start_point.val[2] = temp_start_point.z;
                result_start_point.val[3] = 1.0f;

                Mat temp_mult = Mat(result_start_point);
                temp_mult.convertTo(temp_mult,CV_32FC1);
                inverseRotMat.convertTo(inverseRotMat,CV_32FC1);
                //back translation & rotation
                result_start_mat = inverseRotMat*temp_mult;

                //++++++++++++++++++++++++++++++++++++++++++++++++

                temp_end_point = cam_lines3D.at(i+1);
               // result_end_point = Vec<float,4>(temp_end_point);
                result_end_point.val[0] = temp_end_point.x;
                result_end_point.val[1] = temp_end_point.x;
                result_end_point.val[2] = temp_end_point.x;
                result_end_point.val[3] = temp_end_point.x;

                //back translation & rotation
                result_end_mat = inverseRotMat*Mat(result_end_point);

                //++++++++++++++++++++++++++++++++++++++++++++++++

                line3D.storeLine3D(result_start_mat.at<float>(0,0),result_start_mat.at<float>(0,1),result_start_mat.at<float>(0,2),
                                   result_end_mat.at<float>(0,0), result_end_mat.at<float>(0,1), result_end_mat.at<float>(0,2));

                lines3DproFrame.push_back(line3D);
            }
    }

    return lines3DproFrame;
}
