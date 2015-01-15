#include "BackProjection.h"

//=======================================================================================//

BackProjection::BackProjection() {
    //parameters from http://nicolas.burrus.name/index.php/Research/KinectCalibration
    Mat cameraMatrix;
    Mat distortions;
    Mat cameraMatrix_depth;
    Mat distortions_depth;

    FileStorage fs ("/Users/irina/Develop/workspace/bachelor_1/out_camera_data.yml", FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >>distortions;
    fs["depth_intrinsics"] >> cameraMatrix_depth;
    fs["depth_distortion"] >>distortions_depth;

    //for RGBImage
    focal_point_x_rgb = cameraMatrix.at<double>(0,0);
    focal_point_y_rgb = cameraMatrix.at<double>(1,1);
    center_of_projection_x_rgb = cameraMatrix.at<double>(0,2);
    center_of_projection_y_rgb = cameraMatrix.at<double>(1,2);

    //for depthMat
    focal_point_x_depth = cameraMatrix_depth.at<double>(0,0);
    focal_point_y_depth = cameraMatrix_depth.at<double>(1,1);
    center_of_projection_x_depth = cameraMatrix_depth.at<double>(0,2);
    center_of_projection_y_depth = cameraMatrix_depth.at<double>(1,2);

    //Relative transform between the sensors (in meters)
    rotMatDepthToRGB = (Mat_<float>(3,3) << 9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02,
                                            -1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02,
                                            1.7470421412464927e-02, 1.2275341476520762e-02, 9.9977202419716948e-01);
    transVecDepthToRGB = (Mat_<float>(3,1) << 1.9985242312092553e-02, -7.4423738761617583e-04, -1.0916736334336222e-02);
}

//=======================================================================================//

BackProjection::~BackProjection() {}

//=======================================================================================//

float BackProjection::findZInDepthMap(int x, int y) {


    float depth_z = this->depthImage.at<float>(Point(x,y));

    float depth_z1 = this->depthImage.at<float>(Point(x-2,y-2));
    float depth_z2 = this->depthImage.at<float>(Point(x-1,y-2));
    float depth_z3 = this->depthImage.at<float>(Point(x,y-2));
    float depth_z4 = this->depthImage.at<float>(Point(x+1,y-2));
    float depth_z5 = this->depthImage.at<float>(Point(x+2,y-2));

    float depth_z6 = this->depthImage.at<float>(Point(x-2,y-1));
    float depth_z7 = this->depthImage.at<float>(Point(x-2,y-1));
    float depth_z8 = this->depthImage.at<float>(Point(x,y-1));
    float depth_z9 = this->depthImage.at<float>(Point(x+1,y-1));
    float depth_z10 = this->depthImage.at<float>(Point(x+2,y-1));

    float depth_z11 = this->depthImage.at<float>(Point(x-2,y));
    float depth_z12 = this->depthImage.at<float>(Point(x-1,y));
    float depth_z14 = this->depthImage.at<float>(Point(x+1,y));
    float depth_z15 = this->depthImage.at<float>(Point(x+2,y));

    float depth_z16 = this->depthImage.at<float>(Point(x-2,y+1));
    float depth_z17 = this->depthImage.at<float>(Point(x-1,y+1));
    float depth_z18 = this->depthImage.at<float>(Point(x,y+1));
    float depth_z19 = this->depthImage.at<float>(Point(x+1,y+1));
    float depth_z20 = this->depthImage.at<float>(Point(x+2,y+1));

    float depth_z21 = this->depthImage.at<float>(Point(x-2,y+2));
    float depth_z22 = this->depthImage.at<float>(Point(x-1,y+2));
    float depth_z23 = this->depthImage.at<float>(Point(x,y+2));
    float depth_z24 = this->depthImage.at<float>(Point(x+1,y+2));
    float depth_z25 = this->depthImage.at<float>(Point(x+2,y+2));

//    1   2   3   4   5
//    6   7   8   9   10
//    11  12  13  14  15
//    16  17  18  19  20
//    21  22  23  24  25

    log = SSTR("[DEBUG]: u: " << x << " and v: " << y << " result in z-value: " << depth_z << endl);
    Log(log);

    log = SSTR("[DEBUG]: show neighborhood in depthMap: "<<endl
    <<depth_z1<<", "<<depth_z2<<", "<<depth_z3<<", "<<depth_z4<<", "<<depth_z5<< endl
    <<depth_z6<<", "<<depth_z7<<", "<<depth_z8<<", "<<depth_z9<<", "<<depth_z10<< endl
    <<depth_z11<<", "<<depth_z12<<", "<<depth_z<<", "<<depth_z14<<", "<<depth_z15<< endl
    <<depth_z16<<", "<<depth_z17<<", "<<depth_z18<<", "<<depth_z19<<", "<<depth_z20<< endl
    <<depth_z21<<", "<<depth_z22<<", "<<depth_z23<<", "<<depth_z24<<", "<<depth_z25<< endl);
    Log(log);

    //accumulate in 3x3 neighborhood non-zero values
    vector<float> temp_pixel;
    for (int i=-2;i<=2;i++) {
        for (int j=-2; j<=2;j++) {
            if (this->depthImage.at<float>(Point(x+i,y+j))!=0){
                temp_pixel.push_back(this->depthImage.at<float>(Point(x+i,y+j)));
            }
        }
    }

    if(!temp_pixel.empty()){

        //object should be in this depth band
        float min_depth = averangeDepth - 200.0f;
        float max_depth = averangeDepth + 200.0f;

        //1st case: current depth value is between min_depth and max_depth
        if (this->depthImage.at<float>(Point(x,y)) >= min_depth && this->depthImage.at<float>(Point(x,y)) <= max_depth) {
            log = SSTR("[DEBUG]: ...current depth value is vaild (between "<<min_depth<<" and "<<max_depth<<")"<<endl);
            Log(log);


            /*
            //take the mean of the neighbours
            float temp_result = 0.0f;
            int temp_num = 0;
            for (int i=0; i<temp_pixel.size(); i++) {
                //calculate averange of valid values
                if(temp_pixel.at(i) >= min_depth && temp_pixel.at(i) <= max_depth) {
                    temp_result += temp_pixel.at(i);
                    temp_num ++;
                }
            }
            */


            //take the nearest value to the cam
            float result = *min_element(temp_pixel.begin(),temp_pixel.end());

            log = SSTR("[DEBUG]: calculated depth is: "<<result<<endl);
            Log(log);

            return result;
        }

        //2nd case: current depth value is over the max_depth
        else if (this->depthImage.at<float>(Point(x,y)) > max_depth) {
            log = SSTR("[DEBUG]: ...current depth value is invaild (larger than "<<max_depth<<")"<<endl);
            Log(log);

            vector<float> temp_values;
            for (int i=0; i<(int)temp_pixel.size();i++) {
                //search for valid values in neighborhood
                if (temp_pixel.at(i) <= max_depth && temp_pixel.at(i) >= min_depth) {
                    temp_values.push_back(temp_pixel.at(i));
                }
            }

            //calculate mode
            if(!temp_values.empty()) {
                float temp_result = *min_element(temp_values.begin(),temp_values.end());
                log = SSTR("[DEBUG]: calculated depth is: "<<temp_result<<endl);
                Log(log);

                return temp_result;
            } else {
                log = SSTR("[DEBUG]: reject this z-value"<<endl);
                Log(log);
                return 0.0f;
            }
        }
        //3nth case: current depth value is less than min_depth
        else {
            log = SSTR("[DEBUG]: ...current depth value is invaild (less than "<<min_depth<<")"<<endl);
            Log(log);

            vector<float> temp_values;
            for (int i=0; i<(int)temp_pixel.size();i++) {
                //search for valid values in neighborhood
                if (temp_pixel.at(i) <= max_depth && temp_pixel.at(i) >= min_depth) {
                    temp_values.push_back(temp_pixel.at(i));
                }
            }

            //calculate mode
            if(!temp_values.empty()) {
                float temp_result = *max_element(temp_values.begin(),temp_values.end());
                log = SSTR("[DEBUG]: calculated depth is: "<<temp_result<<endl);
                Log(log);

                return temp_result;
            } else {
                log = SSTR("[DEBUG]: reject this z-value"<<endl);
                Log(log);
                return 0.0f;
            }
        }
    }
    return 0.0f;

    return depth_z;
}

//=======================================================================================//

Point3f BackProjection::calculateCameraXYZ(int x, int y, float depth_z, Mat cloudMap) {
    //calculate world coordinates from screen coordinates
    //http://stackoverflow.com/questions/12007775/to-calculate-world-coordinates-from-screen-coordinates-with-opencv

    float x_camera = ((float)x - center_of_projection_x_rgb) * depth_z / focal_point_x_rgb;
    float y_camera = ((float)y - center_of_projection_y_rgb) * depth_z / focal_point_y_rgb;

    Point3f xyz_camera = Point3f(x_camera, y_camera, depth_z);

    return xyz_camera;
}

//=======================================================================================//

vector<Point3f> BackProjection::calculateBackProjection(vector<Vec4i> lines2D, Mat depthImage, vector<Point2f> pattern_points, Mat cloudMap) {
    this->depthImage.release();
    depthImage.copyTo(this->depthImage);

    float depth_z_start;
    float depth_z_end;
    Vec4i line2D;
    Point3f start_xyz;
    Point3f end_xyz;
    vector<Point3f> cam_lines3D;

    calculatePatternOriginInCam(pattern_points);

    calculateAverangeDepth();

    log = SSTR("========================================================\n");
    Log(log);

    for (int i=0; i<(int)lines2D.size();i++){
        line2D = lines2D.at(i);

        depth_z_start = findZInDepthMap(line2D.val[0],line2D.val[1]);
        depth_z_end = findZInDepthMap(line2D.val[2],line2D.val[3]);

        //z=0 check
        if (((int)depth_z_start <= 0 ) | ((int)depth_z_end <= 0)){

            log = SSTR("[DEBUG]: invalid z value: at line nr. " << i+1 << endl);
            Log(log);
            log = SSTR("[DEBUG]: ........................................" << endl);
            Log(log);

        }else{

            log = SSTR("[DEBUG]: start z value in camera coord: "<<depth_z_start<<" ----> end z value in camera coord: "<<depth_z_end<<endl);
            Log(log);

            start_xyz = calculateCameraXYZ(line2D.val[0],line2D.val[1],depth_z_start, cloudMap);
            end_xyz = calculateCameraXYZ(line2D.val[2], line2D.val[3], depth_z_end, cloudMap);
            cam_lines3D.push_back(start_xyz);
            cam_lines3D.push_back(end_xyz);

            log = SSTR("[DEBUG]: ...start_xyz in camera coord: "<<start_xyz<<" ----> end_xyz in camera coord: "<<end_xyz<<endl);
            Log(log);
            log = SSTR("[DEBUG]: ........................................" << endl);
            Log(log);
        }

    }

    log = SSTR("========================================================\n");
    Log(log);

    return cam_lines3D;
}


//=======================================================================================//

void BackProjection::calculatePatternOriginInCam (vector<Point2f> pattern_origin) {

    log = SSTR("========================================================\n");
    Log(log);

    for(int i=0; i<(int)pattern_origin.size();i++) {
        Point3f temp_point;
        float depth_z_pat = this->depthImage.at<float>(pattern_origin.at(i));

        temp_point.x = ((float)pattern_origin.at(i).x - center_of_projection_x_rgb) * depth_z_pat / focal_point_x_rgb;
        temp_point.y = ((float)pattern_origin.at(i).y - center_of_projection_y_rgb) * depth_z_pat / focal_point_y_rgb;

        temp_point.z = depth_z_pat;

        pat_origin3D.push_back(temp_point);

        log = SSTR("[DEBUG]: marker corners in rgb cam CS: ("
                   <<pat_origin3D.at(i).x<<"|"<<pat_origin3D.at(i).y<<"|"<<depth_z_pat<<")"<<endl);
        Log(log);
    }

    log = SSTR("[DEBUG]: marker origin in cam CS: ("
                    <<pat_origin3D.at(3).x<<"|"<<pat_origin3D.at(3).y<<"|"<<this->depthImage.at<float>(pattern_origin.at(3))<<")"<<endl);
    Log(log);

    log = SSTR("========================================================\n");
    Log(log);
}

//=======================================================================================//

void BackProjection::calculateAverangeDepth() {
    float temp_averange = 0.0f;
    int num = 0;

    for (int i=240; i<=340; i++) {
        for (int j=250; j<=350; j++) {
            temp_averange += this->depthImage.at<float>(j,i);
            num++;
        }
    }

    averangeDepth = temp_averange/(float)num;

    log = SSTR("[DEBUG]: averange depth in frame: "<<averangeDepth<<" mm"<<endl);
    Log(log);
}

//=======================================================================================//
//nach http://nicolas.burrus.name/index.php/Research/KinectCalibration
/*
Mat BackProjection::calculateTranslatedRGBMap(Mat depthMap) {
    Mat rgbMap3Dcorresponded = Mat::zeros(600,750,CV_32FC3);
    Mat depthMap3Dcorresponded = Mat::zeros(600,750,CV_32FC3);

    for (int v=0; v<depthMap.rows; v++) {
        for (int u=0; u<depthMap.cols; u++) {
            Mat P3D_depth (3,1,CV_32F);
            Mat P3D_rgb (3,1,CV_32F);
            Mat P3D_temp (3,1,CV_32F);
            float temp_u;
            float temp_v;
           // cout << "DeptMap u: " << u << " und v: " << v << endl;
           // depthMap3D.at<Vec3f>(v,u)[0] = (u - center_of_projection_x_depth) * depthMap.at<float>(v,u) / focal_point_x_depth;
           // depthMap3D.at<Vec3f>(v,u)[1] = (v - center_of_projection_y_depth) * depthMap.at<float>(v,u) / focal_point_y_depth;
           // depthMap3D.at<Vec3f>(v,u)[2] = depthMap.at<float>(v,u);
            P3D_depth.at<float>(0,0) = (u - center_of_projection_x_depth) * depthMap.at<float>(v,u) / focal_point_x_depth;
          //  cout<<"P3D.x = " << P3D_depth.at<float>(0,0)<<endl;
            P3D_depth.at<float>(1,0) = (v - center_of_projection_y_depth) * depthMap.at<float>(v,u) / focal_point_y_depth;
          //  cout<<"P3D.y = " << P3D_depth.at<float>(1,0)<<endl;
            P3D_depth.at<float>(2,0) = depthMap.at<float>(v,u);
          //  cout<<"P3D.z = " << P3D_depth.at<float>(2,0)<<endl;

            P3D_temp = rotMatDepthToRGB*P3D_depth;

        //    cout << "P3D nach Rotation: " << P3D_temp <<endl;
            P3D_rgb = P3D_temp+transVecDepthToRGB;
          //  cout << "P3D nach Translation: " << P3D_rgb<<endl;

            temp_u = (P3D_rgb.at<float>(0,0) * focal_point_x_rgb / P3D_rgb.at<float>(2,0)) + center_of_projection_x_rgb;
          //  cout << "temp_x: " << temp_u << endl;
            temp_v = (P3D_rgb.at<float>(1,0) * focal_point_y_rgb / P3D_rgb.at<float>(2,0)) + center_of_projection_y_rgb;
          //  cout << "temp_y: " << temp_v << endl;

            temp_u = roundf(temp_u);
          //  cout << "temp_x nach cast: " << temp_u << endl;
            temp_v = roundf(temp_v);
          //  cout << "temp_y nach cast: " << temp_v << endl;


            depthMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[0] = P3D_depth.at<float>(0,0);
            depthMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[1] = P3D_depth.at<float>(1,0);
            depthMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[2] = P3D_depth.at<float>(2,0);

         //   cout << "Teste Map: " << depthMap3Dcorresponded.at<Vec3f>(temp_v,temp_u) << endl;

           // cout << "=================================" << endl;

            rgbMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[0] = P3D_rgb.at<float>(0,0);
            rgbMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[1] = P3D_rgb.at<float>(1,0);
            rgbMap3Dcorresponded.at<Vec3f>(temp_v,temp_u)[2] = P3D_rgb.at<float>(2,0);
        }
    }

    log = SSTR("[DEBUG]: ...MAP DEPTH..." << depthMap << endl);
    Log(log);
    return depthMap3Dcorresponded;
}

vector<Line3D> BackProjection::calculateCorresponded3DLines(vector<Vec4i> lines_hough, Mat correspondedRGBtoDepth,Point2f pattern_mid) {
    vector<Line3D> depth3DLines;

   // Point3f pat_mid = calculatePatternMidInCam(pattern_mid);

    for (int i=0;i<lines_hough.size(); i++) {

        int temp_u_start = lines_hough.at(i)[0];
        cout << "temp_u_start: " << temp_u_start << endl;
        int temp_v_start = lines_hough.at(i)[1];
        cout << "temp_v_start: " << temp_v_start << endl;
        int temp_u_end = lines_hough.at(i)[2];
        int temp_v_end = lines_hough.at(i)[3];
        Line3D temp_line = Line3D::Line3D();

        temp_line.storeLine3D(correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[0],
                              correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[1],
                              correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[2],
                              correspondedRGBtoDepth.at<Vec3f>(temp_v_end,temp_u_end)[0],
                              correspondedRGBtoDepth.at<Vec3f>(temp_v_end,temp_u_end)[1],
                              correspondedRGBtoDepth.at<Vec3f>(temp_v_end,temp_u_end)[2]);

        cout << "temp_start: " << correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[0] << ", "
             << correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[1] << ", " << correspondedRGBtoDepth.at<Vec3f>(temp_v_start,temp_u_start)[2] << endl;

        depth3DLines.push_back(temp_line);
    }


    return depth3DLines;
}
*/

//=======================================================================================//
// for MANUAL INPUT with generated images
//=======================================================================================//

vector<Point3f> BackProjection::calculateBackProjectionManual(vector<Vec4i> lines2D, Mat depthImage) {
    this->depthImage.release();
    depthImage.copyTo(this->depthImage);

    float depth_z_start;
    float depth_z_end;
    Vec4i line2D;
    Point3f start_xyz;
    Point3f end_xyz;
    vector<Point3f> cam_lines3D;

    log = SSTR("========================================================\n");
    Log(log);

    for (int i=0; i<(int)lines2D.size();i++){
        line2D = lines2D.at(i);

        depth_z_start = findZInDepthMapManual(line2D.val[0],line2D.val[1]);
        depth_z_end = findZInDepthMapManual(line2D.val[2],line2D.val[3]);
        cout << "----------------\n";

        //z=0 check
        if (((int)depth_z_start <= 0 ) | ((int)depth_z_end <= 0)){

            log = SSTR("[DEBUG]: invalid z value: at line nr. " << i+1 << endl);
            Log(log);
            log = SSTR("[DEBUG]: ........................................" << endl);
            Log(log);

        }else{

            log = SSTR("[DEBUG]: start z value in camera coord: "<<depth_z_start<<" ----> end z value in camera coord: "<<depth_z_end<<endl);
            Log(log);

            start_xyz = calculateCameraXYZManual(line2D.val[0],line2D.val[1],depth_z_start);
            end_xyz = calculateCameraXYZManual(line2D.val[2], line2D.val[3], depth_z_end);
            cam_lines3D.push_back(start_xyz);
            cam_lines3D.push_back(end_xyz);

            log = SSTR("[DEBUG]: ...start_xyz in camera coord: "<<start_xyz<<" ----> end_xyz in camera coord: "<<end_xyz<<endl);
            Log(log);
            log = SSTR("[DEBUG]: ........................................" << endl);
            Log(log);
        }
    }

    log = SSTR("========================================================\n");
    Log(log);

    return cam_lines3D;
}

//=======================================================================================//

float BackProjection::findZInDepthMapManual(int x, int y) {

     //cout<<this->depthImage<<endl;
    float depth_z = this->depthImage.at<double>(Point(x,y));
   // depth_z = mapValues(depth_z);
    cout<<"show neighborhood at: "<<depth_z<<" -> x: "<<x<<" and y: "<<y<<endl;

    float depth_z1 = this->depthImage.at<double>(Point(x-2,y-2));
    float depth_z2 = this->depthImage.at<double>(Point(x-1,y-2));
    float depth_z3 = this->depthImage.at<double>(Point(x,y-2));
    float depth_z4 = this->depthImage.at<double>(Point(x+1,y-2));
    float depth_z5 = this->depthImage.at<double>(Point(x+2,y-2));

    float depth_z6 = this->depthImage.at<double>(Point(x-2,y-1));
    float depth_z7 = this->depthImage.at<double>(Point(x-2,y-1));
    float depth_z8 = this->depthImage.at<double>(Point(x,y-1));
    float depth_z9 = this->depthImage.at<double>(Point(x+1,y-1));
    float depth_z10 = this->depthImage.at<double>(Point(x+2,y-1));

    float depth_z11 = this->depthImage.at<double>(Point(x-2,y));
    float depth_z12 = this->depthImage.at<double>(Point(x-1,y));
    float depth_z14 = this->depthImage.at<double>(Point(x+1,y));
    float depth_z15 = this->depthImage.at<double>(Point(x+2,y));

    float depth_z16 = this->depthImage.at<double>(Point(x-2,y+1));
    float depth_z17 = this->depthImage.at<double>(Point(x-1,y+1));
    float depth_z18 = this->depthImage.at<double>(Point(x,y+1));
    float depth_z19 = this->depthImage.at<double>(Point(x+1,y+1));
    float depth_z20 = this->depthImage.at<double>(Point(x+2,y+1));

    float depth_z21 = this->depthImage.at<double>(Point(x-2,y+2));
    float depth_z22 = this->depthImage.at<double>(Point(x-1,y+2));
    float depth_z23 = this->depthImage.at<double>(Point(x,y+2));
    float depth_z24 = this->depthImage.at<double>(Point(x+1,y+2));
    float depth_z25 = this->depthImage.at<double>(Point(x+2,y+2));

//    1   2   3   4   5
//    6   7   8   9   10
//    11  12  13  14  15
//    16  17  18  19  20
//    21  22  23  24  25

    cout <<depth_z1<<", "<<depth_z2<<", "<<depth_z3<<", "<<depth_z4<<", "<<depth_z5<< endl;
    cout <<depth_z6<<", "<<depth_z7<<", "<<depth_z8<<", "<<depth_z9<<", "<<depth_z10<< endl;
    cout <<depth_z11<<", "<<depth_z12<<", "<<depth_z<<", "<<depth_z14<<", "<<depth_z15<< endl;
    cout <<depth_z16<<", "<<depth_z17<<", "<<depth_z18<<", "<<depth_z19<<", "<<depth_z20<< endl;
    cout <<depth_z21<<", "<<depth_z22<<", "<<depth_z23<<", "<<depth_z24<<", "<<depth_z25<< endl<<endl;


    log = SSTR("[DEBUG]: u: " << x << " and v: " << y << "result in z-value:" << depth_z << endl);
    Log(log);

    //accumulate in 3x3 neighborhood non-zero values
    vector<float> temp_pixel;
    float depth;
    for (int i=-1;i<=1;i++) {
        for (int j=-1; j<=1;j++) {
         //  depth = mapValues(this->depthImage.at<double>(Point(x+i,y+j)));
             depth = this->depthImage.at<double>(Point(x+i,y+j));
            if (depth!=1){
                temp_pixel.push_back(depth);
               // cout << depth << ",";
            }
        }
      //  cout << "" << endl;
    }

    if(!temp_pixel.empty()){

            //take the nearest value to the cam
            float result = *min_element(temp_pixel.begin(),temp_pixel.end());
            log = SSTR("[DEBUG]: calculated depth is: "<<result<<endl);
            Log(log);

            return result;

    }
    return 0.0f;
}

//=======================================================================================//

Point3f BackProjection::calculateCameraXYZManual(int x, int y, float depth_z) {

    float start_x_camera = (float)((x-(320.0f * depth_z)) /277.128f);
    float start_y_camera = (float)((y-(240.0f * depth_z)) /277.128f);
    Point3f xyz_camera = Point3f(start_x_camera, start_y_camera, depth_z);

    return xyz_camera;
}

//=======================================================================================//

float BackProjection::mapValues(float depth) {

    return depth * 57.0f + 3.0f;
}
