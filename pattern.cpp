//http://xanthippi.ceid.upatras.gr/people/evangelidis/arma/

#include "Pattern.h"
#include <iostream>
using namespace cv;
using namespace std;

#include "opencv2/calib3d/calib3d.hpp"


namespace ARma {

//=======================================================================================//

	Pattern::Pattern(double param1){
		id =-1;
		size = param1;
		orientation = -1;
		confidence = -1;
        rotVec = Mat::zeros( 1, 3, CV_64FC1 );
        transVec = Mat::zeros( 1, 3, CV_64FC1 );
        rotMat = Mat::eye(3, 3, CV_64F);
	}

//=======================================================================================//

    Pattern::~Pattern() {}

//=======================================================================================//

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
    void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);		
	}

//=======================================================================================//

	void Pattern::showPattern()
	{
        log = SSTR("========================================================\n"
                    << "[DEBUG]: PATTERN INFORMATION\n"
                    << "Pattern ID: " << id << endl
                    << "Pattern Size: " << size << endl
                    << "Pattern Confedince Value: " << confidence << endl
                    << "Pattern Orientation: " << orientation << endl
                    << "========================================================\n");
        Log(log);

	}

//=======================================================================================//

    void Pattern::getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{
        Mat intrinsics = cameraMatrix;
        //if distortionions = 0
        Mat disCoeff = (Mat_<float>(5,1) << 0, 0, 0, 0, 0);

        Point2f pat2DPts[4];

        log = SSTR("========================================================\n[DEBUG]: Marker corners in the image (uv-values): ");
        Log(log);

		for (int i = 0; i<4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;

            pattern_origin.push_back(pat2DPts[i]);

            log = SSTR("("<<pat2DPts[i].x<<"|"<<pat2DPts[i].y<<")  ");
            Log(log);
		}

        log = SSTR(endl);
        Log(log);

		//3D points in pattern coordinate system
        Point3f pat3DPts[4];
        /*
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;
        */

        //for left-handed CS (x -> right; y -> up; z -> into the wall)
        pat3DPts[0].x = 0.0;
        pat3DPts[0].y = patternSize;
        pat3DPts[0].z = 0.0;
        pat3DPts[1].x = patternSize;
        pat3DPts[1].y = patternSize;
        pat3DPts[1].z = 0.0;
        pat3DPts[2].x = patternSize;
        pat3DPts[2].y = 0.0;
        pat3DPts[2].z = 0.0;
        pat3DPts[3].x = 0.0;
        pat3DPts[3].y = 0.0;
        pat3DPts[3].z = 0.0;

        Mat objectPts(4, 3, CV_32F, pat3DPts);
        Mat imagePts(4, 2, CV_32F, pat2DPts);
		//find extrinsic parameters
        solvePnPRansac(objectPts, imagePts, intrinsics, disCoeff, rotVec, transVec,false,100,1.2,0.99,noArray(),ITERATIVE);
       // solvePnP(objectPts, imagePts, intrinsics, disCoeff, rotVec, transVec,false,ITERATIVE);


        rotationMatrix(rotVec, rotMat);

        log = SSTR("========================================================\n"
                   << "[DEBUG]: EXTRINSIC PARAMETERS \nrotation vector:\n" << rotVec << endl
                   << "\nrotation matrix:\n" << rotMat << endl
                   << "\ntranslation vector: \n" << transVec << endl
                   << "========================================================\n");
        Log(log);
	}

//=======================================================================================//

    void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix) {

        Mat disCoeff = (Mat_<float>(5,1) << 0, 0, 0, 0, 0);
        Scalar color = Scalar(255,0,255);

		//model 3D points: they must be projected to the image plane
		Mat modelPts = (Mat_<float>(8,3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
            0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size );

		std::vector<cv::Point2f> model2ImagePts;
        //project model 3D points to the image. Points through the transformation matrix
        //(defined by rotVec and transVec) "are transfered" from the pattern CS to the
        //camera CS, and then, points are projected using camera parameters
        //(camera matrix, distortion matrix) from the camera 3D CS to its image plane

        projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);


		//draw cube, or whatever
		int i;
		for (i =0; i<4; i++){
            cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), color, 1);
            cout<<"Punkte1 :"<<model2ImagePts.at(i%4)<<", " <<model2ImagePts.at((i+1)%4)<<endl;
		}
        for (i =4; i<7; i++){
            cv::line(frame, model2ImagePts.at(i%8), model2ImagePts.at((i+1)%8), color, 1);
                        cout<<"Punkte2 :"<<model2ImagePts.at(i%8)<<", " <<model2ImagePts.at((i+1)%8)<<endl;
        }
       cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), color, 1);
       for (i =0; i<4; i++){
            cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i+4), color, 1);
                                cout<<"Punkte3 :"<<model2ImagePts.at(i)<<endl;
        }
		
		//draw the line that reflects the orientation. It indicates the bottom side of the pattern
        cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), Scalar(80,255,80), 1);

		model2ImagePts.clear();
	}

}

