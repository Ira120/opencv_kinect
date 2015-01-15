#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include "Tools.h"
#include "Log.h"
#include "EdgeDetection.h"

class BackProjection {
public:
    //parameters
    vector<Point3f> camera_lines3D;
    vector<Point3f> pat_origin3D;
    string log;

    //methods
    BackProjection();
    ~BackProjection();
    float findZInDepthMap(int x, int y);
    Point3f calculateCameraXYZ (int x, int y, float depth_z, Mat cloudMap);
    vector<Point3f> calculateBackProjection (vector<Vec4i> lines2D, Mat depthImage, vector<Point2f> pattern_points, Mat cloudMap);
    void calculateAverangeDepth();
    void calculatePatternOriginInCam (vector<Point2f> pattern_origin);
    //manual input
    float findZInDepthMapManual(int x, int y);
    Point3f calculateCameraXYZManual (int x, int y, float depth_z);
    vector<Point3f> calculateBackProjectionManual (vector<Vec4i> lines2D, Mat depthImage);
    float mapValues(float depth);
   // Mat calculateTranslatedRGBMap (Mat depthMap);
   // vector<Line3D> calculateCorresponded3DLines(vector<Vec4i> lines_hough, Mat correspondedRGBtoDepth, Point2f pattern_origin);

private:
    //parameters
    float focal_point_x_rgb;
    float focal_point_y_rgb;
    float center_of_projection_x_rgb;
    float center_of_projection_y_rgb;

    float focal_point_x_depth;
    float focal_point_y_depth;
    float center_of_projection_x_depth;
    float center_of_projection_y_depth;

    Mat rotMatDepthToRGB;
    Mat transVecDepthToRGB;

    Mat depthImage;

    float averangeDepth;

    Point3f cloud_point;

};

#endif // BACKPROJECTION_H
