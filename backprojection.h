#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include "Tools.h"
#include "Log.h"
#include "EdgeDetection.h"

class BackProjection {
public:
    //parameters
    vector<Point3f> camera_lines3D;
    string log;

    //methods
    BackProjection();
    ~BackProjection();
    float findZInDepthMap(int x, int y);
    float findZInDepthMapManual(int x, int y);
    Point3f calculateCameraXYZ (int x, int y, float depth_z);
    Point3f calculateCameraXYZManual (int x, int y, float depth_z);
    void calculateBackProjection (vector<Vec4i> lines2D, Mat depthImage, Point2f pattern_mid);
    void calculateBackProjectionManual (vector<Vec4i> lines2D, Mat depthImage, Point2f pattern_mid);
    Point3f calculatePatternOriginInCam (Point2f pattern_origin);
    Mat calculateTranslatedRGBMap (Mat depthMap);
    vector<Line3D> calculateCorresponded3DLines(vector<Vec4i> lines_hough, Mat correspondedRGBtoDepth, Point2f pattern_origin);
    void calculateAverangeDepth();

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
};

#endif // BACKPROJECTION_H
