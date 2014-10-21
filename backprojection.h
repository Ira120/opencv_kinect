#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include "Tools.h"
#include "Log.h"

class BackProjection {
public:
    //parameters
    vector<Point3f> camera_lines3D;
    string log;

    //methods
    BackProjection();
    ~BackProjection();
    float findZInDepthMap(int x, int y);
    Point3f calculateCameraXYZ (int x, int y, float depth_z);
    void calculateBackProjection (vector<Vec4i> lines2D, Mat depthImage);

private:
    //parameters
    float focal_point;
    float center_of_projection_x;
    float center_of_projection_y;
    Mat depthImage;
};

#endif // BACKPROJECTION_H
