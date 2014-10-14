#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include "Tools.h"

class BackProjection {
public:
    vector<Point3f> camera_lines3D;

    BackProjection();
    ~BackProjection();
    float findZInDepthMap(int x, int y);
    Point3f calculateCameraXYZ (int x, int y, float depth_z);
    void calculateBackProjection (vector<Vec4i> lines2D, Mat depthImage);

private:
    float focal_point;
    float center_of_projection_x;
    float center_of_projection_y;
    mat3 cameraMatrix;
    Mat depthImage;
};

#endif // BACKPROJECTION_H
