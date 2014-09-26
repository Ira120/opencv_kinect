#ifndef BACKPROJECTION_H
#define BACKPROJECTION_H

#include "Tools.h"

class BackProjection
{
public:
    BackProjection();

    ~BackProjection();

    // TODO: BackProjection - find x and y in camera coordinates

    float findZInDepthMap(int x, int y, Mat depthImage);
    Point3f calculateCameraXYZ (int x, int y, float depth_z);
    vector<int16_t> store3DLine (Point xy);
    void calculateBackProejction (vector<Vec4i> lines2D, Mat depthImage);

private:

    float focal_point;
    float center_of_projection_x;
    float center_of_projection_y;

    mat3 cameraMatrix;
    float depth_z_start;
    float depth_z_end;

    Mat depthImage;

    // TODO: Application app

};

#endif // BACKPROJECTION_H
