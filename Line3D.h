#ifndef LINE3D_H
#define LINE3D_H

#include "Tools.h"

class Line3D {
public:
    //methods
    Line3D();
    ~Line3D();
    void storeLine3D(float start_x, float start_y, float start_z, float end_x, float end_y, float end_z);
    void storeLine3D(Point3f start_point, Point3f end_point);
    vector<Point3f> getLine3D();
    Point3f getStartPointOfLine3D();
    Point3f getEndPointOfLine3D();
    void showLine3D();

private:
    //parameters
    Point3f start_point;
    Point3f end_point;
};

#endif // LINE3D_H
