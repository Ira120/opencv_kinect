#ifndef LINE3D_H
#define LINE3D_H

#include "Tools.h"

class Line3D
{
public:
    Line3D();
    ~Line3D();

private:
    Point3f start_point;
    Point3f end_point;
};

#endif // LINE3D_H
