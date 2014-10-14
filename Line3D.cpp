#include "Line3D.h"

//=======================================================================================//

Line3D::Line3D() {}

//=======================================================================================//

Line3D::~Line3D() {}

//=======================================================================================//

void Line3D::storeLine3D(float start_x, float start_y, float start_z, float end_x, float end_y, float end_z) {
    start_point.x = start_x;
    start_point.y = start_y;
    start_point.z = start_z;
    end_point.x = end_x;
    end_point.y = end_y;
    end_point.z = end_z;
}

//=======================================================================================//

void Line3D::storeLine3D(Point3f start_point, Point3f end_point) {
    this->start_point = start_point;
    this->end_point = end_point;
}

//=======================================================================================//

vector<Point3f> Line3D::getLine3D() {
    vector<Point3f> temp_line;
    temp_line.push_back(start_point);
    temp_line.push_back(end_point);
    return temp_line;
}

//=======================================================================================//

Point3f Line3D::getStartPointOfLine3D() {
    return start_point;
}

//=======================================================================================//

Point3f Line3D::getEndPointOfLine3D() {
    return end_point;
}

//=======================================================================================//s

void Line3D::showLine3D() {
    cout<<"show 3DLine: start_x: "<<start_point.x<<"; start_y: "<<start_point.y<<"; start_z: "<<start_point.z
            <<" | end_x: "<<end_point.x<<"; end_y: "<<end_point.y<<"; end_z: "<<end_point.z<<endl;
}

//=======================================================================================//

void Line3D::drawLine3D() {

}
