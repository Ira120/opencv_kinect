#include "BackProjection.h"

BackProjection::BackProjection()
{
    focal_point = 5.2903256370810561e+02;
    center_of_projection_x = 3.1950000000000000e+02;
    center_of_projection_y = 2.3950000000000000e+02;

    cameraMatrix = mat3(focal_point, 0.0, center_of_projection_x,
                             0.0, focal_point, center_of_projection_y,
                             0.0, 0.0, 1.0);
}

BackProjection::~BackProjection()
{

}

float BackProjection::findZInDepthMap(int x, int y, Mat depthImage){

    float depth_z = depthImage.at<float>(Point(x,y));

    return depth_z;
}

Point3f BackProjection::calculateCameraXYZ(int x, int y, float depth_z){

    float start_x_camera = ((float)x - center_of_projection_x) * depth_z / focal_point;
    float start_y_camera = ((float)y - center_of_projection_y) * depth_z / focal_point;

    Point3f xyz_camera = Point3f(start_x_camera, start_y_camera, depth_z);
    return xyz_camera;
}

vector<int16_t> BackProjection::store3DLine(Point xy){

    vector<int16_t> line;
    int x = xy.x;
    int y = xy.y;
    line.push_back(x);
    line.push_back(y);
    line.push_back(depth_z_start);
    return line;
}

void BackProjection::calculateBackProejction(vector<Vec4i> lines2D, Mat depthImage){

    Vec4i line2D;
    Point3f start_xyz;
    Point3f end_xyz;

    QString filename_lines = "/Users/irina/Develop/workspace/bachelor_1/3Dlines.txt";
    QFile file (filename_lines);

    depthImage.convertTo(depthImage, CV_32F); // convert the image data to float type

    if (file.open(QIODevice::ReadWrite))
    {
        QTextStream stream (&file);
        stream << "3Dlines pro frame" << endl;
        stream << "===========================================" << endl;
        stream << "Found about "<<lines2D.size()<<" lines with hough"<<endl;
        stream << "===========================================" << endl;

    for (int i=0; i<lines2D.size();i++){
         line2D = lines2D.at(i);

         depth_z_start = findZInDepthMap(line2D.val[0],line2D.val[1],depthImage);
         depth_z_end = findZInDepthMap(line2D.val[2],line2D.val[3],depthImage);

         start_xyz = calculateCameraXYZ(line2D.val[0],line2D[1],depth_z_start);
         end_xyz = calculateCameraXYZ(line2D.val[2], line2D[3], depth_z_end);
         stream << i+1 << "start_point ( "
                << start_xyz.x << "|" << start_xyz.y << "|" << start_xyz.z
                << " ) --------> end_point ( "
                << end_xyz.x << "|" << end_xyz.y << "|" << end_xyz.z << " )" <<  endl;
    }
    file.close();
    }

}
