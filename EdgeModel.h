#ifndef EDGEMODEL_H
#define EDGEMODEL_H

#include "Line3D.h"
#include "Tools.h"
#include "Log.h"

class EdgeModel {
public:
    //parameters
    vector<vector<Line3D> > line3Dall;
    string log;

    //methods
    EdgeModel();
    ~EdgeModel();
    void createOBJproFrame(vector<Line3D> lines3DproFrame,int frame_nr);
    void createOBJfinal();
    void createOBJgrouped(vector<Line3D> grouped_lines);

};

#endif // EDGEMODEL_H
