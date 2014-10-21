#ifndef EDGEMODEL_H
#define EDGEMODEL_H

#include "Line3D.h"
#include "Tools.h"
#include "Log.h"

class EdgeModel {
public:
    //parameters
    vector<Line3D> lines3DproFrame;
    vector<vector<Line3D> > line3Dall;
    string log;

    //methods
    EdgeModel();
    ~EdgeModel();
    void createOBJ(int frame_nr);
};

#endif // EDGEMODEL_H
