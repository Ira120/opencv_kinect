#ifndef EDGEMODEL_H
#define EDGEMODEL_H

#include "Line3D.h"
#include "Tools.h"

class EdgeModel {
public:
    EdgeModel();
    ~EdgeModel();
    void createOBJ(int frame_nr);
    vector<Line3D> lines3DproFrame;
};

#endif // EDGEMODEL_H
