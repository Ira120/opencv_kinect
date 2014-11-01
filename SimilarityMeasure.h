#ifndef SIMILARITYMEASURE_H
#define SIMILARITYMEASURE_H

#include "Tools.h"
#include "Log.h"
#include "Line3D.h"

class SimilarityMeasure {
public:
    //methods
    SimilarityMeasure();
    ~SimilarityMeasure();
    SimilarityMeasure(Line3D first,int i_first,Line3D second,int i_second,float measure);

    //parameters
    Line3D first;
    Line3D second;
    int i_first;
    int i_second;
    float measure;
};

#endif // SIMILARITYMEASURE_H
