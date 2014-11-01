#include "SimilarityMeasure.h"

//=======================================================================================//

SimilarityMeasure::SimilarityMeasure() {}

//=======================================================================================//

SimilarityMeasure::~SimilarityMeasure() {}

//=======================================================================================//

SimilarityMeasure::SimilarityMeasure(Line3D first,int i_first,Line3D second,int i_second,float measure) {
    this->first = first;
    this->second = second;
    this->i_first = i_first;
    this->i_second = i_second;
    this->measure = measure;
}
