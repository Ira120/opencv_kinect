#ifndef GROUPLINES3D_H
#define GROUPLINES3D_H

#include "Tools.h"
#include "Line3D.h"
#include "Log.h"
#include <math.h>
#include <numeric>
#include <cmath>
#include <functional>
#include "SimilarityMeasure.h"
#include "EdgeModel.h"

//C++-Kochbuch - D. Ryan Stephens - O'Reilly Germany, 2006
template<class Value_T>
struct DiffSquared {
    Value_T operator()(Value_T x, Value_T y) const {
        return (x-y) * (x-y);
    }
};

template<class Iter_T, class Iter2_T>
double vectorDistance(Iter_T first, Iter_T last, Iter2_T first2) {
    double ret = inner_product(first, last, first2, 0.0L, plus<double>(), DiffSquared<double>());
    return ret > 0.0 ? sqrt(ret) : 0.0;
}


class GroupLines3D {
public:
    //methods
    GroupLines3D();
    ~GroupLines3D();
    float cosineSimilarity(Line3D first, Line3D second);
    float euclideanDistance(Line3D first, Line3D second);
    int findSimilarLines(vector<vector<Line3D> > lines_vector, int frame_nr);
    int groupSimilarLines(int frame_nr);

    //parameters
    vector<vector<Line3D> > final_lines_vector;
    vector<Line3D> groped_lines;
    string log;
};

#endif // GROUPLINES3D_H
