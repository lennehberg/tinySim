#ifndef FEATURE_H
#define FEATURE_H

#include "tinysim/math/Vec2.h"

typedef struct FeatureVertex{

    Vec2 position;
    int id;

}FeatureVertex;

typedef struct Feature{

    FeatureVertex vertices[2];
    int count; // 1 or 2

    bool isPoint() const {return count == 1;}
    bool isEdge() const {return count == 2;}

}Feature;

#endif // FEATURE_H
