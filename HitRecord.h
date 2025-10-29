#ifndef HITRECORD_H
#define HITRECORD_H

#include "parser.h"

// uint8_t causes a cast (8-bit -> 32-bit zero-extend) when we do arithmetic, comparison, or switch
enum class ObjectType : uint32_t 
{
    Mesh = 0,
    Triangle,
    Sphere,
    Plane,
};

typedef struct {
    //float  bary_beta, bary_gamma; // barycentrics for tris and meshes (optional), 0.0f if not used
    int materialId;
    Vec3f intersectionPoint;
    Vec3f normal;
    ObjectType type;
} HitRecord;

#endif // HITRECORD_H