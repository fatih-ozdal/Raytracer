#ifndef HITRECORD_H
#define HITRECORD_H

#include "parser.h"

// uint8_t causes a cast (8-bit -> 32-bit zero-extend) when we do arithmetic, comparison, or switch
enum class PrimKind : uint32_t 
{
    Mesh = 0,
    Triangle,
    Sphere,
    Plane,
};

struct HitRecord 
{
    int materialId;
    Vec3f intersectionPoint;
    Vec3f normal;
    PrimKind kind;
};

struct PrimRef
{
    PrimKind kind;   // what type of object this refers to
    uint32_t index;  // index into the corresponding object array (spheres[], meshes[], etc.)

    PrimRef() = default;
    PrimRef(PrimKind k, uint32_t i) : kind(k), index(i) {}
};

#endif // HITRECORD_H