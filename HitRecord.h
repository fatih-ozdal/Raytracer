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

std::ostream& operator<<(std::ostream& os, PrimKind type) {
    switch (type) {
        case PrimKind::Mesh:     return os << "Mesh";
        case PrimKind::Triangle: return os << "Triangle";
        case PrimKind::Sphere:   return os << "Sphere";
        case PrimKind::Plane:    return os << "Plane";
        default:                 return os << "Unknown";
    }
}

struct HitRecord 
{
    int materialId;
    Vec3f intersectionPoint;
    Vec3f normal;
    PrimKind kind;
};

#endif // HITRECORD_H