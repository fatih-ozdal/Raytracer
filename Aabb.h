#ifndef AABB_H
#define AABB_H

#include "Vec3f.h"
#include <algorithm>

struct AABB
{
    Vec3f min, max;

    void reset();
    void expand(const Vec3f& p);
    void expand(const AABB& b);
};

#endif // AABB_H
