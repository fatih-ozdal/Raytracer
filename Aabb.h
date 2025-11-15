#ifndef AABB_H
#define AABB_H

#include "Vec3f.h"
#include <algorithm>

struct AABB
{
    Vec3f min, max;

    void reset() noexcept
    {
        min = {FLT_MAX, FLT_MAX, FLT_MAX};
        max = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    }

    void expand(const Vec3f& p) noexcept
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);

        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    void expand(const AABB& b) noexcept
    {
        min.x = std::min(min.x, b.min.x);
        min.y = std::min(min.y, b.min.y);
        min.z = std::min(min.z, b.min.z);

        max.x = std::max(max.x, b.max.x);
        max.y = std::max(max.y, b.max.y);
        max.z = std::max(max.z, b.max.z);
    }
};

#endif // AABB_H
