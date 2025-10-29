#ifndef MATHF_H
#define MATHF_H

#include <algorithm>
#include "BitCast.h"
#include "Vec3f.h"

#define FLT_MAX 3.402823466e+38F

float clampF(float val, float min, float max) {
    return std::min(std::max(val, min), max);
}

inline float Q_rsqrt(float x) {
    if (x <= 0.0f) return 0.0f;

    uint32_t i = portable_bit_cast<uint32_t>(x);
    i = 0x5f3759dfu - (i >> 1);
    float y = portable_bit_cast<float>(i);
    
    // one Newton–Raphson iteration
    y = y * (1.5f - 0.5f * x * y * y);
    return y;
}

inline float det3x3(
    float a11, float a12, float a13,
    float a21, float a22, float a23,
    float a31, float a32, float a33)
{
    return a11 * (a22 * a33 - a23 * a32)
         - a12 * (a21 * a33 - a23 * a31)
         + a13 * (a21 * a32 - a22 * a31);
}

// compiler expands this to what det3x3() does
inline float det3cols(const Vec3f& a, const Vec3f& b, const Vec3f& c) {
    return a.dotProduct(b.crossProduct(c));
}

// compiler expands this to what det3x3() does
inline float det3rows(const Vec3f& a, const Vec3f& b, const Vec3f& c) {
    return a.crossProduct(b).dotProduct(c);
}

#endif // MATHF_H
