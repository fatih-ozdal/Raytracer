#ifndef VEC3F_H
#define VEC3F_H

#include <cmath>
#include "MathF.h"

struct Vec3f
{
    /* data */
    float x, y, z;

    /* functions */
    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    inline float dotProduct(const Vec3f& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    inline Vec3f crossProduct(const Vec3f& rhs) const noexcept {
        return {y * rhs.z - z * rhs.y, 
                z * rhs.x - x * rhs.z, 
                x * rhs.y - y * rhs.x};
    }

    inline float length() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }

    inline float inverse_length() const noexcept {
        float length_squared = x * x + y * y + z * z;
        return Q_rsqrt(length_squared);      
    }

    inline Vec3f normalize() const noexcept {
        float inverseLen = inverse_length();
        if (inverseLen == 0.0f) return {0.f, 0.f, 0.f};
        return *this * inverseLen;
    }

    inline Vec3f operator+(const Vec3f& rhs) const noexcept{
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    inline Vec3f operator-(const Vec3f& rhs) const noexcept{
        return {x - rhs.x,  y - rhs.y, z - rhs.z};
    }
    
    inline Vec3f operator*(float val) const noexcept {
        return {x * val, y * val, z * val};
    }

    inline friend Vec3f operator*(float val, const Vec3f& v) noexcept {
        return v * val;
    }
    
    inline Vec3f elwiseMult(const Vec3f& rhs) const noexcept {
        return {x * rhs.x, y * rhs.y, z * rhs.z};
    }

    inline Vec3f operator/(float val) const noexcept {
        return {x / val, y / val, z / val};
    }

    inline Vec3f& operator+=(const Vec3f& v) noexcept {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    float operator[](int idx) const noexcept {
        assert(idx >= 0 && idx < 3);
        return (&x)[idx];
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec3f& v);
};

inline std::ostream& operator<<(std::ostream& os, const Vec3f& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

// compiler expands this to what det3x3() does
inline float det3cols(const Vec3f& a, const Vec3f& b, const Vec3f& c) noexcept {
    return a.dotProduct(b.crossProduct(c));
}

// compiler expands this to what det3x3() does
inline float det3rows(const Vec3f& a, const Vec3f& b, const Vec3f& c) noexcept {
    return a.crossProduct(b).dotProduct(c);
}

#endif // VEC3F_H
