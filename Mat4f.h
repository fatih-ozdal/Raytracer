#ifndef MATF4F_H
#define MATF4F_H

#include "Vec3f.h"

struct Mat4f {
    float m[4][4];
    
    Mat4f() {
        // Identity matrix
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
    }
    
    static Mat4f identity() {
        return Mat4f();
    }
    
    Vec3f transformPoint(const Vec3f& p) const {
        float x = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3] * 1;  // ← translation applies
        float y = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3] * 1;
        float z = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3] * 1;
        float w = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3] * 1;
        return {x/w, y/w, z/w};  // Perspective divide
    }

    // Vector (direction): has w=0, NOT affected by translation
    Vec3f transformVector(const Vec3f& v) const {
        float x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * 0;  // ← no translation
        float y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * 0;
        float z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * 0;
        // No w component needed, vectors don't divide
        return {x, y, z};
    }
    
    Mat4f operator*(const Mat4f& other) const {
        Mat4f result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }
    
    Mat4f inverse() const;  // Implement matrix inversion
    Mat4f transpose() const;  // Implement transpose
};

#endif // MATF4F_H