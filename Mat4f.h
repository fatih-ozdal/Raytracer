#ifndef MATF4F_H
#define MATF4F_H

#include "Vec3f.h"
#include <iostream>

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
    
    Mat4f transpose() const {
        Mat4f result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }

    Mat4f inverse() const {
        // Using Gauss-Jordan elimination for 4x4 matrix inversion
        float temp[4][8];
        
        // Initialize augmented matrix [M | I]
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                temp[i][j] = m[i][j];
                temp[i][j+4] = (i == j) ? 1.0f : 0.0f;
            }
        }
        
        // Forward elimination
        for (int i = 0; i < 4; i++) {
            // Find pivot
            int pivot = i;
            for (int j = i + 1; j < 4; j++) {
                if (fabs(temp[j][i]) > fabs(temp[pivot][i])) {
                    pivot = j;
                }
            }
            
            // Swap rows
            if (pivot != i) {
                for (int j = 0; j < 8; j++) {
                    float t = temp[i][j];
                    temp[i][j] = temp[pivot][j];
                    temp[pivot][j] = t;
                }
            }
            
            // Scale pivot row
            float scale = temp[i][i];
            if (fabs(scale) < 1e-8f) {
                // Matrix is singular, return identity
                return Mat4f::identity();
            }
            
            for (int j = 0; j < 8; j++) {
                temp[i][j] /= scale;
            }
            
            // Eliminate column
            for (int j = 0; j < 4; j++) {
                if (i != j) {
                    float factor = temp[j][i];
                    for (int k = 0; k < 8; k++) {
                        temp[j][k] -= factor * temp[i][k];
                    }
                }
            }
        }
        
        // Extract result from right half of augmented matrix
        Mat4f result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = temp[i][j+4];
            }
        }
        
        return result;
    }

    // Calculate determinant of upper-left 3x3 (for checking if transformation has reflection)
    float determinant3x3() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    // Check if this transformation includes a reflection (negative scale)
    bool hasReflection() const {
        return determinant3x3() < 0.0f;
    }

    void print() const  {
        std::cout << m[0][0] << " " << m[0][1] << " " << m[0][2] << " " << m[0][3] << std::endl
                << m[1][0] << " " << m[1][1] << " " << m[1][2] << " " << m[1][3] << std::endl
                << m[2][0] << " " << m[2][1] << " " << m[2][2] << " " << m[2][3] << std::endl
                << m[3][0] << " " << m[3][1] << " " << m[3][2] << " " << m[3][3] << std::endl;
    }
};

#endif // MATF4F_H