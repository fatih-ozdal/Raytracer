#ifndef VEC2F_H
#define VEC2F_H

struct Vec2f {
    float u, v;
    
    Vec2f() : u(0), v(0) {}
    Vec2f(float u_, float v_) : u(u_), v(v_) {}
    
    Vec2f operator+(const Vec2f& other) const 
    {
        return Vec2f(u + other.u, v + other.v);
    }
    
    Vec2f operator*(float scalar) const 
    {
        return Vec2f(u * scalar, v * scalar);
    }
};

#endif // VEC2F_H
