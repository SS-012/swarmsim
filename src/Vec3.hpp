#pragma once
#include <cmath>

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const       { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const       { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s)       { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(float s)       { x /= s; y /= s; z /= s; return *this; }

    float norm() const { return std::sqrt(x*x + y*y + z*z); }

    Vec3 normalized() const {
        float n = norm();
        return (n > 1e-6f) ? (*this / n) : Vec3{};
    }

    static float dot(const Vec3& a, const Vec3& b) {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }
};