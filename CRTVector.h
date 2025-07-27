#pragma once
#include <cmath>
#include <algorithm>
class CRTVector
{
public:
    CRTVector() : x(0.0f), y(0.0f), z(0.0f) {}
    CRTVector(float x1, float y1, float z1)
        : x(x1), y(y1), z(z1) {}

    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    CRTVector normalize() const {
        float len = length();
        return len > 0.0f ? CRTVector(x / len, y / len, z / len) : CRTVector(0, 0, 0);
    }

    CRTVector operator+(const CRTVector& rhs) const {
        return CRTVector(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    CRTVector& operator+=(const CRTVector& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    CRTVector operator-(const CRTVector& rhs) const {
        return CRTVector(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    CRTVector operator-() const {
        return CRTVector(-x, -y, -z);
    }

    CRTVector operator*(float s) const {
        return CRTVector(x * s, y * s, z * s);
    }

    CRTVector& operator*=(float s) {
        x *= s; y *= s; z *= s;
        return *this;
    }

    float lengthSquared() const {
    return x * x + y * y + z * z;
    }

    CRTVector cross(const CRTVector& rhs) const {
        return CRTVector(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    float dot(const CRTVector& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    friend CRTVector operator*(float s, const CRTVector& v) {
        return v * s;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

private:
    float x;
    float y;
    float z;
};
