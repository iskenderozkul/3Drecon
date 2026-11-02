#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace recon {

constexpr float kPi = 3.14159265358979323846f;

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }
    Vec3 normalized() const {
        const float l = length();
        return l > 1e-8f ? Vec3{x / l, y / l, z / l} : Vec3{0.f, 0.f, 1.f};
    }
};

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float dot(const Vec3& a, const Vec3& b) { return a.dot(b); }

struct Mat3 {
    // Row-major: m[row * 3 + col]
    float m[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};

    static Mat3 identity() { return Mat3{}; }

    float at(int r, int c) const { return m[r * 3 + c]; }
    float& at(int r, int c) { return m[r * 3 + c]; }

    Mat3 transposed() const {
        Mat3 out;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) out.m[r * 3 + c] = m[c * 3 + r];
        return out;
    }

    Vec3 operator*(const Vec3& v) const {
        return {m[0] * v.x + m[1] * v.y + m[2] * v.z,
                m[3] * v.x + m[4] * v.y + m[5] * v.z,
                m[6] * v.x + m[7] * v.y + m[8] * v.z};
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 out;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out.m[r * 3 + c] = m[r * 3 + 0] * o.m[0 * 3 + c] +
                                   m[r * 3 + 1] * o.m[1 * 3 + c] +
                                   m[r * 3 + 2] * o.m[2 * 3 + c];
            }
        }
        return out;
    }
};

inline Mat3 matFromQuat(float x, float y, float z, float w) {
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z;
    const float wx = w * x, wy = w * y, wz = w * z;
    Mat3 r;
    r.m[0] = 1.f - 2.f * (yy + zz);
    r.m[1] = 2.f * (xy - wz);
    r.m[2] = 2.f * (xz + wy);
    r.m[3] = 2.f * (xy + wz);
    r.m[4] = 1.f - 2.f * (xx + zz);
    r.m[5] = 2.f * (yz - wx);
    r.m[6] = 2.f * (xz - wy);
    r.m[7] = 2.f * (yz + wx);
    r.m[8] = 1.f - 2.f * (xx + yy);
    return r;
}

// ARCore camera space is OpenGL-style (+X right, +Y up, -Z forward).
// The depth pipeline uses an OpenCV-style camera (+X right, +Y down, +Z forward).
// R_cv = R_gl * diag(1, -1, -1)
inline Mat3 arcoreToCvRotation(const Mat3& rGl) {
    Mat3 r = rGl;
    r.m[1] = -r.m[1];
    r.m[4] = -r.m[4];
    r.m[7] = -r.m[7];
    r.m[2] = -r.m[2];
    r.m[5] = -r.m[5];
    r.m[8] = -r.m[8];
    return r;
}

inline Mat3 cvToArcoreRotation(const Mat3& rCv) {
    return arcoreToCvRotation(rCv);
}

struct Intrinsics {
    float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
    int width = 0, height = 0;

    Vec3 unproject(float u, float v, float depth) const {
        return {(u - cx) * depth / fx, (v - cy) * depth / fy, depth};
    }
};

// Column-major 4x4 (OpenGL layout), matching ARCore camera matrices.
struct Mat4 {
    float m[16] = {1.f, 0.f, 0.f, 0.f,
                   0.f, 1.f, 0.f, 0.f,
                   0.f, 0.f, 1.f, 0.f,
                   0.f, 0.f, 0.f, 1.f};

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int c = 0; c < 4; ++c) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.f;
                for (int k = 0; k < 4; ++k) sum += m[k * 4 + row] * o.m[c * 4 + k];
                r.m[c * 4 + row] = sum;
            }
        }
        return r;
    }
};

inline Vec3 rotate(const Mat3& r, const Vec3& v) { return r * v; }

inline float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

inline int floorDiv(int value, int divisor) {
    int q = value / divisor;
    int r = value % divisor;
    if (r != 0 && ((r < 0) != (divisor < 0))) --q;
    return q;
}

}  // namespace recon
