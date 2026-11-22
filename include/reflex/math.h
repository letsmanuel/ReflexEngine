#pragma once
/// @file math.h
/// Minimal math types (right-handed, Y up, metres, kilograms, seconds).
/// The character's local frame is: +Z forward, +Y up, +X = the character's LEFT.

#include <algorithm>
#include <cmath>

namespace reflex {

constexpr float kPi = 3.14159265358979f;
constexpr float kGravity = 9.81f;

inline float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float Saturate(float v) { return Clamp(v, 0.0f, 1.0f); }
inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float SmoothStep(float t) { t = Saturate(t); return t * t * (3.0f - 2.0f * t); }

struct Vec3 {
    float x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};
inline constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline constexpr float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline constexpr Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }
inline float LengthSq(const Vec3& v) { return Dot(v, v); }
inline Vec3 Normalize(const Vec3& v, const Vec3& fallback = {0, 1, 0}) {
    float l = Length(v);
    return l > 1e-9f ? v / l : fallback;
}
inline Vec3 Lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }
inline Vec3 Horizontal(const Vec3& v) { return {v.x, 0, v.z}; }
inline Vec3 ClampLength(const Vec3& v, float maxLen) {
    float l = Length(v);
    return l > maxLen && l > 1e-9f ? v * (maxLen / l) : v;
}

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
    constexpr Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    static Quat Identity() { return {}; }
    static Quat FromAxisAngle(const Vec3& axis, float angle) {
        Vec3 a = Normalize(axis, {0, 1, 0});
        float s = std::sin(angle * 0.5f);
        return {a.x * s, a.y * s, a.z * s, std::cos(angle * 0.5f)};
    }
    /// Rotation vector (axis * angle) to quaternion.
    static Quat FromRotationVector(const Vec3& rv) {
        float a = Length(rv);
        return a < 1e-8f ? Quat{rv.x * 0.5f, rv.y * 0.5f, rv.z * 0.5f, 1.0f} : FromAxisAngle(rv / a, a);
    }
    constexpr Quat operator*(const Quat& o) const {
        return {w * o.x + x * o.w + y * o.z - z * o.y, w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w, w * o.w - x * o.x - y * o.y - z * o.z};
    }
    constexpr Quat Conjugate() const { return {-x, -y, -z, w}; }
    Vec3 Rotate(const Vec3& v) const {
        Vec3 q{x, y, z};
        Vec3 t = 2.0f * Cross(q, v);
        return v + w * t + Cross(q, t);
    }
    Quat Normalized() const {
        float l = std::sqrt(x * x + y * y + z * z + w * w);
        return l > 1e-12f ? Quat{x / l, y / l, z / l, w / l} : Quat{};
    }
    /// Rotation vector (axis * angle, angle in [-pi, pi]).
    Vec3 ToRotationVector() const {
        Quat q = Normalized();
        if (q.w < 0) q = {-q.x, -q.y, -q.z, -q.w};
        float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (s < 1e-8f) return {q.x * 2.0f, q.y * 2.0f, q.z * 2.0f};
        float angle = 2.0f * std::atan2(s, q.w);
        return Vec3{q.x, q.y, q.z} * (angle / s);
    }
};
inline Quat Slerp(const Quat& a, Quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0) { b = {-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f) {
        return Quat{Lerp(a.x, b.x, t), Lerp(a.y, b.y, t), Lerp(a.z, b.z, t), Lerp(a.w, b.w, t)}.Normalized();
    }
    float th = std::acos(Clamp(d, -1.0f, 1.0f));
    float s = std::sin(th);
    float wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return Quat{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb}.Normalized();
}
/// Shortest-arc rotation taking direction `from` to direction `to`.
inline Quat FromTo(const Vec3& from, const Vec3& to) {
    Vec3 a = Normalize(from), b = Normalize(to);
    float d = Dot(a, b);
    if (d < -0.99999f) {
        Vec3 axis = Cross(Vec3{1, 0, 0}, a);
        if (LengthSq(axis) < 1e-6f) axis = Cross(Vec3{0, 0, 1}, a);
        return Quat::FromAxisAngle(axis, kPi);
    }
    Vec3 c = Cross(a, b);
    return Quat{c.x, c.y, c.z, 1.0f + d}.Normalized();
}

struct Transform {
    Vec3 position;
    Quat rotation;
    Vec3 TransformPoint(const Vec3& p) const { return position + rotation.Rotate(p); }
    Vec3 InverseTransformPoint(const Vec3& p) const { return rotation.Conjugate().Rotate(p - position); }
};

} // namespace reflex
