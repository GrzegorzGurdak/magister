#pragma once

#include <SFML/Graphics.hpp>

#include <cmath>

#ifndef M_PIF
# define M_PIF	3.14159265358979323846f
#endif


struct Vec2
{
    Vec2(float _x = 0, float _y = 0) : x{ _x }, y{ _y } {};
    Vec2(int _x, int _y) : x{ (float)_x}, y{ (float)_y} {};
    //Vec2(int _x, int _y) : x{(float)_x}, y{(float)_y} {};
    Vec2(sf::Vector2f _v) : x{ _v.x }, y{ _v.y } {};
    Vec2(sf::Vector2u _v) : x{ (float)_v.x}, y{ (float)_v.y} {};
    float x;
    float y;

    inline Vec2& set(float _x, float _y) { x = _x; y = _y; return *this; }
    inline Vec2& set(int _x, int _y) { x = float(_x); y = float(_y); return *this; }

    Vec2 operator+ (const Vec2 other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator- (const Vec2 other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator+= (const Vec2 other) { x += other.x; y += other.y; return *this; }
    Vec2 operator-= (const Vec2 other) { x -= other.x; y -= other.y; return *this; }
    Vec2 operator* (const Vec2 other) const { return Vec2(x * other.x, y * other.y); }
    Vec2 operator*= (const Vec2 other) { x *= other.x; y *= other.y; return *this; }

    bool operator== (const Vec2 other) const { return x == other.x && y == other.y; }
    bool operator!= (const Vec2 other) const { return x != other.x || y != other.y; }

    Vec2 operator* (const float _m) const { return Vec2(x * _m, y * _m); }
    Vec2 operator/ (const float _m) const { return Vec2(x / _m, y / _m); }

    operator sf::Vector2f() const { return sf::Vector2f(x, y); }

    float length() const { return sqrt(x*x + y*y); };

    static const Vec2 nullV;
    static const Vec2 UP;
    static const Vec2 DOWN;
    static const Vec2 LEFT;
    static const Vec2 RIGHT;

    static Vec2 random_rad(float max_radius) {
        float angle = rand() * 2.f * M_PIF / RAND_MAX;
        return Vec2(sinf(angle), cosf(angle)) * (max_radius * rand() / RAND_MAX);
    }

    static float angleBetween(const Vec2 base, const Vec2 target);
};

std::ostream& operator<< (std::ostream& out, const Vec2& obj);

std::ostream& operator<< (std::ostream& out, const Vec2&& obj);

struct Vec3
{
    Vec3(float _x = 0, float _y = 0, float _z = 0) : x{ _x }, y{ _y }, z{ _z } {};
    Vec3(int _x, int _y, int _z = 0) : x{ (float)_x }, y{ (float)_y }, z{ (float)_z } {};
    //Vec2(int _x, int _y) : x{(float)_x}, y{(float)_y} {};
    Vec3(sf::Vector3f _v) : x{ _v.x }, y{ _v.y }, z{ _v.z } {};
    Vec3(sf::Vector3i _v) : x{ (float)_v.x}, y{ (float)_v.y}, z{ (float)_v.z} {};
    float x;
    float y;
    float z;

    inline Vec3& set(float _x, float _y, float _z) { x = _x; y = _y; z = _z; return *this; }
    inline Vec3& set(int _x, int _y, int _z) { x = (float)_x; y = (float)_y; z = (float)_z; return *this; }

    Vec3 operator+ (const Vec3 other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
    Vec3 operator- (const Vec3 other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
    Vec3 operator+= (const Vec3 other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vec3 operator-= (const Vec3 other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
    Vec3 operator* (const Vec3 other) const { return Vec3(x * other.x, y * other.y, z * other.z); }
    Vec3 operator*= (const Vec3 other) { x *= other.x; y *= other.y; z *= other.z; return *this; }

    bool operator== (const Vec3 other) const { return x == other.x && y == other.y && z == other.z; }
    bool operator!= (const Vec3 other) const { return x != other.x || y != other.y || z != other.z; }

    Vec3 operator* (const float _m) const { return Vec3(x * _m, y * _m, z * _m); }
    Vec3 operator/ (const float _m) const { return Vec3(x / _m, y / _m, z / _m); }

    operator sf::Vector3f() const { return sf::Vector3f(x, y, z);  }

    float length() const { return sqrt(x*x + y*y + z*z); };

    Vec3 normal() const {
        float len = length();
        if (len == 0.f) return Vec3(0.f, 0.f, 0.f);
        return Vec3(x / len, y / len, z / len);
    }

    static const Vec3 nullV;
    static const Vec3 UP;
    static const Vec3 DOWN;
    static const Vec3 LEFT;
    static const Vec3 RIGHT;
    static const Vec3 FORWARD;
    static const Vec3 BACK;

    static Vec3 random_rad(float min_radius, float max_radius) {
        float yaw = rand() * 2.f * M_PIF / RAND_MAX;
        float pitch = rand() * 2.f * M_PIF / RAND_MAX;
        float radius = min_radius + (max_radius - min_radius) * rand() / RAND_MAX;
        return Vec3(sinf(yaw) * cosf(pitch), sinf(pitch), cosf(yaw) * cosf(pitch)) * radius;
    }

    static float angleBetween(const Vec3 base, const Vec3 target);
};

std::ostream& operator<< (std::ostream& out, const Vec3& obj);

std::ostream& operator<< (std::ostream& out, const Vec3&& obj);
