#pragma once

#ifndef PLANET_HPP
#define PLANET_HPP

#include "Sphere.hpp"

#include <algorithm>
#include <cmath>



class Planet : public Sphere
{
public:
    Planet(const Vec3& _position, float _radius, int _density = 1, bool _drawWireframe = false)
        : Sphere(_position, _radius, _density, _drawWireframe) {}
    ~Planet() {};

protected:
    float radiusOffsetForVertex(const Vec3& normal) const override
    {
        const float h = terrainHeight(normal);
        return h * terrainAmplitude;
    }

    Vec3 fillColorForVertex(const Vec3& normal) const override
    {
        const float h = terrainHeight(normal);

        if (h < oceanLevel - 0.06f) return deepOceanColor;
        if (h < oceanLevel) return shallowOceanColor;
        if (h < oceanLevel + 0.02f) return beachColor;
        if (h < 0.22f) return lowlandColor;
        if (h < 0.45f) return highlandColor;
        return snowColor;
    }

private:
    float terrainHeight(const Vec3& n) const
    {
        float value = 0.0f;
        float amp = 1.0f;
        float freq = baseFrequency;
        float ampSum = 0.0f;

        for (int i = 0; i < octaves; ++i)
        {
            value += amp * waveBand(n, freq, float(i) * 3.731f);
            ampSum += amp;
            amp *= persistence;
            freq *= lacunarity;
        }

        if (ampSum > 0.0f)
            value /= ampSum;

        value = value * 0.5f + 0.5f;

        const float ridge = 1.0f - std::fabs(2.0f * value - 1.0f);
        value = std::clamp(0.75f * value + 0.25f * ridge, 0.0f, 1.0f);

        return value * 2.0f - 1.0f;
    }

    float waveBand(const Vec3& n, float f, float phase) const
    {
        const float a = std::sinf((n.x * 1.13f + n.y * 0.77f + n.z * 1.61f) * f + phase);
        const float b = std::cosf((n.x * -1.71f + n.y * 1.37f + n.z * 0.53f) * (f * 1.23f) - phase * 0.5f);
        const float c = std::sinf((n.x * 0.49f + n.y * -1.93f + n.z * 1.07f) * (f * 0.82f) + phase * 1.2f);
        return (a + b + c) / 3.0f;
    }

    float terrainAmplitude = 8.0f;
    float oceanLevel = -0.08f;

    int octaves = 5;
    float persistence = 0.52f;
    float lacunarity = 2.1f;
    float baseFrequency = 3.2f;

    Vec3 deepOceanColor = Vec3(0.04f, 0.20f, 0.47f);
    Vec3 shallowOceanColor = Vec3(0.10f, 0.42f, 0.64f);
    Vec3 beachColor = Vec3(0.86f, 0.80f, 0.58f);
    Vec3 lowlandColor = Vec3(0.22f, 0.55f, 0.24f);
    Vec3 highlandColor = Vec3(0.40f, 0.40f, 0.33f);
    Vec3 snowColor = Vec3(0.92f, 0.92f, 0.95f);
};

#endif // PLANET_HPP