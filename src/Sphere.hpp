#pragma once

#ifndef SPHERE_HPP
#define SPHERE_HPP

#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#include <iostream>

#include "MVec.hpp"

class Sphere : public sf::Drawable
{
public:
    Sphere(const Vec3& _position, float _radius, int _density = 1, bool _drawWireframe = false) {
        position = _position;
        radius = _radius;
        this->density = _density;
        this->drawWireframe = _drawWireframe;
        buildGeodesicMesh();
    }
    virtual ~Sphere() {
        deleteDisplayLists();
    };

    void draw(sf::RenderTarget& target, sf::RenderStates states) const override{
        (void)target;
        (void)states;

        //std::cout << "Drawing sphere at position: " << position << " with radius: " << radius << std::endl;

        if (!drawFilled && !drawWireframe)
            return;

        if (listsDirty)
            prepareDraw();

        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);

        if (drawFilled && fillList != 0)
            glCallList(fillList);

        if (drawWireframe && wireFrameList != 0)
        {
            glDisable(GL_LIGHTING);
            glCallList(wireFrameList);
            glEnable(GL_LIGHTING);
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glPopMatrix();
    }

    void toggleWireframe() {
        drawWireframe = !drawWireframe;
    }

    void toggleFilled() {
        drawFilled = !drawFilled;
    }

    void prepareDraw() const
    {
        deleteDisplayLists();

        fillList = glGenLists(1);
        if (fillList != 0)
        {
            glNewList(fillList, GL_COMPILE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glBegin(GL_TRIANGLES);
            std::cout << "Preparing draw: " << vertices.size() << " vertices, " << indices.size() << " faces." << std::endl;
            for (const auto& face : indices)
            {
                const Vec3& n0 = vertices[face[0]];
                const Vec3& n1 = vertices[face[1]];
                const Vec3& n2 = vertices[face[2]];

                const Vec3 p0 = displacedPoint(n0);
                const Vec3 p1 = displacedPoint(n1);
                const Vec3 p2 = displacedPoint(n2);

                glNormal3f(n0.x, n0.y, n0.z);
                const Vec3 c0 = fillColorForVertex(n0);
                glColor3f(c0.x, c0.y, c0.z);
                glVertex3f(p0.x, p0.y, p0.z);

                glNormal3f(n1.x, n1.y, n1.z);
                const Vec3 c1 = fillColorForVertex(n1);
                glColor3f(c1.x, c1.y, c1.z);
                glVertex3f(p1.x, p1.y, p1.z);

                glNormal3f(n2.x, n2.y, n2.z);
                const Vec3 c2 = fillColorForVertex(n2);
                glColor3f(c2.x, c2.y, c2.z);
                glVertex3f(p2.x, p2.y, p2.z);
            }
            glEnd();
            glEndList();
        }

        wireFrameList = glGenLists(1);
        if (wireFrameList != 0)
        {
            glNewList(wireFrameList, GL_COMPILE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glLineWidth(1.2f);
            glColor3f(wireframeColor.x, wireframeColor.y, wireframeColor.z);
            glBegin(GL_TRIANGLES);
            for (const auto& face : indices)
            {
                const Vec3& n0 = vertices[face[0]];
                const Vec3& n1 = vertices[face[1]];
                const Vec3& n2 = vertices[face[2]];

                const Vec3 p0 = displacedPoint(n0);
                const Vec3 p1 = displacedPoint(n1);
                const Vec3 p2 = displacedPoint(n2);

                glVertex3f(p0.x, p0.y, p0.z);
                glVertex3f(p1.x, p1.y, p1.z);
                glVertex3f(p2.x, p2.y, p2.z);
            }
            glEnd();
            glEndList();
        }

        std::cout << "Display lists prepared: fillList=" << fillList << ", wireFrameList=" << wireFrameList << std::endl;

        listsDirty = false;
    }

protected:
    virtual Vec3 fillColorForVertex(const Vec3& normal) const
    {
        (void)normal;
        return fillColor;
    }

    virtual float radiusOffsetForVertex(const Vec3& normal) const
    {
        (void)normal;
        return 0.0f;
    }

    Vec3 displacedPoint(const Vec3& normal) const
    {
        const float r = radius + radiusOffsetForVertex(normal);
        return normal * r;
    }

    static Vec3 normalize(const Vec3& v)
    {
        const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len <= 0.0f)
            return Vec3::nullV;

        return Vec3(v.x / len, v.y / len, v.z / len);
    }

private:
    void invalidateDrawCache()
    {
        listsDirty = true;
    }

    void deleteDisplayLists() const
    {
        if (fillList != 0)
        {
            glDeleteLists(fillList, 1);
            fillList = 0;
        }

        if (wireFrameList != 0)
        {
            glDeleteLists(wireFrameList, 1);
            wireFrameList = 0;
        }
    }

    unsigned int midpointIndex(unsigned int i0, unsigned int i1, std::unordered_map<unsigned long long, unsigned int>& cache)
    {
        const unsigned int a = std::min(i0, i1);
        const unsigned int b = std::max(i0, i1);
        const unsigned long long key = (static_cast<unsigned long long>(a) << 32) | static_cast<unsigned long long>(b);

        const auto found = cache.find(key);
        if (found != cache.end())
            return found->second;

        const Vec3 mid = normalize((vertices[a] + vertices[b]) * 0.5f);
        vertices.push_back(mid);
        const unsigned int idx = static_cast<unsigned int>(vertices.size() - 1);
        cache.emplace(key, idx);
        return idx;
    }

    void buildGeodesicMesh()
    {
        vertices.clear();
        indices.clear();

        const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;

        vertices = {
            normalize(Vec3(-1.0f,  t,  0.0f)),
            normalize(Vec3( 1.0f,  t,  0.0f)),
            normalize(Vec3(-1.0f, -t,  0.0f)),
            normalize(Vec3( 1.0f, -t,  0.0f)),
            normalize(Vec3( 0.0f, -1.0f,  t)),
            normalize(Vec3( 0.0f,  1.0f,  t)),
            normalize(Vec3( 0.0f, -1.0f, -t)),
            normalize(Vec3( 0.0f,  1.0f, -t)),
            normalize(Vec3( t,  0.0f, -1.0f)),
            normalize(Vec3( t,  0.0f,  1.0f)),
            normalize(Vec3(-t,  0.0f, -1.0f)),
            normalize(Vec3(-t,  0.0f,  1.0f))
        };

        indices = {
            {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
            {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
            {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
            {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
        };

        const int subdivisions = std::max(0, density);
        for (int s = 0; s < subdivisions; ++s)
        {
            std::vector<std::array<unsigned int, 3>> next;
            next.reserve(indices.size() * 4);
            std::unordered_map<unsigned long long, unsigned int> cache;
            cache.reserve(indices.size() * 3);

            for (const auto& tri : indices)
            {
                const unsigned int a = midpointIndex(tri[0], tri[1], cache);
                const unsigned int b = midpointIndex(tri[1], tri[2], cache);
                const unsigned int c = midpointIndex(tri[2], tri[0], cache);

                next.push_back({tri[0], a, c});
                next.push_back({tri[1], b, a});
                next.push_back({tri[2], c, b});
                next.push_back({a, b, c});
            }

            indices.swap(next);
        }

        invalidateDrawCache();
    }

    Vec3 position;
    float radius;
    int density;
    std::vector<Vec3> vertices;
    std::vector<std::array<unsigned int, 3>> indices;
    bool drawWireframe = false;
    bool drawFilled = true;
    Vec3 fillColor = Vec3(0.35f, 0.35f, 0.35f);
    Vec3 wireframeColor = Vec3(0.95f, 0.2f, 0.2f);

    mutable GLuint fillList = 0;
    mutable GLuint wireFrameList = 0;
    mutable bool listsDirty = true;
};

#endif // SPHERE_HPP
