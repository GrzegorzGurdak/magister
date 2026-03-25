#pragma once

#ifndef BALL_HPP
#define BALL_HPP

#include <vector>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#include "MVec.hpp"

class Sphere : public sf::Drawable
{
public:
    Sphere(const Vec3& _position, float _radius, int density = 1) {
        vertices.reserve(density * 4);
        position = _position;
        radius = _radius;
        density = density;

        // vertices.push_back(Vec3(0.f, 0.f, radius));
        // vertices.push_back(Vec3(, 0.f, -radius/3));
        // for (int i = 0; i < 4; i++) {
        //     float phi = M_PIF * (i + 1) / (density + 1);
        //     for (int j = 0; j < density * 2; j++) {
        //         float theta = M_PIF * j / density;
        //         vertices.push_back(Vec3(sinf(theta) * cosf(phi), sinf(phi), cosf(theta) * cosf(phi)) * radius);
        //     }
        // }
        vertices.push_back(Vec3(1.f, 1.f, 1.f) * radius);
        vertices.push_back(Vec3(-1.f, -1.f, 1.f) * radius);
        vertices.push_back(Vec3(-1.f, 1.f, -1.f) * radius);
        vertices.push_back(Vec3(1.f, -1.f, -1.f) * radius);
    }
    ~Sphere() {};

    void draw(sf::RenderTarget& target, sf::RenderStates states) const override{
        glBegin(GL_TRIANGLE_STRIP);
        float i = 0.f;
        for (const Vec3& v : vertices) {
            glColor3f(.2f, i, .2f); i+= 0.2f;
            glVertex3f(position.x + v.x, position.y + v.y, position.z + v.z);
        }
        glVertex3f(position.x + vertices[0].x, position.y + vertices[0].y, position.z + vertices[0].z);
        glEnd();
    }

private:
    Vec3 position;
    float radius;
    std::vector<Vec3> vertices;
    int density;
};

#endif // BALL_HPP