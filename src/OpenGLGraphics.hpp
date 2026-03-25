#ifndef OPENGLGRAPHICS_H
#define OPENGLGRAPHICS_H
// #pragma once

#include <SFML/Graphics.hpp>

class OpenGLGraphics {
public:
    OpenGLGraphics(): R{700}, theta{1.5f}, phi{0} {};
    OpenGLGraphics(float R, float theta, float phi): R{R}, theta{theta}, phi{phi} {}
    ~OpenGLGraphics() = default;
    // void draw();

    void initOpenGL(int argc, char** argv);

    void enableLight();
    void disableLight();
    void switchLight();

    void setCamera();

    void reshapeScreen(sf::Vector2u size);

    void drawScreen(sf::Vector2u size);

    void drawAxes();

    float R=700;
    float theta = 1.5f;
    float phi;
};

#endif // OPENGLGRAPHICS_H
