#ifndef OPENGLGRAPHICS_H
#define OPENGLGRAPHICS_H
// #pragma once

#include <SFML/Graphics.hpp>

class OpenGLGraphics {
public:
    OpenGLGraphics(): R{700}, theta{1.5f}, phi{0}, centerX{0}, centerY{0}, centerZ{0} {};
    OpenGLGraphics(float R, float theta, float phi): R{R}, theta{theta}, phi{phi} {}
    ~OpenGLGraphics() = default;
    // void draw();

    void initOpenGL(int argc, char** argv);

    void enableLight();
    void disableLight();
    void switchLight();

    void setCamera();
    void updateCamera(float R, float theta, float phi) {
        this->R = R;
        this->theta = theta;
        this->phi = phi;
        setCamera();
    }
    // void setEye(float eyex, float eyey, float eyez) {
    //     this->eyex = eyex;
    //     this->eyey = eyey;
    //     this->eyez = eyez;
    // }

    void setCenter(float centerx, float centery, float centerz) {
        this->centerX = centerx;
        this->centerY = centery;
        this->centerZ = centerz;
    }

    // void setUp(float upx, float upy, float upz) {
    //     this->upx = upx;
    //     this->upy = upy;
    //     this->upz = upz;
    // }

    void reshapeScreen(sf::Vector2u size);

    void drawScreen(sf::Vector2u size);

    void drawAxes();

    float R=700;
    float theta = 1.5f;
    float phi;

    // float eyex, eyey, eyez;
    float centerX, centerY, centerZ;
    // float upx, upy, upz;
};

#endif // OPENGLGRAPHICS_H
