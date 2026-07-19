#include "OpenGLGraphics.hpp"
#include <SFML/OpenGL.hpp>
#include <gl/glu.h>
#include <GL/glut.h>  // Uncomment when GLUT is available

void OpenGLGraphics::initOpenGL(int argc, char** argv) {
    glClearColor(1.f, 1.f, 1.f, 0.f);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glutInit(&argc, argv);  // Uncomment when GLUT is available
    setCamera();
    enableLight();
}

void OpenGLGraphics::enableLight() {
    glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
}

void OpenGLGraphics::disableLight() {
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
}

void OpenGLGraphics::switchLight() {
    if(glIsEnabled(GL_LIGHTING)){
        disableLight();
    } else {
        enableLight();
    }
}

void OpenGLGraphics::setCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        centerX + R * cosf(theta) * cosf(phi), centerY + R * sinf(phi), centerZ + R * sinf(theta) * cosf(phi),
        centerX, centerY, centerZ,
        0,   R * cosf(phi),   0
    );
    // gluLookAt(
    //     eyex, eyey, eyez,
    //     centerx, centery, centerz,
    //     upx, upy, upz
    // );
}

void OpenGLGraphics::reshapeScreen(sf::Vector2u size) {
    glViewport(0, 0, (GLsizei)size.x, (GLsizei)size.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLdouble)size.x / (GLdouble)size.y, 0.1, 2000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void OpenGLGraphics::drawScreen(sf::Vector2u size) {
    glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void OpenGLGraphics::drawAxes() {
    glLineWidth((GLfloat)3);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(800, 0, 0);
        glVertex3f(0, 0, 0);

        glColor3f(0, 1, 0);
        glVertex3f(0, 700, 0);
        glVertex3f(0, 0, 0);

        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 1);
        glVertex3f(0, 0, 0);
    glEnd();

    glLineStipple(3, 0xAAAA);
    glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(-1, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, -1, 0);
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, -1);
    glEnd();
	glDisable(GL_LINE_STIPPLE);
}
