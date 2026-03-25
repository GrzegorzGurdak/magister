#include <iostream>
#include <SFML/Graphics.hpp>
#include <optional>
#include <SFML/OpenGL.hpp>

#include "OpenGLGraphics.hpp"
#include "MVec.hpp"
#include "Sphere.hpp"

int main(int argc, char** argv)
{
    Vec2 a(1, 2);
    std::cout << "SFML 3 + OpenGL demo" << std::endl;
    std::cout << "Vec2 a: " << a << std::endl;

    OpenGLGraphics oglGraphics(700, 1.5f, 0);

    sf::RenderWindow window(sf::VideoMode({800, 600}), "SFML 3 + CMake");
    window.setFramerateLimit(60);

    if (!window.setActive(true))
    {
        std::cerr << "Failed to activate OpenGL context" << std::endl;
        return 1;
    }
	//sf::RenderWindow window(sf::VideoMode(770, 730), "Orbiting", sf::Style::Titlebar | sf::Style::Close, sf::ContextSettings(24, 8, 8, 4, 5)); //730
	oglGraphics.reshapeScreen(window.getSize());
    oglGraphics.initOpenGL(argc, argv);

    // glClearDepth(1.f);
    // glClearColor(0.08f, 0.09f, 0.12f, 1.f);
    // glEnable(GL_DEPTH_TEST);
    // glDepthMask(GL_TRUE);

    // enableLight();

    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // const float ratio = 800.f / 600.f;
    // glFrustum(-ratio, ratio, -1.f, 1.f, 1.f, 500.f);

    float angle = 0.f;

    Sphere ball(Vec3(0.f, 0.f, 0.f), 50.f, 1);

    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
            //if pressed L key switch light
            else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->scancode == sf::Keyboard::Scancode::L)
                oglGraphics.switchLight();
        }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.f, 0.f, -220.f);
        glRotatef(angle, 0.f, 1.f, 0.f);

        // glBegin(GL_TRIANGLES);
        // glColor3f(1.f, 0.2f, 0.25f);
        // glVertex3f(-70.f, -60.f, 0.f);

        // glColor3f(0.2f, 1.f, 0.4f);
        // glVertex3f(70.f, -60.f, 0.f);

        // glColor3f(0.3f, 0.5f, 1.f);
        // glVertex3f(0.f, 70.f, 0.f);
        // glEnd();

        window.draw(ball);

        angle += 0.8f;
        if (angle >= 360.f)
            angle -= 360.f;

        window.display();
    }

    return 0;
}
