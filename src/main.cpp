#include <iostream>
#include <SFML/Graphics.hpp>
#include <optional>
#include <SFML/OpenGL.hpp>

#include "OpenGLGraphics.hpp"
#include "MVec.hpp"
#include "planet.hpp"

int main(int argc, char** argv)
{
    Vec2 a(1, 2);
    std::cout << "SFML 3 + OpenGL demo" << std::endl;
    std::cout << "Vec2 a: " << a << std::endl;

    OpenGLGraphics oglGraphics(900, 1.5f, 0);

    sf::RenderWindow window(sf::VideoMode({1200, 900}), "SFML 3 + CMake");
    window.setFramerateLimit(30);

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

    Planet ball(Vec3(0.f, 0.f, 0.f), 30.f, 8, true);

    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
            else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->scancode == sf::Keyboard::Scancode::L)
                    oglGraphics.switchLight();
                else if (keyPressed->scancode == sf::Keyboard::Scancode::W)
                    ball.toggleWireframe();
                else if (keyPressed->scancode == sf::Keyboard::Scancode::F)
                    ball.toggleFilled();
            }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // glMatrixMode(GL_MODELVIEW);
        // glLoadIdentity();
        // glTranslatef(0.f, 0.f, -220.f);
        // glRotatef(angle*50.f, 0.f, 1.f, 0.f);

        oglGraphics.updateCamera(120, angle,  0.1f);

        // glBegin(GL_TRIANGLES);
        // glColor3f(1.f, 0.2f, 0.25f);
        // glVertex3f(-70.f, -60.f, 0.f);

        // glColor3f(0.2f, 1.f, 0.4f);
        // glVertex3f(70.f, -60.f, 0.f);

        // glColor3f(0.3f, 0.5f, 1.f);
        // glVertex3f(0.f, 70.f, 0.f);
        // glEnd();

        window.draw(ball);

        angle += 0.002f;
        if (angle >= M_PIF * 2.f)
            angle -= M_PIF * 2.f;

        window.display();
    }

    return 0;
}
