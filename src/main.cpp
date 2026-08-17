#include <iostream>
#include <SFML/Graphics.hpp>
#include <optional>
#include <SFML/OpenGL.hpp>

#include "OpenGLGraphics.hpp"
#include "GUI_elements.hpp"
#include "MVec.hpp"
#include "planet.hpp"

#include "PhysicSolver3d.hpp"

int main(int argc, char** argv)
{
    Vec2 a(1, 2);
    std::cout << "SFML 3 + OpenGL demo" << std::endl;
    std::cout << "Vec2 a: " << a << std::endl;

    OpenGLGraphics oglGraphics(900, 1.5f, 0);

    sf::RenderWindow window(sf::VideoMode({1200, 900}), "SFML 3 + CMake");
    window.setFramerateLimit(60);

    if (!window.setActive(true))
    {
        std::cerr << "Failed to activate OpenGL context" << std::endl;
        return 1;
    }
	//sf::RenderWindow window(sf::VideoMode(770, 730), "Orbiting", sf::Style::Titlebar | sf::Style::Close, sf::ContextSettings(24, 8, 8, 4, 5)); //730
	oglGraphics.reshapeScreen(window.getSize());
    oglGraphics.initOpenGL(argc, argv);

    float angle = 0.f;

    Planet ball(Vec3(0.f, 0.f, 0.f), 50.f, 5, true);

    sf::Font font;
	if (!font.openFromFile("fonts/arial.ttf")) {
		std::cerr << "Failed to load fonts/arial.ttf" << std::endl;
		return 1;
	}

    StatElement statElement(font);

    PhysicSolver3d sandbox(ChunkGrid3d(20.f, 1.f, Vec3(-200.f, -200.f, -200.f), Vec3(200.f, 200.f, 200.f)), &ball);
	PhysicDrawer3d sandbox_draw(sandbox);

    // acceleration function: to the center of Vec3(0,0,0)
    sandbox.set_acceleration([](PhysicBody3d* obj, std::vector<PhysicBody3d*>& objs) -> Vec3 {
        (void)objs;
        Vec3 center(0.f, 0.f, 0.f);
        Vec3 dir = center - obj->getPos();
        float dist = dir.length()/10;
        if (dist < 1.f) dist = 1.f;
        return dir.normal() * (100.f / (dist * dist)); //inverse square law
    });
    // sandbox.set_acceleration(Vec3(0,10,0));
    sandbox.set_constraints_def();

    //PhysicBody3d* controlObj;

    for(int i = 0; i < 1000; i++)
    {
        PhysicBody3d* obj = new PhysicBody3d(
            Vec3::random_rad(80.f, 100.f),
            1.f,
            // sf::Color(rand() % 256, rand() % 256, rand() % 256)
            //ocean blue sf::Color(0, 105, 148)
            sf::Color(11, 57, 135)
        );
        sandbox.add(obj);
        // controlObj = obj; //last added object will be for debug
    }

    long long timeResults[7] = { 0, 0, 0, 0, 0, 0, 0 };

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

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.f, 0.f, -400.f);
        glRotatef(angle, 0.f, 1.f, 0.f);

        sandbox.update(timeResults, 1 / 30.f, 8);
        statElement.update();

        //std::cout << controlObj->getPos() << "\n";

        window.draw(sandbox_draw);
        window.draw(ball);

        window.pushGLStates();
		window.draw(statElement);
		window.popGLStates();

        angle += 0.005f * 50;
        if (angle >= 360.f)
            angle -= 360.f;

        window.display();
    }

    return 0;
}
