#include <cmath>
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

    OpenGLGraphics oglGraphics(400.f, 1.5f, 0.f);

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

    bool orbiting = false;
    sf::Vector2i lastMousePos;
    constexpr float orbitSensitivity = 0.005f;
    // Kept just under +-pi/2: at exactly the pole the up vector
    // (0, R*cos(phi), 0) in OpenGLGraphics::setCamera degenerates to zero,
    // which flips the view.
    constexpr float phiLimit = 1.5f;
    constexpr float zoomSensitivity = 20.f;
    constexpr float minZoom = 60.f; // stay outside the planet (radius 50)
    constexpr float maxZoom = 1000.f;
    constexpr float panSpeed = 80.f; // world units per second
    sf::Clock cameraClock;

    Planet ball(Vec3(0.f, 0.f, 0.f), 50.f, 5, true);
    // Second planet, smaller and offset along X. Distance (150) clears the sum
    // of both planets' max terrain-displaced radii (50+16 and 25+16, i.e. 66+41)
    // with margin, and its own particle shell (radius 30-40, spawned below) tops
    // out at magnitude 190 - inside both the 195 containment sphere and the
    // +-200 chunk grid bounds (see PhysicSolver3d::update_constraints).
    Planet ball2(Vec3(150.f, 0.f, 0.f), 25.f, 4, true);

    sf::Font font;
	if (!font.openFromFile("fonts/arial.ttf")) {
		std::cerr << "Failed to load fonts/arial.ttf" << std::endl;
		return 1;
	}

    StatElement statElement(font);

    PhysicSolver3d sandbox(ChunkGrid3d(6.f, 2.f, Vec3(-200.f, -200.f, -200.f), Vec3(200.f, 200.f, 200.f)), &ball);
	PhysicDrawer3d sandbox_draw(sandbox);
    sandbox.addPlanet(&ball2);

    // acceleration function: inverse-square gravity toward each planet's own
    // center, summed. Captured by value since planet positions are fixed.
    const Vec3 ball1Pos = ball.getPosition();
    const Vec3 ball2Pos = ball2.getPosition();
    sandbox.set_acceleration([ball1Pos, ball2Pos](PhysicBody3d* obj, std::vector<PhysicBody3d*>& objs) -> Vec3 {
        (void)objs;
        Vec3 total{};
        for (const Vec3& center : { ball1Pos, ball2Pos }) {
            Vec3 dir = center - obj->getPos();
            float dist = dir.length()/10;
            if (dist < 1.f) dist = 1.f;
            total += dir.normal() * (100.f / (dist * dist)); //inverse square law
        }
        return total;
    });
    // sandbox.set_acceleration(Vec3(0,10,0));
    sandbox.set_constraints_def();

    //PhysicBody3d* controlObj;

    for(int i = 0; i < 5000; i++)
    {
        PhysicBody3d* obj = new PhysicBody3d(
            Vec3::random_rad(90.f, 100.f),
            2.f,
            sf::Color(rand() % 256, rand() % 256, rand() % 256)
            //ocean blue sf::Color(0, 105, 148)
            //sf::Color(11, 57, 135)
        );
        //move slightly towards the center to avoid stalling
        obj->setPos(obj->getPos() + (obj->getPos().normal() * -0.001f));
        sandbox.add(obj);
        // controlObj = obj; //last added object will be for debug
    }

    for(int i = 0; i < 2000; i++)
    {
        PhysicBody3d* obj = new PhysicBody3d(
            ball2Pos + Vec3::random_rad(30.f, 40.f),
            2.f,
            sf::Color(rand() % 256, rand() % 256, rand() % 256)
        );
        //move slightly towards ball2's center to avoid stalling
        obj->setPos(obj->getPos() + ((obj->getPos() - ball2Pos).normal() * -0.001f));
        sandbox.add(obj);
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
                else if (keyPressed->scancode == sf::Keyboard::Scancode::T)
                {
                    ball.toggleWireframe();
                    ball2.toggleWireframe();
                }
                else if (keyPressed->scancode == sf::Keyboard::Scancode::F)
                {
                    ball.toggleFilled();
                    ball2.toggleFilled();
                }
            }
            else if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                if (mousePressed->button == sf::Mouse::Button::Right)
                {
                    orbiting = true;
                    lastMousePos = mousePressed->position;
                }
            }
            else if (const auto* mouseReleased = event->getIf<sf::Event::MouseButtonReleased>())
            {
                if (mouseReleased->button == sf::Mouse::Button::Right)
                    orbiting = false;
            }
            else if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>())
            {
                if (orbiting)
                {
                    const sf::Vector2i delta = mouseMoved->position - lastMousePos;
                    lastMousePos = mouseMoved->position;

                    oglGraphics.theta += delta.x * orbitSensitivity;
                    oglGraphics.phi += delta.y * orbitSensitivity;
                    if (oglGraphics.phi > phiLimit) oglGraphics.phi = phiLimit;
                    if (oglGraphics.phi < -phiLimit) oglGraphics.phi = -phiLimit;
                }
            }
            else if (const auto* wheelScrolled = event->getIf<sf::Event::MouseWheelScrolled>())
            {
                if (wheelScrolled->wheel == sf::Mouse::Wheel::Vertical)
                {
                    oglGraphics.R -= wheelScrolled->delta * zoomSensitivity;
                    if (oglGraphics.R < minZoom) oglGraphics.R = minZoom;
                    if (oglGraphics.R > maxZoom) oglGraphics.R = maxZoom;
                }
            }
        }

        const float dt = cameraClock.restart().asSeconds();
        {
            // Forward/right derived from theta alone (ignoring phi) so WASD
            // panning stays on the horizontal plane regardless of the current
            // orbit elevation - matches the eye direction in
            // OpenGLGraphics::setCamera projected onto the XZ plane.
            const float forwardX = -std::cos(oglGraphics.theta);
            const float forwardZ = -std::sin(oglGraphics.theta);
            const float rightX = std::sin(oglGraphics.theta);
            const float rightZ = -std::cos(oglGraphics.theta);

            float moveX = 0.f, moveZ = 0.f;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::W)) { moveX += forwardX; moveZ += forwardZ; }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::S)) { moveX -= forwardX; moveZ -= forwardZ; }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::D)) { moveX += rightX; moveZ += rightZ; }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::A)) { moveX -= rightX; moveZ -= rightZ; }

            const float moveLenSq = moveX * moveX + moveZ * moveZ;
            if (moveLenSq > 0.f) {
                const float invLen = 1.f / std::sqrt(moveLenSq);
                oglGraphics.centerX += moveX * invLen * panSpeed * dt;
                oglGraphics.centerZ += moveZ * invLen * panSpeed * dt;
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::E)) oglGraphics.centerY += panSpeed * dt;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::Q)) oglGraphics.centerY -= panSpeed * dt;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.1f, 0.1f, 0.1f, 1.f);

        oglGraphics.setCamera();

        sandbox.update(timeResults, 1 / 30.f, 4);
        statElement.update();


        // std::cout << "Time results (us): "
        //     << "acceleration: " << timeResults[0] << ", "
        //     << "constraints: " << timeResults[1] << ", "
        //     << "links: " << timeResults[2] << ", "
        //     << "assign grid: " << timeResults[3] << ", "
        //     << "collision: " << timeResults[4] << ", "
        //     << "position: " << timeResults[5] << ", "
        //     << "planet collision: " << timeResults[6] << "\n";

        // for (int i = 0; i < 7; i++)
        //     timeResults[i] = 0;

        //std::cout << controlObj->getPos() << "\n";

        window.draw(sandbox_draw);
        window.draw(ball);
        window.draw(ball2);

        window.pushGLStates();
		window.draw(statElement);
		window.popGLStates();

        window.display();
    }

    return 0;
}
