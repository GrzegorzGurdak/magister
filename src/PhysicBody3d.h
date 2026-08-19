#pragma once

#include "MVec.hpp"

struct PhysicBody3d
{
	PhysicBody3d(Vec3 cp, float r, sf::Color  cl):
		radius{ r }, current_position{ cp }, old_position{ cp }
	{
		color = cl;
	}

	PhysicBody3d(Vec3 cp = {}, float r = 20.f) :
		radius{ r }, current_position{ cp }, old_position{ cp }
	{
		color = sf::Color(rand() % 256, rand() % 256, rand() % 256);
	}

	void update_position(const float dtime) {
		if (isKinematic) {
			const Vec3 velocity = (current_position - old_position) * 0.999; // velocity [/dtime]
			old_position = current_position;
			current_position += velocity + acceleration * dtime * dtime; //verlet
		}acceleration = 0;
	}

	float getRadius() const { return radius; }
	void setPos(const Vec3 p) { current_position = p; }
	Vec3 getOldPos() const { return old_position; }
	Vec3 getAcceleration() const { return acceleration; }
	Vec3 getFigurePos() const { return current_position - Vec3{ radius, radius, radius }; }
	sf::Color getFigureColor() const { return color; }

	Vec3 getPos() const { return current_position; }

	void accelerate(const Vec3 add_acc) { acceleration += add_acc; }
	void move(Vec3 p) { old_position = current_position = p; }

	bool isHere(Vec3 here) {return (current_position - here).length() < radius; }
	bool isKinematic = true;

	// Index of the planet-surface triangle that last resolved this body's
	// collision (see PhysicSolver3d::update_planet_collision_heightfield), or -1.
	// A resting body sits on the same patch of ground for many consecutive
	// substeps, so trying this one triangle first before falling back to the
	// full bucket search turns most substeps into an O(1) check instead of a
	// ~20-candidate one.
	int restingTriangle = -1;
	// How many consecutive substeps restingTriangle has been reused without a
	// full search. Capped in update_planet_collision_heightfield() - without a
	// cap, a body resting near a shared edge between two differently-angled
	// triangles can stay locked onto one of them while its true contact point
	// drifts toward that edge, picking up a small but consistent tangential
	// bias each substep. Verlet turns a repeated identical push into a
	// persistent velocity, so that bias compounds into fast sliding.
	// Forcing a periodic re-search re-anchors to the true nearest triangle
	// before that can build up.
	int restingTriangleAge = 0;

	static PhysicBody3d nullPB;

	float radius;
	Vec3 current_position;
	Vec3 old_position;
	Vec3 acceleration = {};
	sf::Color color;
	// sf::CircleShape cs;
	//accelerate
};

