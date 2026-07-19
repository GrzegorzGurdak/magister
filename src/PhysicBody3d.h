#pragma once

#include "MVec.hpp"

struct PhysicBody3d
{
	PhysicBody3d(Vec3 cp, float r, sf::Color  cl):
		current_position{ cp }, old_position{ cp }, radius{ r }
	{
		color = cl;
	}

	PhysicBody3d(Vec3 cp = {}, float r = { 20 }) :
		current_position{ cp }, old_position{ cp }, radius{ r }
	{
		color = sf::Color(rand() % 256, rand() % 256, rand() % 256);
	}

	void update_position(const float dtime) {
		if (isKinematic) {
			const Vec3 velocity = current_position - old_position; // velocity [/dtime]
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

	static PhysicBody3d nullPB;

	Vec3 current_position;
	float radius;

protected:
	Vec3 old_position;
	Vec3 acceleration = {};
	sf::Color color;
	// sf::CircleShape cs;
	//accelerate
};

