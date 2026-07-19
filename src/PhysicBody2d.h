#pragma once

#include "MVec.hpp"

struct PhysicBody2d
{
	PhysicBody2d(Vec2 cp, float r, sf::Color  cl):
		current_position{ cp }, old_position{ cp }, radius{ r }, cs{ r }
	{
		cs.setFillColor(cl);
		cs.setPosition(current_position - Vec2{ radius,radius });
	}

	PhysicBody2d(Vec2 cp = {}, float r = { 20 }) :
		current_position{ cp }, old_position{ cp }, radius{ r }, cs{ r }
	{
		cs.setFillColor(sf::Color(rand() % 256, rand() % 256, rand() % 256));
		cs.setPosition(current_position - Vec2{ radius,radius });
	}

	void update_position(const float dtime) {
		if (isKinematic) {
			const Vec2 velocity = current_position - old_position; // velocity [/dtime]
			old_position = current_position;
			current_position += velocity + acceleration * dtime * dtime; //verlet
			cs.setPosition(current_position - Vec2{ radius,radius });
		}acceleration = 0;
	}

	float getRadius() const { return radius; }
	void setPos(const Vec2 p) { current_position = p; }
	Vec2 getOldPos() const { return old_position; }
	Vec2 getAcceleration() const { return acceleration; }
	const sf::CircleShape& getFigure() const { return cs; }

	Vec2 getPos() const { return current_position; }

	void accelerate(const Vec2 add_acc) { acceleration += add_acc; }
	void move(Vec2 p) { old_position = current_position = p; cs.setPosition(p - Vec2{ radius,radius }); }

	bool isHere(Vec2 here) {return (current_position - here).length() < radius; }
	bool isKinematic = true;

	static PhysicBody2d nullPB;

protected:
	float radius;
	Vec2 current_position;
	Vec2 old_position;
	Vec2 acceleration = {};
	sf::CircleShape cs;
	friend class PhysicSolver;
	friend class ChunkGrid;
	friend class PhysicLink2d;
	//accelerate
};

