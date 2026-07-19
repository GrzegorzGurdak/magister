#pragma once

#include "PhysicBody2d.h"

class PhysicLink2d
{
public:
	PhysicLink2d(PhysicBody2d& pb1, PhysicBody2d& pb2, float ll) : physicBody1{ pb1 }, physicBody2{ pb2 }, link_length{ ll } {
		//line[0].position = physicBody1.current_position;
		//line[1].position = physicBody2.current_position;
	}
	void update_link() {
		Vec2 diff = physicBody2.getPos() - physicBody1.getPos();
		float diff_l = diff.length();
		line[0].position = physicBody1.current_position;
		line[1].position = physicBody2.current_position;
		//if (diff_l < 1e-6) diff_l = 1e-6;
		//Vec2 tran = diff / diff_l * (diff_l - link_length) / 2;
		if (physicBody1.isKinematic) physicBody1.current_position += diff / diff_l * (diff_l - link_length) / 2;
		if (physicBody2.isKinematic) physicBody2.current_position -= diff / diff_l * (diff_l - link_length) / 2;
	}

	const sf::VertexArray& getFigure() const { return line; }

private:
	PhysicBody2d& physicBody1;
	PhysicBody2d& physicBody2;

	sf::VertexArray line{ sf::PrimitiveType::Lines, 2 };

	float link_length;
};

