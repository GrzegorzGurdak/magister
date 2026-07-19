#pragma once

#include "PhysicBody3d.h"

class PhysicLink3d
{
public:
	PhysicLink3d(PhysicBody3d& pb1, PhysicBody3d& pb2, float ll) : physicBody1{ pb1 }, physicBody2{ pb2 }, link_length{ ll } {
		//line[0].position = physicBody1.current_position;
		//line[1].position = physicBody2.current_position;
	}
	void update_link() {
		Vec3 diff = physicBody2.getPos() - physicBody1.getPos();
		float diff_l = diff.length();
		position[0] = physicBody1.current_position;
		position[1] = physicBody2.current_position;
		//if (diff_l < 1e-6) diff_l = 1e-6;
		//Vec2 tran = diff / diff_l * (diff_l - link_length) / 2;
		if (physicBody1.isKinematic) physicBody1.current_position += diff / diff_l * (diff_l - link_length) / 2;
		if (physicBody2.isKinematic) physicBody2.current_position -= diff / diff_l * (diff_l - link_length) / 2;
	}

	const Vec3& getLinkBegin() const { return position[0]; }
	const Vec3& getLinkEnd() const { return position[1]; }

private:
	PhysicBody3d& physicBody1;
	PhysicBody3d& physicBody2;

	Vec3 position[2];
	// sf::VertexArray line{ sf::Lines, 2 }; //TODO: fix this

	float link_length;
};

