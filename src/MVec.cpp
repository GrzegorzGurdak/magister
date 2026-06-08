#include "MVec.hpp"

#include <iostream>

const Vec2 Vec2::nullV = Vec2();
const Vec2 Vec2::UP = Vec2(0,1);
const Vec2 Vec2::DOWN = Vec2(0,-1);
const Vec2 Vec2::LEFT = Vec2(-1,0);
const Vec2 Vec2::RIGHT = Vec2(1,0);

std::ostream& operator<< (std::ostream& out, const Vec2& obj)
{
	return out << "(" << obj.x << ", " << obj.y << ")";
}

std::ostream& operator<< (std::ostream& out, const Vec2&& obj)
{
	return out << "(" << obj.x << ", " << obj.y << ")";
}

float Vec2::angleBetween(Vec2 base, Vec2 target)
{
	float det = (target.x * base.y - target.y * base.x);

	float dot = (target.x * base.x + target.y * base.y);


	//if(det>0 && dot > 0) return atan2f(det, dot) * 180 / M_PI;

	//if(dot>0) return atan2f(det, dot) * 180 / M_PI; // det<0

	//if (det < 0) return 360 + atan2f(det, dot);// *180.f / M_PIF; // det <0 -180

	return atan2f(det, dot);// *180.f / M_PIF;
}


const Vec3 Vec3::nullV = Vec3();
const Vec3 Vec3::UP = Vec3(0,1,0);
const Vec3 Vec3::DOWN = Vec3(0,-1,0);
const Vec3 Vec3::LEFT = Vec3(-1,0,0);
const Vec3 Vec3::RIGHT = Vec3(1,0,0);
const Vec3 Vec3::FORWARD = Vec3(0,0,1);
const Vec3 Vec3::BACK = Vec3(0,0,-1);

std::ostream& operator<< (std::ostream& out, const Vec3& obj)
{
	return out << "(" << obj.x << ", " << obj.y << ", " << obj.z << ")";
}

std::ostream& operator<< (std::ostream& out, const Vec3&& obj)
{
	return out << "(" << obj.x << ", " << obj.y << ", " << obj.z << ")";
}

//function to calculate angle between two 3D vectors
float Vec3::angleBetween(Vec3 base, Vec3 target)
{
	float dot = base.x * target.x + base.y * target.y + base.z * target.z;
	float det = base.x * target.y - base.y * target.x + base.x * target.z - base.z * target.x + base.y * target.z - base.z * target.y;

	return atan2f(det, dot);
}
