#pragma once

#include "math/vec3.h"

class ray
{
public:
	ray() : origin(), direction(), time() {}
	ray(const vec3& o, const vec3& d, double t = 0.0) : origin(o), direction(d), time(t) {}
	
	static constexpr double epsilon = 0.0001;

	__forceinline vec3 ori() const { return origin; }
	__forceinline vec3 dir() const { return direction; }
	__forceinline double tm() const { return time; }
	__forceinline vec3 pos(double t) const { return origin + t * direction; }

private:
	vec3 origin;
	vec3 direction;
	double time;
};