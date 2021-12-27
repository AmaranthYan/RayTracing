#pragma once

#include "../math/vec3.h"

struct onb
{
	onb() : axis{} {}
	onb(const vec3& n)
	{
		w = vec3::unit(n);
		vec3 a = fabs(w.x) > 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
		v = vec3::unit(vec3::cross(w, a));
		u = vec3::cross(v, w);	
	}

	__forceinline vec3 operator[](int i) const { return axis[i]; }
	__forceinline vec3& operator[](int i) { return axis[i]; }

	vec3 local(double a, double b, double c) const
	{
		return a * u + b * v + c * w;
	}

	vec3 local(const vec3& a) const
	{
		return a.x * u + a.y * v + a.z * w;
	}

	union
	{
		struct 
		{
			vec3 u;
			vec3 v;
			vec3 w;
		};
		vec3 axis[3];
	};
};