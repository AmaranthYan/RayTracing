#pragma once

#define _USE_MATH_DEFINES

#include <math.h>
#include "ray.h"

__forceinline double degrees_to_radians(double deg) {
	return deg * M_PI / 180.0;
}

class camera
{
public:
	camera(vec3 ori, vec3 lookat, vec3 vup, double vfov, double aspect, double aperture, double focus) : origin(ori)
	{
		auto theta = degrees_to_radians(vfov);
		auto h = tan(theta / 2) * focus;

		auto viewport_height = 2.0 * h;
		auto viewport_width = aspect * viewport_height;

		w = vec3::unit(origin - lookat);
		u = vec3::unit(vec3::cross(vup, w));
		v = vec3::cross(w, u);

		horizontal = viewport_width  * u;
		vertical = viewport_height * v;
		lower_left = -horizontal / 2 - vertical / 2 - focus * w;

		lens_radius = aperture / 2;
	}

	ray get_ray(double s, double t, double t0, double t1) const
	{
		vec3 rd = lens_radius * vec3::random_unit_disk();
		vec3 offset = u * rd.x + v * rd.y;
		return ray(origin + offset, lower_left + s * horizontal + t * vertical - offset, drand(t0, t1));
	}

private:
	vec3 origin;
	vec3 horizontal;
	vec3 vertical;
	vec3 lower_left;
	vec3 u, v, w;
	double lens_radius;
};