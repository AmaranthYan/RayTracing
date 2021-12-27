#pragma once

#include "../math/vec3.h"
#include "../ray.h"

struct aabb
{
	aabb() : minimum(), maximum() {}
	aabb(const vec3& min, const vec3& max) : minimum(min), maximum(max) {}

	__forceinline bool intersect_ray(const ray& r, double t_min, double t_max) const;
	static __forceinline aabb surrounding_box(const aabb& a, const aabb& b);

	__forceinline double half_area() const
	{
		auto d = maximum - minimum;
		return d.x * (d.y + d.z) + d.y * d.z;
	}

	vec3 minimum;
	vec3 maximum;
};

__forceinline bool aabb::intersect_ray(const ray& r, double t_min, double t_max) const
{
	const auto inv_dir = 1.0 / r.dir();
	auto t0 = (this->minimum - r.ori()) * inv_dir;
	auto t1 = (this->maximum - r.ori()) * inv_dir;

	if (inv_dir.x < 0.0)
	{
		std::swap(t0.x, t1.x);
	}
	if (inv_dir.y < 0.0)
	{
		std::swap(t0.y, t1.y);
	}
	if (inv_dir.z < 0.0)
	{
		std::swap(t0.z, t1.z);
	}

	// the order is important, need to eliminate any NaN value, always put t_min/t_max on the left side
	t_min = std::max(std::max(std::max(t_min, t0.x), t0.y), t0.z);
	t_max = std::min(std::min(std::min(t_max, t1.x), t1.y), t1.z);

	// don't use <= for potential infinity
	return t_min < t_max;
}

__forceinline aabb aabb::surrounding_box(const aabb& a, const aabb& b)
{
	vec3 min = vec3::vmin(a.minimum, b.minimum);
	vec3 max = vec3::vmax(a.maximum, b.maximum);
	return aabb(min, max);
}