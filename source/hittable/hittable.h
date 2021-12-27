#pragma once

#define _USE_MATH_DEFINES

#include <cassert>
#include <memory>
#include <vector>
#include "../ray.h"
#include "../3d/aabb.h"
#include "../math/mat3.h"

class material;

struct hit_result
{
	std::shared_ptr<material> mat;
	vec3 pos;
	vec3 norm;
	double u, v;
	double frac;
};

class hittable
{
public:
	virtual ~hittable() = default; // note that shared_ptr does not require virtual destructor to clean up derived classes, add it for safety anyway

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const = 0;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const = 0;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const
	{
		assert(false); // not implemented
		return 0.0;
	}

	virtual vec3 random(const vec3& p, double tm) const
	{
		assert(false); // not implemented
		return vec3(0.0, 0.0, 0.0);
	}
};

class sphere : public hittable
{
public:
	sphere() : cent(), rad(0), mat() {}
	sphere(const vec3& c, double r) : cent(c), rad(r), mat() {}
	sphere(const vec3& c, double r, std::shared_ptr<material> m) : cent(c), rad(r), mat(m) {}

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override
	{
		const vec3 r(rad, rad, rad);
		bbox = aabb(cent - r, cent + r);
		return true;
	}

	virtual vec3 center(double tm) const { return cent; }
	virtual bool intersect_ray(const ray& r, double t_min, double t_max, double& out_t) const;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override;
	virtual vec3 random(const vec3& p, double tm) const override;

	static __forceinline void calc_uv(const vec3& unit, double& u, double& v)
	{
		auto theta = acos(-unit.y);
		auto phi = atan2(-unit.z, unit.x) + M_PI; // atan2(a, b) = atan2(-a, -b) + pi
		u = phi / (2.0 * M_PI);
		v = theta / M_PI;
	}

protected:
	std::shared_ptr<material> mat;
	vec3 cent;
	double rad;
};

class moving_sphere : public sphere
{
public:
	moving_sphere() : sphere(), cent1(), tm0(0), tm1(0) {}
	moving_sphere(const vec3& c0, const vec3& c1, double t0, double t1, double r, std::shared_ptr<material> m) : sphere(c0, r, m), cent1(c1), tm0(t0), tm1(t1) {}

	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	virtual vec3 center(double tm) const override
	{
		return ((tm - tm0) * cent1 + (tm1 - tm) * cent) / (tm1 - tm0);
	}

private:
	vec3 cent1;
	double tm0, tm1;
};

struct hittable_list : public hittable
{
	hittable_list() : list() {}

	__forceinline void add_hittable(std::shared_ptr<hittable> hittable) { list.push_back(hittable); }
	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override;
	virtual vec3 random(const vec3& p, double tm) const override;

	std::vector<std::shared_ptr<hittable>> list;
};

template<unsigned AXIS>
class aligned_rectangle : public hittable
{
	static_assert(AXIS < 3, "axis must be 0, 1 or 2");

public:
	aligned_rectangle() : axis{}, dist(), norm(), mat() {};
	aligned_rectangle(double u0, double u1, double v0, double v1, double d, bool sign, const std::shared_ptr<material>& m) : axis{ { u0, u1 }, { v0, v1 } }, dist(d), mat(m)
	{
		norm = vec3(0, 0, 0);
		norm[AXIS] = sign ? 1 : -1;

		area = (axis[0][1] - axis[0][0]) * (axis[1][1] - axis[1][0]);
	};

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override;
	virtual vec3 random(const vec3& p, double tm) const override;

private:
	static constexpr unsigned axis_u = (AXIS + 1) % 3;
	static constexpr unsigned axis_v = (AXIS + 2) % 3;

	std::shared_ptr<material> mat;
	vec3 norm;
	double axis[2][2];
	double dist;
	double area;
};

using yz_rect = aligned_rectangle<0>;
using zx_rect = aligned_rectangle<1>;
using xy_rect = aligned_rectangle<2>;


template<unsigned AXIS>
bool aligned_rectangle<AXIS>::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	auto t = (this->dist - r.ori()[AXIS]) / r.dir()[AXIS];
	if (t >= t_min && t <= t_max) // t could NaN
	{
		auto u = r.ori()[axis_u] + t * r.dir()[axis_u];
		auto v = r.ori()[axis_v] + t * r.dir()[axis_v];

		if (u < this->axis[0][0] || u > this->axis[0][1] || v < this->axis[1][0] || v > this->axis[1][1])
		{
			return false;
		}

		res.u = (u - this->axis[0][0]) / (this->axis[0][1] - this->axis[0][0]);
		res.v = (v - this->axis[1][0]) / (this->axis[1][1] - this->axis[1][0]);

		res.frac = t;
		res.pos = r.pos(t);
		res.norm = this->norm;
		res.mat = this->mat;
		return true;
	}
	else
	{
		return false;
	}
}

template<unsigned AXIS>
bool aligned_rectangle<AXIS>::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	constexpr auto padding = 1e-4;
	vec3 min, max;
	min[AXIS] = this->dist - padding;
	min[axis_u] = this->axis[0][0];
	min[axis_v] = this->axis[1][0];
	max[AXIS] = this->dist + padding;
	max[axis_u] = this->axis[0][1];
	max[axis_v] = this->axis[1][1];
	bbox = aabb(min, max);
	return true;
}

template<unsigned AXIS>
double aligned_rectangle<AXIS>::pdf(const vec3& p, const vec3& dir, double tm) const
{
	hit_result res;
	if (!this->hit(ray(p, dir, tm), ray::epsilon, std::numeric_limits<double>::infinity(), res))
	{
		return 0.0;
	}
	
	const auto len_sqr = dir.length_sqr();

	const double dist_sqr = res.frac * res.frac * len_sqr;
	const double cos = fabs(vec3::dot(res.norm, dir) / sqrt(len_sqr));

	return dist_sqr / (this->area * cos);
}

template<unsigned AXIS>
vec3 aligned_rectangle<AXIS>::random(const vec3& p, double tm) const
{
	vec3 rand;
	rand[AXIS] = this->dist;
	rand[axis_u] = drand(this->axis[0][0], this->axis[0][1]);
	rand[axis_v] = drand(this->axis[1][0], this->axis[1][1]);
	return rand - p;
}

class aligned_box : public hittable
{
public:
	aligned_box() : sides(), pmin(), pmax(), mat() {}
	aligned_box(const vec3& p0, const vec3& p1, const std::shared_ptr<material>& m);

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override
	{
		return sides.hit(r, t_min, t_max, res);
	}

	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override
	{
		bbox = aabb(pmin, pmax);
		return true;
	}

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override
	{
		return sides.pdf(p, dir, tm);
	}

	virtual vec3 random(const vec3& p, double tm) const override
	{
		return sides.random(p, tm);
	}

private:
	std::shared_ptr<material> mat;
	hittable_list sides;
	vec3 pmin, pmax;
};

class translate : public hittable
{
public:
	translate(const std::shared_ptr<hittable>& h, const vec3& o) : entity(h), offset(o) {}

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override;
	virtual vec3 random(const vec3& p, double tm) const override;

private:
	std::shared_ptr<hittable> entity;
	vec3 offset;
};

class rotate : public hittable
{
public:
	rotate(const std::shared_ptr<hittable>& h, const vec3& o, const vec3& ax, double a);
	rotate(const std::shared_ptr<hittable>& h, const vec3& ax, double a) : rotate(h, vec3(0.0, 0.0, 0.0), ax, a) {}
	
	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	virtual double pdf(const vec3& p, const vec3& dir, double tm) const override;
	virtual vec3 random(const vec3& p, double tm) const override;

private:
	std::shared_ptr<hittable> entity;
	mat3 rotation, reversed;
	vec3 origin;
	vec3 axis;
	double angle;
};

class scale : public hittable
{
public:
	scale(const std::shared_ptr<hittable>& h, const vec3& s) : entity(h), ratio(s), inv_ratio(1.0 / ratio) {};
	scale(const std::shared_ptr<hittable>& h, double s) : entity(h), ratio(vec3(s, s, s)), inv_ratio(1.0 / ratio) {}

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

private:
	std::shared_ptr<hittable> entity;	
	vec3 ratio, inv_ratio;
};

class triangle : public hittable
{
public:
	struct text_coord
	{
		text_coord() : u(), v() {}
		text_coord(double x, double y) : u(x), v(y) {}

		double u;
		double v;
	};

	triangle() : vertices(), vert{}, normals(), norm{}, uvs(), coord{}, mat() {}
	triangle(const vec3& v0, const vec3& v1, const vec3& v2, const std::shared_ptr<material>& m);
	triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<material>& m);
	triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<vec3>>& nlist, int nidx[3], const std::shared_ptr<material>& m);
	triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<text_coord>>& clist, int cidx[3], const std::shared_ptr<material>& m);
	triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<vec3>>& nlist, int nidx[3], const std::shared_ptr<std::vector<text_coord>>& clist, int cidx[3], const std::shared_ptr<material>& m);

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

	template<unsigned VERT>
	__forceinline const vec3& vertex() const
	{
		static_assert(VERT < 3, "vertex must be 0, 1 or 2");
		return (*this->vertices)[vert[VERT]];
	}

	template<unsigned VERT_0, unsigned VERT_1>
	__forceinline vec3 edge() const
	{
		static_assert(VERT_0 < 3 && VERT_1 < 3, "vertices must be 0, 1 or 2");
		return this->vertex<VERT_1>() - this->vertex<VERT_0>();
	}

	template<unsigned VERT>
	__forceinline const vec3& normal() const
	{
		static_assert(VERT < 3, "vertex must be 0, 1 or 2");
		return (*this->normals)[norm[VERT]];
	}

	template<unsigned VERT>
	__forceinline const double& u() const
	{
		static_assert(VERT < 3, "vertex must be 0, 1 or 2");
		return (*this->uvs)[coord[VERT]].u;
	}

	template<unsigned VERT>
	__forceinline const double& v() const
	{
		static_assert(VERT < 3, "vertex must be 0, 1 or 2");
		return (*this->uvs)[coord[VERT]].v;
	}

private:
	std::shared_ptr<material> mat;
	std::shared_ptr<std::vector<vec3>> vertices;
	std::shared_ptr<std::vector<vec3>> normals;
	std::shared_ptr<std::vector<text_coord>> uvs;	
	int vert[3];
	int norm[3];
	int coord[3];
};