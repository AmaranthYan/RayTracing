#include "hittable.h"
#include "../3d/onb.h"

bool sphere::intersect_ray(const ray& r, double t_min, double t_max, double& out_t) const
{
	auto cent = this->center(r.tm());
	// t * t * dot(dir, dir) + 2 * t * dot(dir, ori - cent) + dot(ori - cent, ori - cent) - rad * rad = 0
	vec3 oc = r.ori() - cent;
	double a = vec3::dot(r.dir(), r.dir());
	double b = vec3::dot(r.dir(), oc);
	double c = vec3::dot(oc, oc) - this->rad * this->rad;
	double disc = b * b - a * c;
	if (disc > 0)
	{
		double dsqrt = sqrt(disc);
		auto rt = (-b - dsqrt) / a;
		if (rt < t_min || t_max < rt)
		{
			rt = (-b + dsqrt) / a;
			if (rt < t_min || t_max < rt)
			{
				return false;
			}
		}

		out_t = rt;
		return true;
	}
	return false;
}

bool sphere::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	double t;
	if (this->intersect_ray(r, t_min, t_max, t))
	{
		res.frac = t;
		res.pos = r.pos(t);
		res.norm = (res.pos - this->cent) / this->rad;
		sphere::calc_uv(res.norm, res.u, res.v);
		res.mat = this->mat;
		return true;
	}
	return false;
}

__forceinline vec3 random_to_sphere(double rad, double dist_sqr)
{
	auto r1 = drand();
	auto r2 = drand();
	
	auto phi = 2.0 * M_PI * r1;
	auto cos_theta = 1.0 + r2 * (sqrt(1.0 - rad * rad / dist_sqr) - 1.0);
	auto sin_theta = sqrt(1.0 - cos_theta * cos_theta);
	return vec3
	{
		cos(phi) * sin_theta,
		sin(phi)* sin_theta,
		cos_theta
	};
}

double sphere::pdf(const vec3& p, const vec3& dir, double tm) const
{
	hit_result res;
	if (!this->hit(ray(p, dir, tm), ray::epsilon, std::numeric_limits<double>::infinity(), res))
	{
		return 0.0;
	}
	
	auto dist = this->center(tm) - p;
	auto cos_theta = sqrt(1.0 - this->rad * this->rad / dist.length_sqr());
	auto solid_angle = 2.0 * M_PI * (1.0 - cos_theta);
	return 1.0 / solid_angle;
}

vec3 sphere::random(const vec3& p, double tm) const
{
	auto dir = this->center(tm) - p;
	onb ortho(dir);
	return ortho.local(random_to_sphere(this->rad, dir.length_sqr()));
}

bool moving_sphere::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	const vec3 r(this->rad, this->rad, this->rad);
	aabb box0(this->center(tm0) - r, this->center(tm0) + r);
	aabb box1(this->center(tm1) - r, this->center(tm1) + r);
	bbox = aabb::surrounding_box(box0, box1);
	return true;
}

bool hittable_list::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	hit_result result;
	bool hit_any = false;
	for (const auto& hittable : this->list)
	{
		if (hittable->hit(r, t_min, t_max, result))
		{
			hit_any = true;
			t_max = result.frac;
			res = result;
		}
	}
	return hit_any;
}

bool hittable_list::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	if (this->list.empty())
	{
		return false;
	}

	bool first = true;
	aabb box;

	for (const auto& h : this->list)
	{
		if (!h->bounding_box(tm0, tm1, box))
		{
			return false;
		}

		bbox = first ? box : aabb::surrounding_box(bbox, box);
		first = false;
	}
	return true;
}

double hittable_list::pdf(const vec3& p, const vec3& dir, double tm) const
{
	assert(!this->list.empty());

	auto w = 1.0 / this->list.size();
	double val = 0.0;
	for (const auto& hitable : this->list)
	{
		val += w * hitable->pdf(p, dir, tm);
	}
	return val;
}

vec3 hittable_list::random(const vec3& p, double tm) const
{
	assert(!this->list.empty());

	auto idx = static_cast<size_t>(drand() * this->list.size());
	return this->list[idx]->random(p, tm);
}

aligned_box::aligned_box(const vec3& p0, const vec3& p1, const std::shared_ptr<material>& m) : pmin(p0), pmax(p1), mat(m)
{
	this->sides.add_hittable(std::make_shared<xy_rect>(p0.x, p1.x, p0.y, p1.y, p0.z, false, m)); // front
	this->sides.add_hittable(std::make_shared<xy_rect>(p0.x, p1.x, p0.y, p1.y, p1.z, true, m)); // back
	this->sides.add_hittable(std::make_shared<yz_rect>(p0.y, p1.y, p0.z, p1.z, p0.x, false, m)); // right
	this->sides.add_hittable(std::make_shared<yz_rect>(p0.y, p1.y, p0.z, p1.z, p1.x, true, m)); // left
	this->sides.add_hittable(std::make_shared<zx_rect>(p0.z, p1.z, p0.x, p1.x, p0.y, false, m)); // bottom
	this->sides.add_hittable(std::make_shared<zx_rect>(p0.z, p1.z, p0.x, p1.x, p1.y, true, m)); // top
}

bool translate::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	ray moved(r.ori() - this->offset, r.dir(), r.tm());
	if (this->entity->hit(moved, t_min, t_max, res))
	{
		res.pos += this->offset;
		return true;
	}
	return false;
}

bool translate::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	if (this->entity->bounding_box(tm0, tm1, bbox))
	{
		bbox = aabb(bbox.minimum + this->offset, bbox.maximum + this->offset);
		return true;
	}
	return false;
}

double translate::pdf(const vec3& p, const vec3& dir, double tm) const
{
	return entity->pdf(p - this->offset, dir, tm);
}

vec3 translate::random(const vec3& p, double tm) const
{
	return entity->random(p - this->offset, tm);
}

rotate::rotate(const std::shared_ptr<hittable>& h, const vec3& o, const vec3& ax, double a) : entity(h), origin(o), axis(vec3::unit(ax)), angle(a * M_PI / 180.0)
{
	const mat3 k
	{
		{ 0.0, -this->axis.z, this->axis.y },
		{ this->axis.z, 0.0, -this->axis.x },
		{ -this->axis.y, this->axis.x, 0.0 }
	};

	// Rodrigues' Rotation Formula I + sin(a) * K + (1- cos(a)) * K * K
	const auto c = mat3::identity() + (1 - cos(this->angle)) * k * k;
	const auto s = sin(this->angle) * k;

	this->rotation = c + s;
	this->reversed = c - s;
}

bool rotate::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	// rotate the ray in reverse
	const vec3 rev_ori = this->reversed * (r.ori() - this->origin) + this->origin;
	const vec3 rev_dir = this->reversed * r.dir();
	ray rotated(rev_ori, rev_dir, r.tm());
	if (this->entity->hit(rotated, t_min, t_max, res))
	{
		res.pos = this->rotation * (res.pos - this->origin) + this->origin;
		res.norm = this->rotation * res.norm;
		return true;
	}
	return false;
}

bool rotate::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	if (this->entity->bounding_box(tm0, tm1, bbox))
	{
		constexpr double inf = std::numeric_limits<double>::infinity();
		vec3 min(inf, inf, inf);
		vec3 max(-inf, -inf, -inf);

		// this method does not generate the optimal aabb though
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 2; k++)
				{
					vec3 p
					{
						i * bbox.minimum.x + (1.0 - i) * bbox.maximum.x,
						j * bbox.minimum.y + (1.0 - j) * bbox.maximum.y,
						k * bbox.minimum.z + (1.0 - k) * bbox.maximum.z,
					};

					vec3 rot_p = this->rotation * (p - this->origin) + this->origin;

					for (int ax = 0; ax < 3; ax++)
					{
						min[ax] = std::min(min[ax], rot_p[ax]);
						max[ax] = std::max(max[ax], rot_p[ax]);
					}					
				}
			}
		}
		bbox = aabb(min, max);
		return true;
	}
	return false;
}

double rotate::pdf(const vec3& p, const vec3& dir, double tm) const
{
	const vec3 rev_p = this->reversed * (p - this->origin) + this->origin;
	const vec3 rev_dir = this->reversed * dir;
	return this->entity->pdf(rev_p, rev_dir, tm);
}

vec3 rotate::random(const vec3& p, double tm) const
{
	const vec3 rev_p = this->reversed * (p - this->origin) + this->origin;
	return this->rotation * this->entity->random(rev_p, tm);
}

bool scale::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	ray scaled(r.ori() * this->inv_ratio, r.dir() * this->inv_ratio, r.tm());
	if (this->entity->hit(scaled, t_min, t_max, res))
	{
		res.pos *= this->ratio;
		return true;
	}
	return false;
}

bool scale::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	if (this->entity->bounding_box(tm0, tm1, bbox))
	{
		bbox = aabb(bbox.minimum * this->ratio, bbox.maximum * this->ratio);
		return true;
	}
	return false;
}

triangle::triangle(const vec3& v0, const vec3& v1, const vec3& v2, const std::shared_ptr<material>& m)
	: vertices(std::make_shared<std::vector<vec3>>()), vert{ 0, 1, 2 }, normals(std::make_shared<std::vector<vec3>>()), norm{ 0, 0, 0 }, uvs(std::make_shared<std::vector<text_coord>>()), coord{ 0, 0, 0 }, mat(m)
{
	this->vertices->push_back(v0);
	this->vertices->push_back(v1);
	this->vertices->push_back(v2);

	auto e01 = this->edge<0, 1>();
	auto e02 = this->edge<0, 2>();
	auto n = vec3::unit(vec3::cross(e01, e02));
	this->normals->push_back(n);

	this->uvs->emplace_back(0.0, 0.0);
}

triangle::triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<material>& m)
	: vertices(vlist), vert{ vidx[0], vidx[1], vidx[2] }, normals(std::make_shared<std::vector<vec3>>()), norm{ 0, 0, 0 }, uvs(std::make_shared<std::vector<text_coord>>()), coord{ 0, 0, 0 }, mat(m)
{
	auto e01 = this->edge<0, 1>();
	auto e02 = this->edge<0, 2>();
	auto n = vec3::unit(vec3::cross(e01, e02));
	this->normals->push_back(n);

	this->uvs->emplace_back(0.0, 0.0);
}

triangle::triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<text_coord>>& clist, int cidx[3], const std::shared_ptr<material>& m)
	: vertices(vlist), vert{ vidx[0], vidx[1], vidx[2] }, normals(std::make_shared<std::vector<vec3>>()), norm{ 0, 0, 0 }, uvs(clist), coord{ cidx[0], cidx[1], cidx[2] }, mat(m)
{
	auto e01 = this->edge<0, 1>();
	auto e02 = this->edge<0, 2>();
	auto n = vec3::unit(vec3::cross(e01, e02));
	this->normals->push_back(n);
}

triangle::triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<vec3>>& nlist, int nidx[3], const std::shared_ptr<material>& m)
	: vertices(vlist), vert{ vidx[0], vidx[1], vidx[2] }, normals(nlist), norm{ nidx[0], nidx[1], nidx[2] }, uvs(std::make_shared<std::vector<text_coord>>()), coord{ 0, 0, 0 }, mat(m)
{
	this->uvs->emplace_back(0.0, 0.0);
}

triangle::triangle(const std::shared_ptr<std::vector<vec3>>& vlist, int vidx[3], const std::shared_ptr<std::vector<vec3>>& nlist, int nidx[3], const std::shared_ptr<std::vector<text_coord>>& clist, int cidx[3], const std::shared_ptr<material>& m)
	: vertices(vlist), vert{ vidx[0], vidx[1], vidx[2] }, normals(nlist), norm{ nidx[0], nidx[1], nidx[2] }, uvs(clist), coord{ cidx[0], cidx[1], cidx[2] }, mat(m) {}

bool triangle::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	// Möller-Trumbore algorithm
	const vec3 d = r.dir(); // D
	const vec3 t = r.ori() - this->vertex<0>(); // O - A
	const vec3 e0 = this->edge<0, 1>(); // B - A
	const vec3 e1 = this->edge<0, 2>(); // C - A
	const vec3 p = vec3::cross(d, e1); // D x E1
	const auto det = vec3::dot(p, e0); // P . E0

	if (abs(det) > 0) // not parallel
	{
		const vec3 q = vec3::cross(t, e0); // T x E0
		const auto inv_det = 1.0 / det;
		
		const auto t_hit = vec3::dot(q, e1) * inv_det; // (Q . E1) / (P . E0)
		if (t_hit < t_min || t_hit > t_max )
		{
			return false;
		}

		const auto b = vec3::dot(p, t) * inv_det; // (P . T) / (P . E0)
		if (b < 0 || b > 1)
		{
			return false;
		}
		const auto c = vec3::dot(q, d) * inv_det; // (Q . D) / (P . E0)
		if (c < 0 || c + b > 1)
		{
			return false;
		}
		const auto a = 1.0 - b - c;

		res.frac = t_hit;
		res.pos = r.pos(res.frac);
		res.norm = a * this->normal<0>() + b * this->normal<1>() + c * this->normal<2>();
		res.u = a * this->u<0>() + b * this->u<1>() + c * this->u<2>();
		res.v = a * this->v<0>() + b * this->v<1>() + c * this->v<2>();
		res.mat = this->mat;
		return true;
	}

	return false;
}

bool triangle::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	constexpr auto padding = 1e-4;

	vec3 min
	{
		std::min(this->vertex<0>().x, std::min(this->vertex<1>().x, this->vertex<2>().x)),
		std::min(this->vertex<0>().y, std::min(this->vertex<1>().y, this->vertex<2>().y)),
		std::min(this->vertex<0>().z, std::min(this->vertex<1>().z, this->vertex<2>().z))
	};
	vec3 max
	{
		std::max(this->vertex<0>().x, std::max(this->vertex<1>().x, this->vertex<2>().x)),
		std::max(this->vertex<0>().y, std::max(this->vertex<1>().y, this->vertex<2>().y)),
		std::max(this->vertex<0>().z, std::max(this->vertex<1>().z, this->vertex<2>().z))
	};

	if (min.x == max.x)
	{
		min.x -= padding;
		max.x += padding;
	}
	if (min.y == max.y)
	{
		min.y -= padding;
		max.y += padding;
	}
	if (min.z == max.z)
	{
		min.z -= padding;
		max.z += padding;
	}

	bbox = aabb(min, max);
	return true;
}
