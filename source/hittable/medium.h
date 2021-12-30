#pragma once

#include "hittable.h"
#include "../material/material.h"
#include "../material/texture.h"

class medium : public hittable
{
public:
	medium() : boundary(), phase(), max_density(), inv_density() {}
	medium(const std::shared_ptr<hittable>& b, double d, const std::shared_ptr<texture> a ) : boundary(b), phase(std::make_shared<isotropic>(a)), max_density(d), inv_density(1.0 / d) {}
	medium(const std::shared_ptr<hittable>& b, double d, const vec3& c) : medium(b, d, std::make_shared<solid_color>(c)) {}

	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override
	{
		return boundary->bounding_box(tm0, tm1, bbox);
	}

protected:
	virtual double density(const vec3& pos) const = 0;

	std::shared_ptr<hittable> boundary;
	std::shared_ptr<material> phase;
	double max_density;
	double inv_density;
};

class uniform_medium : public medium
{
public:
	uniform_medium() : medium() {}
	uniform_medium(const std::shared_ptr<hittable>& b, double d, const std::shared_ptr<texture> a) : medium(b, d, a) {}
	uniform_medium(const std::shared_ptr<hittable>& b, double d, const vec3& c) : medium(b, d, c) {}

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;

protected:
	virtual double density(const vec3& pos) const override { return max_density; }
};

template<class NOISE_P, class NOISE_W>
class cloud : public medium
{
public:
	cloud(const std::shared_ptr<hittable>& b, double d, const std::shared_ptr<texture> a, const NOISE_P& p, const NOISE_W& w) : medium(b, d, a), perlin(p), worley(w), freq(0.5), cov(0.85), height_fading(false), height_min(), height_max(), fade_dist() {}
	cloud(const std::shared_ptr<hittable>& b, double d, const std::shared_ptr<texture> a, const NOISE_P& p, const NOISE_W& w, double fq, double cv) : medium(b, d, a), perlin(p), worley(w), freq(fq), cov(cv), height_fading(false), height_min(), height_max(), fade_dist() {}
	cloud(const std::shared_ptr<hittable>& b, double d, const vec3& c, const NOISE_P& p, const NOISE_W& w) : cloud(b, d, std::make_shared<solid_color>(c), p, w) {}
	cloud(const std::shared_ptr<hittable>& b, double d, const vec3& c, const NOISE_P& p, const NOISE_W& w, double fq, double cv) : cloud(b, d, std::make_shared<solid_color>(c), p, w, fq, cv) {}

	void set_height_fading(double min, double max, double dist)
	{
		height_min = min;
		height_max = max;
		fade_dist = dist;

		height_fading = true;
	}

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;

protected:
	virtual double density(const vec3& p) const override;

private:
	const NOISE_P& perlin;
	const NOISE_W& worley;
	double freq;
	double cov;

	double height_min, height_max;
	double fade_dist;
	bool height_fading;
};

template<class NOISE_P, class NOISE_W>
double cloud<NOISE_P, NOISE_W>::density(const vec3& pos) const
{
	const auto w = this->worley.turbulence(pos * this->freq);
	auto p = abs(this->perlin.turbulence(pos * 4.0 * this->freq, 7));
	p = remap(p, 0.0, 1.0, w, 1.0); // Perlin-Worley
	const auto t = 0.625 * this->worley.turbulence(pos * 2.0 * this->freq) + 0.25 * this->worley.turbulence(pos * 4.0 * this->freq) + 0.125 * this->worley.turbulence(pos * 8.0 * this->freq);
	auto k = remap(p, t - 1.0, 1.0, 0.0, 1.0);

	auto c = this->cov;
	if (this->height_fading)
	{
		// coverage fading on y-axis
		if (pos.y < this->height_min)
		{
			c = remap(pos.y, this->height_min, this->height_min - this->fade_dist, c, 1.0);
		}
		else if (pos.y > this->height_max)
		{
			c = remap(pos.y, this->height_max, this->height_max + this->fade_dist, c, 1.0);
		}
	}	
	c = std::min(1.0, c);

	k = clamp(remap(k, c, 1.0, 0.0, 1.0), 0.0, 1.0); // cloud coverage

	return this->max_density * k;
}

template<class NOISE_P, class NOISE_W>
bool cloud<NOISE_P, NOISE_W>::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	double t0 = t_min;
	double t1 = t0 - ray::epsilon;
	double inv_len = 1.0 / r.dir().length();

	double t_hit;
	bool once = true;

	hit_result h;

	while (this->boundary->hit(r, t1 + ray::epsilon, t_max, h))
	{
		t1 = h.frac;
		if (!vec3::is_back(r.dir(), h.norm))
		{
			t0 = h.frac;
			continue;
		}

		if (once)
		{
			t_hit = -this->inv_density * log(1.0 - drand()) * inv_len;
			once = false;
		}

		double t_depth = t1 - t0;
		while (t_hit < t_depth)
		{
			// Woodcock tracking
			if (this->density(r.ori() + (t0 + t_hit) * r.dir()) * this->inv_density > drand())
			{
				res.frac = t0 + t_hit;
				res.pos = r.pos(res.frac);
				res.mat = this->phase;
				return true;
			}
			else
			{
				t0 += t_hit;
				t_depth -= t_hit;
				t_hit = -this->inv_density * log(1.0 - drand()) * inv_len;
			}
			
		}

		t_hit -= t_depth;
	}

	return false;
}