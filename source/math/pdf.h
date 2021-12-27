#pragma once

#include "vec3.h"
#include "../hittable/hittable.h"
#include "../3d/onb.h"

class pdf
{
public:
	virtual ~pdf() = default;

	virtual double value(const vec3& dir, double tm) const = 0;
	virtual vec3 generate(double tm) const = 0;
};

class cos_pdf : public pdf
{
public:
	cos_pdf(const vec3& n) : ortho(n) {}

	virtual double value(const vec3& dir, double tm) const override
	{
		const auto cos = vec3::dot(ortho.w, vec3::unit(dir));		
		return cos < 0.0 ? 0.0 : cos / M_PI;
	}

	virtual vec3 generate(double tm) const override
	{
		return ortho.local(cos_pdf::random_cos_direction());
	}

private:
	static __forceinline vec3 random_cos_direction();

	onb ortho;
};

__forceinline vec3 cos_pdf::random_cos_direction()
{
	const auto r1 = drand();
	const auto r2 = drand();
	const auto phi = 2.0 * M_PI * r1;
	const auto sin_theta = sqrt(r2);
	const auto cos_theta = sqrt(1 - r2);

	return vec3
	{
		cos(phi) * sin_theta,
		sin(phi) * sin_theta,
		cos_theta
	};
}

class uniform_pdf : public pdf
{
public:
	virtual double value(const vec3& dir, double tm) const override
	{
		return 1.0 / (4.0 * M_PI);
	}

	virtual vec3 generate(double tm) const override
	{
		return vec3::random_unit_vector();
	}
};

class hittable_pdf : public pdf
{
public:
	hittable_pdf(const vec3& o, const std::shared_ptr<hittable>& h) : origin(o), target(h) {};

	virtual double value(const vec3& dir, double tm) const override
	{
		return target->pdf(origin, dir, tm);
	}

	virtual vec3 generate(double tm) const override
	{
		return target->random(origin, tm);
	}

private:
	std::shared_ptr<hittable> target;
	vec3 origin;
};

class mix_pdf : public pdf
{
public:
	mix_pdf(const std::shared_ptr<pdf>& p0, const std::shared_ptr<pdf>& p1, double f) : p{ p0, p1 }, frac(clamp(f, 0.0, 1.0)) {}

	virtual double value(const vec3& dir, double tm) const override
	{
		return frac * p[0]->value(dir, tm) + (1.0 - frac) * p[1]->value(dir, tm);
	}

	virtual vec3 generate(double tm) const override
	{
		if (drand() < frac)
		{
			return p[0]->generate(tm);
		}
		else
		{
			return p[1]->generate(tm);
		}
	}

private:
	std::shared_ptr<pdf> p[2];
	double frac;
};