#include "material.h"
#include "../3d/onb.h"

__forceinline vec3 random_cos_direction()
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

bool lambertian::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	scatt.set_non_specular(std::make_shared<cos_pdf>(hit.norm));
	return !vec3::is_back(r.dir(), hit.norm);
}

vec3 lambertian::brdf_cos(const ray& r, const hit_result& hit, const ray& scatt) const
{
	auto cos_pi = vec3::dot(hit.norm, vec3::unit(scatt.dir())) / M_PI;
	if (cos_pi > 0)
	{
		// Lambert BRDF = A/Pi
		auto atten = this->albedo->sample(hit.u, hit.v, hit.pos) * cos_pi;
		return atten;
	}
	else
	{
		return vec3(0.0, 0.0, 0.0);
	}
}

bool metal::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	bool back = vec3::is_back(r.dir(), hit.norm);
	vec3 norm = back ? -hit.norm : hit.norm;

	auto udir = vec3::unit(r.dir());
	vec3 reflected = reflect(udir, norm);
	auto spec = ray(hit.pos, reflected + this->fuzz * vec3::random_unit_sphere(), r.tm());

	auto cos = fmin(-vec3::dot(udir, norm), 1.0);
	auto atten = metal::reflectance(cos, this->albedo->sample(hit.u, hit.v, hit.pos)); // use albedo as r0

	scatt.set_specular(spec, atten);
	return vec3::dot(spec.dir(), norm) > 0;
}

bool dielectric::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	bool back = vec3::is_back(r.dir(), hit.norm);
	vec3 norm = back ? -hit.norm : hit.norm;

	double refract_ratio = back ? this->ir : 1.0 / this->ir;

	auto udir = vec3::unit(r.dir());
	auto cos = fmin(-vec3::dot(udir, norm), 1.0);
	auto sin = sqrt(1.0 - cos * cos);
	bool no_refract = refract_ratio * sin > 1.0;

	vec3 ref;
	if (no_refract || dielectric::reflectance(cos, refract_ratio) > drand())
	{
		ref = reflect(udir, norm);
	}
	else
	{
		ref = refract(udir, norm, refract_ratio);

	}
	auto spec = ray(hit.pos, ref, r.tm());

	vec3 atten(1.0, 1.0, 1.0);
	if (back)
	{
		// absorption based on traveled distance of the ray inside dielectric
		auto dist = (hit.pos - r.ori()).length();
		atten = dielectric::absorption(absorb, dist);
	}

	scatt.set_specular(spec, atten);
	return true;
}

bool isotropic::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	scatt.set_non_specular(std::make_shared<uniform_pdf>());
	return true;
}

vec3 isotropic::brdf_cos(const ray& r, const hit_result& hit, const ray& scatt) const
{
	auto atten = albedo->sample(hit.u, hit.v, hit.pos);
	return atten / (4.0 * M_PI);
}
