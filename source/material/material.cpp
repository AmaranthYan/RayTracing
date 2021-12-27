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
	scatt.atten = this->albedo->sample(hit.u, hit.v, hit.pos);
	scatt.pdf = std::make_shared<cos_pdf>(hit.norm);
	scatt.specular = false;
	return !vec3::is_back(r.dir(), hit.norm);
}

double lambertian::scattering_pdf(const ray& r, const hit_result& hit, const ray& scatt) const
{
	auto cos = vec3::dot(hit.norm, vec3::unit(scatt.dir()));
	return cos < 0.0 ? 0.0 : cos / M_PI;
}

bool metal::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	bool back = vec3::is_back(r.dir(), hit.norm);
	vec3 norm = back ? -hit.norm : hit.norm;

	auto udir = vec3::unit(r.dir());
	vec3 reflected = reflect(udir, norm);
	scatt.spec = ray(hit.pos, reflected + this->fuzz * vec3::random_unit_sphere(), r.tm());

	auto cos = fmin(-vec3::dot(udir, norm), 1.0);
	scatt.atten = metal::reflectance(cos, this->albedo->sample(hit.u, hit.v, hit.pos)); // use albedo as r0

	scatt.pdf = nullptr;
	scatt.specular = true;
	return vec3::dot(scatt.spec.dir(), norm) > 0;
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

	vec3 spec;
	if (no_refract || dielectric::reflectance(cos, refract_ratio) > drand())
	{
		spec = reflect(udir, norm);
	}
	else
	{
		spec = refract(udir, norm, refract_ratio);

	}
	scatt.spec = ray(hit.pos, spec, r.tm());

	if (back)
	{
		// absorption based on traveled distance of the ray inside dielectric
		auto dist = (hit.pos - r.ori()).length();
		scatt.atten = dielectric::absorption(absorb, dist);
	}
	else
	{
		scatt.atten = vec3(1.0, 1.0, 1.0);
	}
	scatt.pdf = nullptr;
	scatt.specular = true;
	return true;
}

bool isotropic::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	scatt.atten = albedo->sample(hit.u, hit.v, hit.pos);
	scatt.pdf = std::make_shared<uniform_pdf>();
	scatt.specular = false;
	return true;
}

double isotropic::scattering_pdf(const ray& r, const hit_result& hit, const ray& scatt) const
{
	return 1.0 / (4.0 * M_PI);
}
