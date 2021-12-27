#pragma once

#include "../hittable/hittable.h"
#include "texture.h"
#include "../math/pdf.h"

struct scatter_record
{
	std::shared_ptr<pdf> pdf;
	ray spec; // specular ray
	vec3 atten;
	bool specular;
};

__forceinline vec3 reflect(const vec3& v, const vec3& n)
{
	return v - 2 * vec3::dot(v, n) * n;
}

__forceinline vec3 refract(const vec3& uv, const vec3& n, double r)
{
	auto c = fmin(-vec3::dot(n, uv), 1.0);
	// squared module of refracted vector projected onto -n, fabs to prevent < 0
	auto psqr = fabs(1.0 - r * r * (1 - c * c));
	return r * uv + (r * c - sqrt(psqr)) * n;
}

class material
{
public:
	virtual ~material() = default;

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const = 0;
	virtual double scattering_pdf(const ray& r, const hit_result& hit, const ray& scatt) const { return 0.0; };
	virtual vec3 emit(const ray& r, const hit_result& hit) const { return vec3(0.0, 0.0, 0.0); };
};

class lambertian : public material
{
public:
	lambertian() : albedo() {}
	lambertian(const std::shared_ptr<texture>& a) : albedo(a) {}
	lambertian(const vec3& c) : lambertian(std::make_shared<solid_color>(c)) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override;
	virtual double scattering_pdf(const ray& r, const hit_result& hit, const ray& scatt) const override;

private:
	std::shared_ptr<texture> albedo;
};

class metal : public material
{
public:
	metal() : albedo(), fuzz() {}
	metal(const std::shared_ptr<texture>& a, double f = 0.0) : albedo(a), fuzz(f < 1 ? f : 1) {}
	metal(const vec3& c, double f = 0.0) : metal(std::make_shared<solid_color>(c), f) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override;
	static __forceinline vec3 reflectance(double cos, const vec3& r0);

private:
	std::shared_ptr<texture> albedo;
	double fuzz;
};

__forceinline vec3 metal::reflectance(double cos, const vec3& r0)
{
	// Fresnel-Schlick approximation for metal
	return r0 + (vec3(1.0, 1.0, 1.0) - r0) * pow(1 - cos, 5);
}

class dielectric : public material
{
public:
	dielectric() : absorb(), ir() {}
	dielectric(double r, vec3 ab) : ir(r), absorb(ab) {}
	dielectric(double r) : dielectric(r, vec3(0.0, 0.0, 0.0)) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override;
	static __forceinline double reflectance(double cos, double ratio);
	static __forceinline vec3 absorption(const vec3& ab, double d);

private:
	vec3 absorb;
	double ir; // index of refraction
};

__forceinline double dielectric::reflectance(double cos, double ratio)
{
	// Fresnel-Schlick approximation for dielectric
	auto r0 = (1.0 - ratio) / (1.0 + ratio);
	r0 *= r0;
	return r0 + (1 - r0) * pow(1 - cos, 5);
}

__forceinline vec3 dielectric::absorption(const vec3& ab, double d)
{
	// Beer-Lambert law
	return vec3(exp(-ab.x * d), exp(-ab.y* d), exp(-ab.z * d));
}

class diffuse_light : public material
{
public:
	diffuse_light() : luminance() {}
	diffuse_light(const std::shared_ptr<texture>& l) : luminance(l) {}
	diffuse_light(const vec3& c) : diffuse_light(std::make_shared<solid_color>(c)) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override
	{
		return false;
	}

	virtual vec3 emit(const ray& r, const hit_result& hit) const override
	{
		return vec3::is_back(r.dir(), hit.norm) ? vec3(0.0, 0.0, 0.0) : luminance->sample(hit.u, hit.v, hit.pos);
	}

private:
	std::shared_ptr<texture> luminance;
};

class isotropic : public material
{
public:
	isotropic() : albedo() {}
	isotropic(std::shared_ptr<texture> a) : albedo(a) {}
	isotropic(const vec3& c) : isotropic(std::make_shared<solid_color>(c)) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override;
	virtual double scattering_pdf(const ray& r, const hit_result& hit, const ray& scatt) const override;

private:
	std::shared_ptr<texture> albedo;
};