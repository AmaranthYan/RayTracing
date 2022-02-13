#pragma once

#include "material.h"
#include "../math/pdf.h"
#include "../3d/onb.h"

// Cook-Torrance BRDF model was from http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
// though I believe the article mistook roughness as alpha, and the last part about importance sampling is wrong,
// should convert pdf(vec_h) to pdf(vec_i) and also need not multiply by sin(theta_i) at the end, as the variable of the Monte Carlo integration is omega_i
// so I took the GGX sampling from https://agraphicsguynotes.com/posts/sample_microfacet_brdf/

// v = view direction = unit(-ray direction)
// i = incident direction = unit(scattered direction)
// h = half vector of view/incident
// n = macro normal of the surface
class glossy : public material
{
public:
	glossy() : albedo(), roughness_metallic_refractive() {}
	glossy(const std::shared_ptr<texture>& a, const std::shared_ptr<texture>& r_m_ri) : albedo(a), roughness_metallic_refractive(r_m_ri) {}
	glossy(const std::shared_ptr<texture>& a, double r, double m, double ri) : glossy(a, std::make_shared<solid_color>(vec3(r, m, ri))) {}
	glossy(const vec3& a, double r, double m, double ri) : glossy(std::make_shared<solid_color>(a), std::make_shared<solid_color>(vec3(r, m, ri))) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override;
	virtual vec3 brdf_cos(const ray& r, const hit_result& hit, const ray& scatt) const override;
	
	static __forceinline double rough_to_alpha(double r)
	{
		// Disney Smith GGX alpha = (0.5 + roughness / 2.0) ^ 2
		return 0.25 * (1.0 + r) * (1.0 + r);
	}

	static __forceinline vec3 fresnel(const vec3& v, const vec3& h, double ri, const vec3& a, double m); // Fresnel function
	static __forceinline double distribution(const vec3& h, const vec3& n, double aa); // normal Distribution function
	static __forceinline double geometry(const vec3& v, const vec3& i, const vec3& h, double aa); // Geometry shadowing function

private:
	std::shared_ptr<texture> albedo;
	// R channel = roughness, G channel = metallic, B channel = refractive index - 1
	std::shared_ptr<texture> roughness_metallic_refractive;
};

__forceinline vec3 glossy::fresnel(const vec3& v, const vec3& h, double ri, const vec3& a, double m)
{
	const auto vh = vec3::dot(v, h);
	if (vh > 0.0)
	{
		// use albedo for metal color and metallic for dielectric color
		// use f0 to lerp between the 2 colors, metal ri < 1, dielectric ri > 1
		double f0 = (1.0 - ri) / (1.0 + ri);
		f0 *= f0;

		const auto metal = f0 * a;
		const auto dielect = (1.0 - f0) * m;
		const vec3 r0
		{
			metal.r + dielect,
			metal.g + dielect,
			metal.b + dielect,
		};

		// Fresnel-Schlick approximation
		return r0 + vec3(1.0 - r0.r, 1.0 - r0.g, 1.0 - r0.b) * pow(1.0 - vh, 5);
	}
	return vec3(0.0, 0.0, 0.0);
}

__forceinline double glossy::distribution(const vec3& h, const vec3& n, double aa)
{
	// GGX Trowbridge-Reitz NDF
	const auto hn = vec3::dot(h, n);
	if (hn > 0.0)
	{
		const auto den = (aa - 1.0) * hn * hn + 1.0;
		return aa / (M_PI * den * den);
	}
	return 0.0;
}

__forceinline double glossy::geometry(const vec3& v, const vec3& i, const vec3& h, double aa)
{
	// Smith GGX Disney
	const auto vh = vec3::dot(v, h);
	if (vh > 0.0)
	{
		// Disney G1
		const auto g1 = 2.0 * vh / (vh + sqrt(aa + (1.0 - aa) * vh * vh));
		// G2 = G1(v) * G1(i), Disney G1(v) = G1(i)
		return g1 * g1;
	}
	return 0.0;
}

class ggx_pdf : public pdf
{
public:
	ggx_pdf(const vec3& n, const vec3& v, double a) : ortho(n), view(vec3::unit(v)), alpha_sqr(a * a) {}

	virtual double value(const vec3& dir, double tm) const override;
	virtual vec3 generate(double tm) const override;
	
private:
	static __forceinline vec3 random_half_vector(double aa);

	onb ortho;
	vec3 view;
	double alpha_sqr;
};

__forceinline vec3 ggx_pdf::random_half_vector(double aa)
{
	const auto r1 = drand();
	const auto r2 = drand();
	const auto phi = 2.0 * M_PI * r1;
	const auto cos_theta = sqrt((1.0 - r2) / (r2 * (aa - 1.0) + 1.0));
	const auto sin_theta = sqrt(1.0 - cos_theta * cos_theta);

	return vec3
	{
		cos(phi) * sin_theta,
		sin(phi) * sin_theta,
		cos_theta,
	};
}