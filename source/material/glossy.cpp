#include "glossy.h"

bool glossy::scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const
{
	double rough = this->roughness_metallic_refractive->sample(hit.u, hit.v, hit.pos).r;
	double alpha = glossy::rough_to_alpha(rough);
	scatt.set_non_specular(std::make_shared<ggx_pdf>(hit.norm, -r.dir(), alpha), 1.0 - 0.5 * alpha);
	return !vec3::is_back(r.dir(), hit.norm);
}

vec3 glossy::brdf_cos(const ray& r, const hit_result& hit, const ray& scatt) const
{
	const vec3 view = vec3::unit(-r.dir());
	const vec3 incident = vec3::unit(scatt.dir());

	const double vn = vec3::dot(view, hit.norm);
	const double in = vec3::dot(incident, hit.norm);
	// both view and incident direction should be above the surface
	if (vn > 0.0 && in > 0.0)
	{
		const vec3 half = vec3::unit(view + incident);

		const double vh = vec3::dot(view, half);
		const double hn = vec3::dot(half, hit.norm);
		const double ih = vec3::dot(incident, half);

		const auto rough_metal_refract = this->roughness_metallic_refractive->sample(hit.u, hit.v, hit.pos);
		const double rough = rough_metal_refract.r;
		const double metal = rough_metal_refract.g;
		const double refract = 1.0 + rough_metal_refract.b;

		const double alpha = glossy::rough_to_alpha(rough);
		const double aa = alpha * alpha;

		const vec3 color = this->albedo->sample(hit.u, hit.v, hit.pos);

		const auto f = glossy::fresnel(view, half, refract, color, metal); // also ks
		const auto d = glossy::distribution(half, hit.norm, aa);
		const auto g = glossy::geometry(view, incident, half, aa);

		// metallic surface doesn't diffuse light hence the (1.0 - metal)
		const auto kd = vec3(1.0 - f.r, 1.0 - f.g, 1.0 - f.b) * (1.0 - metal);

		const auto diffuse = kd * color * in / M_PI;
		const auto specular = f * d * g / (4.0 * vn);
		return diffuse + specular;
	}
	return vec3(0.0, 0.0, 0.0);
}

double ggx_pdf::value(const vec3& dir, double tm) const
{
	const vec3 half = vec3::unit(this->view + vec3::unit(dir));
	// pdf(h) should use NDF * dot(h, n) as the integral over the hemisphere = 1 by NDF definition
	const vec3 norm = this->ortho.w;
	const double pdf_h = glossy::distribution(half, norm, this->alpha_sqr) * vec3::dot(half, norm);
	// convert to pdf of incident direction, pdf(i) = pdf(h) / (4.0 * dot(v, h))
	const double pdf_i = pdf_h / (4.0 * vec3::dot(this->view, half));
	return std::max(0.0, pdf_i);
}

vec3 ggx_pdf::generate(double tm) const
{
	const vec3 half = ggx_pdf::random_half_vector(this->alpha_sqr); // generate the half vector h
	return reflect(-this->view, this->ortho.local(half)); // reflect view using world h as normal
}