#pragma once

#include "../lib/stb_image.h"
#include "../math/vec3.h"
#include "../math/noise.h"

template<class T>
__forceinline T clamp(const T& val, const T& min, const T& max)
{
	return min < val ? (max > val ? val : max) : min;
}

#pragma optimize( "", off )
__forceinline double remap(double v, double t0, double t1, double s0, double s1)
{
	double r = s0 + (v - t0) / (t1 - t0) * (s1 - s0);

	if (r != r)
	{
		__debugbreak();
	}
	return r;
}
#pragma optimize( "", on )

class texture
{
public:
	virtual ~texture() = default;

	virtual vec3 sample(double u, double v, const vec3& pos) const = 0;
};

class solid_color : public texture
{
public:
	solid_color() : color() {}
	solid_color(const vec3& col) : color(col) {}
	solid_color(double r, double g, double b) : color(vec3(r, g, b)) {}

	virtual vec3 sample(double u, double v, const vec3& pos) const override
	{
		return color;
	}

private:
	vec3 color;
};

class checker_texture : public texture
{
public:
	checker_texture() : tex{}, scale() {}
	checker_texture(std::shared_ptr<texture> tex0, std::shared_ptr<texture> tex1, const vec3& s) : tex{ tex0, tex1 }, scale(s) {}
	checker_texture(const vec3& col0, const vec3& col1, const vec3& s) : checker_texture(std::make_shared<solid_color>(col0), std::make_shared<solid_color>(col1), s) {}

	virtual vec3 sample(double u, double v, const vec3& pos) const override;

private:
	std::shared_ptr<texture> tex[2];
	vec3 scale;
};

template<class NOISE>
class noise_texture : public texture
{
public:
	explicit noise_texture(const NOISE& n) : noise(n), scale(), turb() {}
	noise_texture(const NOISE& n, const vec3& s, double t) : noise(n), scale(s), turb(t) {}
	noise_texture(const NOISE& n, double s, double t) : noise_texture(n, vec3(s, s, s), t) {}

	virtual vec3 sample(double u, double v, const vec3& pos) const override
	{
		//return vec3(0.5, 0.5, 0.5) * (1.0 + noise.noise(scale * pos));
		//return vec3(1.0, 1.0, 1.0) * (0.0 + noise.turbulence(scale * pos));
		return vec3(0.5, 0.5, 0.5) * (1.0 + sin(vec3::dot(scale, pos) + turb * noise.turbulence(pos)));
	}

private:
	const NOISE& noise;
	vec3 scale;
	double turb;
};

template<class NOISE_P, class NOISE_W>
class cloud_texture : public texture
{
public:
	explicit cloud_texture(const NOISE_P& p, const NOISE_W& w) : perlin(p), worley(w), freq(0.5), cov(0.85) {}
	cloud_texture(const NOISE_P& p, const NOISE_W& w, double fq, double cv) : perlin(p), worley(w), freq(fq), cov(clamp(cv, 0.0, 0.99)) {}

	virtual vec3 sample(double u, double v, const vec3& pos) const override
	{
		auto w = worley.turbulence(pos * freq);
		auto p = abs(perlin.turbulence(pos * 4.0 * freq, 7));
		p = remap(p, 0.0, 1.0, w, 1.0); // Perlin-Worley

		w = 0.625 * w + 0.125 * worley.turbulence(pos * 2.0 * freq) + 0.25 * worley.turbulence(pos * 4.0 * freq);
		auto cloud = remap(p, w - 1.0, 1.0, 0.0, 1.0);
		cloud = clamp(remap(cloud, cov, 1.0, 0.0, 1.0), 0.0, 1.0 ); // cloud coverage

		return vec3(cloud, cloud, cloud);
	}

private:
	const NOISE_P& perlin;
	const NOISE_W& worley;
	double freq;
	double cov;
};

class image_texture : public texture
{
public:
	static constexpr int bytes_pixel = 3;

	image_texture() : data(nullptr), width(0), height(0), channels(0), bytes_scanline(0) {}
	image_texture(const char* file);

	virtual ~image_texture();

	virtual vec3 sample(double u, double v, const vec3& pos) const override;

private:
	stbi_uc* data;
	int width, height;
	int channels;
	int bytes_scanline;
};