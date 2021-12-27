#define STB_IMAGE_IMPLEMENTATION
#define _CRT_SECURE_NO_WARNINGS 1

#include <algorithm>
#include "texture.h"

vec3 checker_texture::sample(double u, double v, const vec3& pos) const
{
	auto freq = this->scale * pos;
	auto sign = sin(freq.x) * sin(freq.y) * sin(freq.z);
	return (sign < 0 ? this->tex[1]->sample(u, v, pos) : this->tex[0]->sample(u, v, pos));
}

image_texture::image_texture(const char* file) : image_texture()
{
	this->data = stbi_load(file, &this->width, &this->height, &this->channels, 0);
	bytes_scanline = this->width * this->channels;
}

image_texture::~image_texture()
{
	STBI_FREE(this->data);
}

vec3 image_texture::sample(double u, double v, const vec3& pos) const
{
	if (!this->data)
	{
		// image debug color
		return vec3(1.0, 0.0, 1.0);
	}

	u = clamp(u, 0.0, 1.0);
	v = 1.0 - clamp(v, 0.0, 1.0); // flip v to image top-to-bottom

	size_t i = std::min(static_cast<int>(u * this->width), this->width - 1);
	size_t j = std::min(static_cast<int>(v * this->height), this->height - 1);

	constexpr double color_scale = 1.0 / 255.0;
	auto pixel = this->data + j * this->bytes_scanline + i * this->channels;
	return vec3
	{
		pixel[0] * color_scale,
		pixel[1] * color_scale,
		pixel[2] * color_scale
	};
}