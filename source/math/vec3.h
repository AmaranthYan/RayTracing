#pragma once

#define _USE_MATH_DEFINES

#include <algorithm>
#include <math.h>
#include <random>
#include <thread>

struct vec3
{
	vec3() : v{} {}
	vec3(double a, double b, double c) : v{ a, b, c } {}

	__forceinline vec3& operator+=(const vec3& o);
	__forceinline vec3& operator-=(const vec3& o);
	__forceinline vec3& operator*=(const vec3& o);
	__forceinline vec3& operator/=(const vec3& o);
	__forceinline vec3& operator*=(double t);
	__forceinline vec3& operator/=(double t);

	__forceinline const vec3& operator+() const { return *this; };
	__forceinline vec3 operator-() const { return vec3(-x, -y, -z); };
	__forceinline double operator[](int i) const { return v[i]; };
	__forceinline double& operator[](int i) { return v[i]; };

	__forceinline double length() const { return ::sqrt(length_sqr()); }
	__forceinline double length_sqr() const { return x * x + y * y + z * z; }
	__forceinline void normalize() { *this *= 1.0 / length(); }
	__forceinline vec3 sqrt() const { return vec3(::sqrt(x), ::sqrt(y), ::sqrt(z)); }
	__forceinline bool near_zero() const
	{
		constexpr auto epsilon = 1e-8;
		return fabs(x) < epsilon && fabs(y) < epsilon && fabs(z) < epsilon;
	}

	// vec3 utility funcs
	static __forceinline double dot(const vec3& a, const vec3& b);
	static __forceinline vec3 cross(const vec3& a, const vec3& b);
	static __forceinline vec3 unit(const vec3& v);

	// random utility funcs
	static __forceinline vec3 random_unit_cube();
	static __forceinline vec3 random_cube(double min, double max);
	static __forceinline vec3 random_unit_vector();
	static __forceinline vec3 random_hemi_vector(const vec3& n);
	static __forceinline vec3 random_unit_sphere();
	static __forceinline vec3 random_hemi_sphere(const vec3& n);
	static __forceinline vec3 random_unit_disk();

	// min/max utility funcs
	static __forceinline vec3 vmin(const vec3& a, const vec3& b);
	static __forceinline vec3 vmax(const vec3& a, const vec3& b);

	static __forceinline bool is_back(const vec3& v, const vec3& n);

	union
	{
		struct
		{
			double x;
			double y;
			double z;
		};
		struct
		{
			double r;
			double g;
			double b;
		};
		double v[3];
	};
};

__forceinline vec3& vec3::operator+=(const vec3& o)
{
	this->x += o.x;
	this->y += o.y;
	this->z += o.z;
	return *this;
}

__forceinline vec3& vec3::operator-=(const vec3& o)
{
	this->x -= o.x;
	this->y -= o.y;
	this->z -= o.z;
	return *this;
}

__forceinline vec3& vec3::operator*=(const vec3& o)
{
	this->x *= o.x;
	this->y *= o.y;
	this->z *= o.z;
	return *this;
}

__forceinline vec3& vec3::operator/=(const vec3& o)
{
	this->x /= o.x;
	this->y /= o.y;
	this->z /= o.z;
	return *this;
}

__forceinline vec3& vec3::operator*=(double t)
{
	this->x *= t;
	this->y *= t;
	this->z *= t;
	return *this;
}

__forceinline vec3& vec3::operator/=(double t)
{
	double k = 1.0 / t;

	this->x *= k;
	this->y *= k;
	this->z *= k;
	return *this;
}

__forceinline vec3 operator+(const vec3& a, const vec3& b)
{
	return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__forceinline vec3 operator-(const vec3& a, const vec3& b)
{
	return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__forceinline vec3 operator*(const vec3& a, const vec3& b)
{
	return vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

__forceinline vec3 operator/(const vec3& a, const vec3& b)
{
	return vec3(a.x / b.x, a.y / b.y, a.z / b.z);
}

__forceinline vec3 operator*(const vec3& v, double t)
{
	return vec3(v.x * t, v.y * t, v.z * t);
}

__forceinline vec3 operator*(double t, const vec3& v)
{
	return vec3(t * v.x, t * v.y, t * v.z);
}

__forceinline vec3 operator/(const vec3& v, double t)
{
	return vec3(v.x / t, v.y / t, v.z / t);
}

__forceinline vec3 operator/(double t, const vec3& v)
{
	return vec3(t / v.x, t / v.y, t / v.z);
}

__forceinline double vec3::dot(const vec3& a, const vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

__forceinline vec3 vec3::cross(const vec3& a, const vec3& b)
{
	return vec3
	{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

__forceinline vec3 vec3::unit(const vec3& v)
{
	return v / v.length();
}

__forceinline double drand()
{
	thread_local auto thread_seed = std::hash<std::thread::id>()(std::this_thread::get_id());
	thread_local std::mt19937 rand(static_cast<std::mt19937::result_type>(thread_seed));
	static std::uniform_real_distribution<double> dist(0.0, 1.0);
	return dist(rand);
	//return double(rand()) / rand.max();
}

__forceinline double drand_normal()
{
	thread_local auto thread_seed = std::hash<std::thread::id>()(std::this_thread::get_id());
	thread_local std::mt19937 rand(static_cast<std::mt19937::result_type>(thread_seed));
	static std::normal_distribution<double> dist(0.0, 1.0);
	return dist(rand);
	//return double(rand()) / rand.max();
}


__forceinline double drand(double min, double max)
{
	return min + (max - min) * drand();
}

__forceinline vec3 vec3::random_unit_cube()
{
	return vec3(drand(), drand(), drand());
}

__forceinline vec3 vec3::random_cube(double min, double max)
{
	return vec3(drand(min, max), drand(min, max), drand(min, max));
}

__forceinline vec3 vec3::random_unit_vector()
{
	// !! this is wrong! random vector should have each axis normally distributed, not uniformly!
	//return vec3::unit(vec3::random_cube(-1.0, 1.0));

	// use cylinder surface mapping
	auto theta = 2.0 * M_PI * drand();
	auto z = drand(-1.0, 1.0);
	auto r = ::sqrt(1.0 - z * z);

	return vec3
	{
		r * cos(theta),
		r * sin(theta),
		z
	};
}

__forceinline vec3 vec3::random_hemi_vector(const vec3& n)
{
	auto u = vec3::random_unit_vector();
	return vec3::dot(u, n) > 0.0 ? u : -u;
}

__forceinline vec3 vec3::random_unit_sphere()
{
	double u = drand();
	return cbrt(u) * vec3::random_unit_vector();
}

__forceinline vec3 vec3::random_hemi_sphere(const vec3& n)
{
	auto u = vec3::random_unit_sphere();
	return vec3::dot(u, n) > 0.0 ? u : -u;
}

__forceinline vec3 vec3::random_unit_disk()
{
	vec3 v = 2.0 * vec3(drand(), drand(), 0) - vec3(1.0, 1.0, 0);
	double u = drand();
	return ::sqrt(u) * vec3::unit(v);
}

__forceinline vec3 vec3::vmin(const vec3& a, const vec3& b)
{
	return vec3
	{
		std::min(a.x, b.x),
		std::min(a.y, b.y),
		std::min(a.z, b.z)
	};
}
__forceinline vec3 vec3::vmax(const vec3& a, const vec3& b)
{
	return vec3
	{
		std::max(a.x, b.x),
		std::max(a.y, b.y),
		std::max(a.z, b.z)
	};
}

__forceinline bool vec3::is_back(const vec3& v, const vec3& n)
{
	return vec3::dot(v, n) > 0.0;
}