#pragma once

#include "vec3.h"

struct mat3
{
	mat3() : row{} {}
	mat3(const vec3& r0, const vec3& r1, const vec3& r2) : row{ r0, r1, r2 } {}

	static __forceinline mat3 identity()
	{
		return mat3
		{
			{ 1.0, 0.0, 0.0 },
			{ 0.0, 1.0, 0.0 },
			{ 0.0, 0.0, 1.0 }
		};
	}

	__forceinline mat3 transpose() const
	{
		return mat3
		{
			{ row[0][0], row[1][0], row[2][0] },
			{ row[0][1], row[1][1], row[2][1] },
			{ row[0][2], row[1][2], row[2][2] }
		};
	}

	__forceinline double determinant() const
	{
		return vec3::dot(row[0], vec3::cross(row[1], row[2]));
	}

	__forceinline const mat3& operator+() const { return *this; };
	__forceinline mat3 operator-() const { return mat3(-row[0], -row[1], -row[2]); };
	__forceinline vec3 operator[](int i) const { return row[i]; };
	__forceinline vec3& operator[](int i) { return row[i]; };

	__forceinline mat3& operator+=(const mat3& m);
	__forceinline mat3& operator-=(const mat3& m);
	__forceinline mat3& operator*=(const mat3& m);

	vec3 row[3];
};

__forceinline mat3 operator*(double t, const mat3& m)
{
	return mat3
	{
		t * m[0],
		t * m[1],
		t * m[2]
	};
}

__forceinline mat3 operator*(const mat3& m, double t)
{
	return mat3
	{
		m[0] * t,
		m[1] * t,
		m[2] * t
	};
}

__forceinline mat3 operator/(const mat3& m, double t)
{
	double k = 1.0 / t;

	return mat3
	{
		m[0] * k,
		m[1] * k,
		m[2] * k
	};
}

__forceinline vec3 operator*(const mat3& m, const vec3& v)
{
	return vec3
	{
		vec3::dot(m[0], v),
		vec3::dot(m[1], v),
		vec3::dot(m[2], v)
	};
}

__forceinline mat3 operator*(const mat3& a, const mat3& b)
{
	const mat3 t = b.transpose();

	return mat3
	{
		t * a[0],
		t * a[1],
		t * a[2]
	};
}

__forceinline mat3& mat3::operator+=(const mat3& m)
{
	this->row[0] += m[0];
	this->row[1] += m[1];
	this->row[2] += m[2];
	return *this;
}

__forceinline mat3& mat3::operator-=(const mat3& m)
{
	this->row[0] -= m[0];
	this->row[1] -= m[1];
	this->row[2] -= m[2];
	return *this;
}

__forceinline mat3& mat3::operator*=(const mat3& m)
{
	mat3 mul = *this * m;
	this->row[0] = mul[0];
	this->row[1] = mul[1];
	this->row[2] = mul[2];
	return *this;
}

__forceinline mat3 operator+(const mat3& a, const mat3& b)
{
	return mat3
	{
		a[0] + b[0],
		a[1] + b[1],
		a[2] + b[2]
	};
}

__forceinline mat3 operator-(const mat3& a, const mat3& b)
{
	return mat3
	{
		a[0] - b[0],
		a[1] - b[1],
		a[2] - b[2]
	};
}