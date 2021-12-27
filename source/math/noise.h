#pragma once

#include <array>
#include "vec3.h"

//#define PERLIN_QUINTIC

template<unsigned CAPACITY>
class perlin_noise
{
	static_assert(CAPACITY && !(CAPACITY & (CAPACITY - 1)), "capacity must be power of 2");

public:
	perlin_noise() : rand_unit{}, perm_x{}, perm_y{}, perm_z{}
	{
		for (int i = 0; i < CAPACITY; i++)
		{
			// this is less optimal, as it produces random vectors that are biased towards the diagonals of unit cube
			//rand_unit[i] = vec3::unit(vec3::random_cube(-1.0, 1.0));
			// use random unit vector instead
			rand_unit[i] = vec3::random_unit_vector();			
		}

		permute(perm_x);
		permute(perm_y);
		permute(perm_z);
	}

	double noise(const vec3& p) const
	{
		int ix = static_cast<int>(floor(p.x));
		int iy = static_cast<int>(floor(p.y));
		int iz = static_cast<int>(floor(p.z));

		vec3 grad[2][2][2];
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 2; k++)
				{
					int px = perm_x[(ix + i) & (CAPACITY - 1)];
					int py = perm_y[(iy + j) & (CAPACITY - 1)];
					int pz = perm_z[(iz + k) & (CAPACITY - 1)];
					grad[i][j][k] = rand_unit[px ^ py ^ pz];
				}
			}
		}

		vec3 t
		{
			p.x - floor(p.x),
			p.y - floor(p.y),
			p.z - floor(p.z)
		};

		return interpolation(grad, t);
	}

	double turbulence(vec3 p, int depth = 7) const
	{
		double sum = 0.0;
		double weight = 1.0;

		for (int i = 0; i < depth; i++)
		{
			sum += weight * noise(p);
			weight *= 0.5;
			p *= 2.0;
		}

		return abs(sum);
	}

private:
	// [min, max]
	__forceinline int irand(int min, int max)
	{
		std::random_device seed;
		std::uniform_int_distribution<int> dist(min, max);
		return dist(seed);
	}

	void permute(int axis[CAPACITY])
	{
		for (int i = 0; i < CAPACITY; i++)
		{
			axis[i] = i;
		}

		for (int i = CAPACITY - 1; i > 0; i--)
		{
			std::swap(axis[i], axis[irand(0, i)]);
		}
	}

	// Perlin(p) = sum(falloff(p) * Gradient(p))
	// falloff(p) = smoothstep(p.x) * smoothstep(p.y) * smoothstep(p.z)
	// Gradient(p) = dot(rand_vec(p), p);
	static double interpolation(vec3 grad[2][2][2], const vec3& t)
	{
#ifdef PERLIN_QUINTIC
		// Quintic Smoothing
		vec3 tt = t * t;
		vec3 ttt = tt * t;
		vec3 smooth = ttt * (vec3(6.0, 6.0, 6.0) * tt - vec3(15.0, 15.0, 15.0) * t + vec3(10.0, 10.0, 10.0));
#else
		// Hermitian Smoothing(aka. smoothstep)
		vec3 smooth = t * t * (vec3(3.0, 3.0, 3.0) - 2.0 * t);
#endif
		
		double sum = 0.0;
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 2; k++)
				{
					vec3 p(t.x - i, t.y - j, t.z - k);
					vec3 fall
					{
						i * smooth.x + (1.0 - i) * (1.0 - smooth.x),
						j * smooth.y + (1.0 - j) * (1.0 - smooth.y),
						k * smooth.z + (1.0 - k) * (1.0 - smooth.z),
					};
					sum += fall.x * fall.y * fall.z * vec3::dot(grad[i][j][k], p);
				}
			}
		}
		return sum;
	}

	std::array<vec3, CAPACITY> rand_unit;
	int perm_x[CAPACITY];
	int perm_y[CAPACITY];
	int perm_z[CAPACITY];
};

using perlin_256 = perlin_noise<256>;


template<unsigned CAPACITY>
class worley_noise
{
	static_assert(CAPACITY && !(CAPACITY& (CAPACITY - 1)), "capacity must be power of 2");

public:
	worley_noise() : rand_cube{}, perm_x{}, perm_y{}, perm_z{}
	{
		for (int i = 0; i < CAPACITY; i++)
		{
			rand_cube[i] = vec3::random_unit_cube();
		}

		permute(perm_x);
		permute(perm_y);
		permute(perm_z);
	}

	double noise(const vec3& p) const
	{
		int ix = static_cast<int>(floor(p.x));
		int iy = static_cast<int>(floor(p.y));
		int iz = static_cast<int>(floor(p.z));

		double min = std::numeric_limits<double>::infinity();
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				for (int k = -1; k <= 1; k++)
				{
					int px = perm_x[(ix + i) & (CAPACITY - 1)];
					int py = perm_y[(iy + j) & (CAPACITY - 1)];
					int pz = perm_z[(iz + k) & (CAPACITY - 1)];
					vec3 feat = rand_cube[px ^ py ^ pz];
					vec3 dist
					{
						p.x - static_cast<double>(ix + i) - feat.x,
						p.y - (iy + j) - feat.y,
						p.z - (iz + k) - feat.z,
					};
					min = std::min(min, dist.length_sqr());
				}
			}
		}

		return 1.0 - min;
	}

	double turbulence(vec3 p, int depth = 3) const
	{
		return 0.625 * noise(p) + 0.25 * noise(p * 2.0) + 0.125 * noise(p * 4.0);
	}

private:
	// [min, max]
	__forceinline int irand(int min, int max)
	{
		std::random_device seed;
		std::uniform_int_distribution<int> dist(min, max);
		return dist(seed);
	}

	void permute(int axis[CAPACITY])
	{
		for (int i = 0; i < CAPACITY; i++)
		{
			axis[i] = i;
		}

		for (int i = CAPACITY - 1; i > 0; i--)
		{
			std::swap(axis[i], axis[irand(0, i)]);
		}
	}

	std::array<vec3, CAPACITY> rand_cube;
	int perm_x[CAPACITY];
	int perm_y[CAPACITY];
	int perm_z[CAPACITY];
};

using worley_256 = worley_noise<256>;