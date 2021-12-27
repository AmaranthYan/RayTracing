#include "medium.h"

bool uniform_medium::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	double t0 = t_min;
	double t1 = t0 - ray::epsilon;
	double inv_len = 1.0 / r.dir().length();

	double t_hit;
	bool once = true;

	hit_result h;

	while (this->boundary->hit(r, t1 + ray::epsilon, t_max, h))
	{
		t1 = h.frac;
		if (!vec3::is_back(r.dir(), h.norm))
		{
			t0 = h.frac;
			continue;
		}

		if (once)
		{
			t_hit = -this->inv_density * log(1.0 - drand()) * inv_len;
			once = false;
		}

		const double t_depth = t1 - t0;
		if (t_hit < t_depth)
		{
			res.frac = t0 + t_hit;
			res.pos = r.pos(res.frac);			
			res.mat = this->phase;
			return true;
		}

		t_hit -= t_depth;
	}
	
	return false;
}