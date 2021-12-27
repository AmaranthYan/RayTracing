#pragma once

#include <cassert>
#include <algorithm>
#include <unordered_map>
#include "../hittable/hittable.h"
#include "../material/material.h"

//#define DEBUG_BVH

#ifdef DEBUG_BVH
#define DEBUG_BVH_LEVEL 4
#endif

struct bvh : public hittable
{
private:
	using aabb_map = std::unordered_map<std::shared_ptr<hittable>, aabb>;

	// comparison should take times into account
	template<unsigned AXIS>
	struct axis_comparator
	{
		static_assert(AXIS < 3, "axis must be 0, 1 or 2");

		explicit axis_comparator(const std::shared_ptr<aabb_map>& m) : map(m) {}

		bool operator()(const std::shared_ptr<hittable>& a, const std::shared_ptr<hittable>& b) const
		{
			const auto& box0 = map->at(a);
			const auto& box1 = map->at(b);

			// calculate center along the axis
			auto c0 = box0.minimum[AXIS] + box0.maximum[AXIS];
			auto c1 = box1.minimum[AXIS] + box1.maximum[AXIS];
			return c0 < c1;
		}

		std::shared_ptr<aabb_map> map;
	};

	// buffer to avoid frequent allocation and accelerate bvh construction by caching aabb
	struct
	{
		bool _allocate(const std::vector<std::shared_ptr<hittable>>& list, double tm0, double tm1)
		{
			if (allocated)
			{
				return false;
			}

			const size_t count = list.size();

			sortable = list; // copy the hittable list
			time0 = tm0;
			time1 = tm1;

			left.resize(count);
			right.resize(count);

			// use ptr as hash key
			box_map = std::make_shared<aabb_map>(count);
			for (size_t i = 0; i < count; i++)
			{
				aabb bbox;
				bool valid = sortable[i]->bounding_box(time0, time1, bbox);
				assert(valid);

				box_map->emplace(sortable[i], bbox);
			}

			allocated = true;
			return true;
		}

		void _releases()
		{
			sortable.clear();
			time0 = time1 = 0;

			left.clear();
			right.clear();

			box_map.reset();

			allocated = false;
		}

		std::shared_ptr<aabb_map> box_map;
		std::vector<std::shared_ptr<hittable>> sortable;
		std::vector<double> left, right;
		double time0, time1;

		bool allocated = false;
	}
	static thread_local _constructor_buffer;

public:
	bvh() : child{}, box(), time0(), time1() {};
	bvh(const std::vector<std::shared_ptr<hittable>>& list, double tm0, double tm1);
	bvh(const std::vector<std::shared_ptr<hittable>>& list) : bvh(list, 0.0, 0.0) {};
	bvh(decltype(bvh::_constructor_buffer)& _buffer, size_t start, size_t end); // should be private, leave public for std::make_shared

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override
	{
		// the bvh bounding box is invalid if time interval is not fully in [time0, time1]
		assert(this->time0 <= tm0 && this->time1 >= tm1);

		bbox = this->box;
		return true;
	}

#ifdef DEBUG_BVH
	static void _debug_traverse(const std::shared_ptr<hittable>& node, int depth, std::vector<std::shared_ptr<bvh>>& traversed);
	static void _debug_tint(const std::shared_ptr<bvh>& root, int depth);
#endif

private:
	std::shared_ptr<hittable> child[2];
	aabb box;
	double time0, time1; // bvh is only valid inside time slice [time0, time1]

#ifdef DEBUG_BVH	
	std::shared_ptr<material> _debug_color;
#endif
};