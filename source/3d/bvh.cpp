#include "bvh.h"

thread_local decltype(bvh::_constructor_buffer) bvh::_constructor_buffer;

bvh::bvh(const std::vector<std::shared_ptr<hittable>>& list, double tm0, double tm1) : child{}, box(), time0(tm0), time1(tm1)
{
	if (list.empty())
	{
		return;
	}

	if (list.size() == 1)
	{
		this->child[0] = this->child[1] = list[0];
		this->child[0]->bounding_box(time0, time1, this->box);
		return;
	}

	bool allocate = bvh::_constructor_buffer._allocate(list, time0, time1);

	new (this) bvh(bvh::_constructor_buffer, 0, list.size());

	if (allocate)
	{
		bvh::_constructor_buffer._releases();
	}
}

// [start, end)
bvh::bvh(decltype(bvh::_constructor_buffer)& _buffer, size_t start, size_t end)
{
	assert(_buffer.allocated);

	this->time0 = _buffer.time0;
	this->time1 = _buffer.time1;

	auto size = end - start;
	size_t mid = start + 1;

	if (size > 2)
	{
		std::vector<std::shared_ptr<hittable>> temp = { _buffer.sortable.cbegin() + start, _buffer.sortable.cbegin() + end };

		double sah = std::numeric_limits<double>::max();

		// Surface Area Heuristic along the axis
		auto sort_axis = [&](auto cmp) // auto lambda param is a c++14 feature
		{
			std::sort(temp.begin(), temp.end(), cmp);

			aabb lbox = _buffer.box_map->at(temp[0]);
			aabb rbox = _buffer.box_map->at(temp[size - 1]);
			_buffer.left[0] = lbox.half_area();
			_buffer.right[size - 1] = rbox.half_area();

			for (size_t lidx = 1, ridx = size - 2; lidx < size - 1; lidx++, ridx--)
			{
				lbox = aabb::surrounding_box(lbox, _buffer.box_map->at(temp[lidx]));
				rbox = aabb::surrounding_box(rbox, _buffer.box_map->at(temp[ridx]));
				_buffer.left[lidx] = lbox.half_area();
				_buffer.right[ridx] = rbox.half_area();
			}

			double min = sah;
			size_t idx = 1;

			for (size_t i = 1; i < size; i++)
			{
				auto area = i * _buffer.left[i - 1] + (size - i) * _buffer.right[i];
				if (area < min)
				{
					min = area;
					idx = i;
				}
			}

			if (min < sah)
			{
				std::copy(temp.cbegin(), temp.cend(), _buffer.sortable.begin() + start);
				sah = min;
				mid = start + idx;
			}
		};

		// sort axis x, y, z respectively, find the best axis division with minimum SAH sum
		sort_axis(bvh::axis_comparator<0>(_buffer.box_map));
		sort_axis(bvh::axis_comparator<1>(_buffer.box_map));
		sort_axis(bvh::axis_comparator<2>(_buffer.box_map));
	}

	aabb box0;
	if (mid == start + 1)
	{
		this->child[0] = _buffer.sortable[start];
		box0 = _buffer.box_map->at(this->child[0]);
	}
	else
	{
		auto sub_bvh = std::make_shared<bvh>(_buffer, start, mid);
		this->child[0] = sub_bvh;
		box0 = sub_bvh->box;
	}

	aabb box1;
	if (mid == end - 1)
	{
		this->child[1] = _buffer.sortable[end - 1];
		box1 = _buffer.box_map->at(this->child[1]);
	}
	else
	{
		auto sub_bvh = std::make_shared<bvh>(_buffer, mid, end);
		this->child[1] = sub_bvh;
		box1 = sub_bvh->box;
	}

	this->box = aabb::surrounding_box(box0, box1);
}

bool bvh::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	// the bvh bounding box is invalid if ray time is not in [time0, time1]
	assert(this->time0 <= r.tm() && this->time1 >= r.tm());

	if (this->box.intersect_ray(r, t_min, t_max))
	{
		bool hit0 = this->child[0]->hit(r, t_min, t_max, res);
		bool hit1 = this->child[1]->hit(r, t_min, hit0 ? res.frac : t_max, res);

#ifdef DEBUG_BVH
		if ((hit0 || hit1) && this->_debug_color)
		{
			res.mat = this->_debug_color;
		}
#endif

		return hit0 || hit1;
	}
	return false;
}

#ifdef DEBUG_BVH
class _debug_unlit : public material
{
public:
	_debug_unlit(const vec3& a) :albedo(a) {}

	virtual bool scatter(const ray& r, const hit_result& hit, scatter_record& scatt) const override
	{
		return false;
	}

	virtual vec3 emit(const ray& r, const hit_result& hit) const override
	{
		return albedo;
	}

private:
	vec3 albedo;
};

// true for leaf node that at maximum depth or contains at least 1 non-bvh child
void bvh::_debug_traverse(const std::shared_ptr<hittable>& node, int depth, std::vector<std::shared_ptr<bvh>>& traversed)
{
	auto bvh_node = std::dynamic_pointer_cast<bvh>(node);
	if (bvh_node == nullptr)
	{
		return;
	}

	if (depth > 0)
	{
		bvh::_debug_traverse(bvh_node->child[0], depth - 1, traversed);
		bvh::_debug_traverse(bvh_node->child[1], depth - 1, traversed);
	}
	else
	{
		traversed.push_back(bvh_node);
	}
}

void bvh::_debug_tint(const std::shared_ptr<bvh>& root, int depth)
{
	std::vector<std::shared_ptr<bvh>> traversed;
	bvh::_debug_traverse(root, depth, traversed);

	const size_t count = traversed.size();
	if (count == 1)
	{
		traversed[0]->_debug_color = std::make_shared<_debug_unlit>(vec3(1.0, 0.0, 0.0));
	}
	else
	{
		for (size_t i = 0; i < count; i++)
		{
			auto t = 2.0 * i / (count - 1);
			auto c = abs(t - 1.0);
			vec3 col
			{
				(1.0 - t > 0 ? c : 0.0),
				1.0 - c,
				(t - 1.0 > 0 ? c : 0.0)
			};
			traversed[i]->_debug_color = std::make_shared<_debug_unlit>(col);
		}
	}
}
#endif