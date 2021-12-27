#pragma once

#include "hittable.h"

class mesh : public hittable
{
public:
	mesh() : vertices(), normals(), uvs(), faces(), mat() {}
	mesh(const char* file, const std::shared_ptr<material>& m);

	bool load_from_obj(const char* obj_file);

	virtual bool hit(const ray& r, double t_min, double t_max, hit_result& res) const override;
	virtual bool bounding_box(double tm0, double tm1, aabb& bbox) const override;

private:
	std::shared_ptr<material> mat;
	std::shared_ptr<std::vector<vec3>> vertices;
	std::shared_ptr<std::vector<vec3>> normals;
	std::shared_ptr<std::vector<triangle::text_coord>> uvs;
	std::shared_ptr<hittable> faces;
};