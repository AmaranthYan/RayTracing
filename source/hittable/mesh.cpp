#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream> 
#include "mesh.h"
#include "../3d/bvh.h"

mesh::mesh(const char* file, const std::shared_ptr<material>& m) : vertices(std::make_shared<std::vector<vec3>>()), normals(std::make_shared<std::vector<vec3>>()), uvs(std::make_shared<std::vector<triangle::text_coord>>()), faces(), mat(m)
{
	mesh::load_from_obj(file);
}

// simple .obj parser, support only v/vt/vn/f
bool mesh::load_from_obj(const char* obj_file)
{
	this->vertices->clear();
	this->normals->clear();
	this->uvs->clear();
	this->faces.reset();

	if (strcmp(strrchr(obj_file, '\0') - 4, ".obj")) // not .obj file
	{
		std::cerr << obj_file << " is not an obj file" << std::endl;
		return false;
	}

	std::ifstream fs(obj_file, std::ios::in);
	if (!fs)
	{
		std::cerr << "failed to load obj file " << obj_file << std::endl;
		return false;
	}

	std::vector<std::shared_ptr<hittable>> tri_list;

	std::string ln;
	while (std::getline(fs, ln))
	{
		auto type = ln.substr(0, 2);
		if (type == "v ")
		{
			std::istringstream ss(ln.substr(2));
			double x, y, z;
			ss >> x >> y >> z;
			this->vertices->emplace_back(x, y, z);
		}
		else if (type == "vt")
		{
			std::istringstream ss(ln.substr(2));
			double u, v;
			ss >> u >> v;
			this->uvs->emplace_back(u, v);
		}
		else if (type == "vn")
		{
			std::istringstream ss(ln.substr(2));
			double x, y, z;
			ss >> x >> y >> z;
			this->normals->emplace_back(x, y, z);
		}		
		else if (type == "f ")
		{
			std::istringstream ss(ln.substr(2));
			std::string data;

			int idx_v[3]{ -1, -1, -1 };
			int idx_vt[3]{ -1, -1, -1 };
			int idx_vn[3]{ -1, -1, -1 };

			bool has_uv = false;
			bool has_norm = false;

			for (int i = 0; i < 3; i++)
			{
				// "v/vt", "v/vt/vn", "v//vn" are all valid face formats
				ss >> data;

				auto end_v = data.find('/');
				idx_v[i] = std::stoi(data.substr(0, end_v)) - 1;

				if (end_v != std::string::npos)
				{
					auto end_vt = data.find('/', end_v + 1);

					auto str_vt = data.substr(end_v + 1, end_vt - end_v - 1);
					if (!str_vt.empty())
					{
						idx_vt[i] = std::stoi(str_vt) - 1;
						has_uv = true;
					}					

					if (end_vt != std::string::npos)
					{
						idx_vn[i] = std::stoi(data.substr(end_vt + 1)) - 1;
						has_norm = true;
					}
				}
			}
			
			if (has_uv && has_norm)
			{
				tri_list.emplace_back(std::make_shared<triangle>(this->vertices, idx_v, this->normals, idx_vn, this->uvs, idx_vt, this->mat));
			}
			else if (has_uv)
			{
				tri_list.emplace_back(std::make_shared<triangle>(this->vertices, idx_v, this->uvs, idx_vt, this->mat));
			}
			else if (has_norm)
			{
				tri_list.emplace_back(std::make_shared<triangle>(this->vertices, idx_v, this->normals, idx_vn, this->mat));
			}
			else
			{
				tri_list.emplace_back(std::make_shared<triangle>(this->vertices, idx_v, this->mat));
			}
		}
		else
		{
			continue;
		}
	}

	this->faces = std::make_shared<bvh>(tri_list);
#ifdef DEBUG_BVH
	bvh::_debug_tint(std::dynamic_pointer_cast<bvh>(this->faces), DEBUG_BVH_LEVEL);
#endif
	return true;
}

bool mesh::hit(const ray& r, double t_min, double t_max, hit_result& res) const
{
	return this->faces->hit(r, t_min, t_max, res);
}

bool mesh::bounding_box(double tm0, double tm1, aabb& bbox) const
{
	return this->faces->bounding_box(tm0, tm1, bbox);
}