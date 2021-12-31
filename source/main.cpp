#define STB_IMAGE_WRITE_IMPLEMENTATION
#define _CRT_SECURE_NO_WARNINGS 1

#include <chrono>
#include <iostream>
#include "lib/stb_image_write.h"
#include "3d/bvh.h"
#include "camera.h"
#include "parallel/parallel_tracer.h"
#include "hittable/medium.h"
#include "hittable/mesh.h"
#include "math/pdf.h"
#include "material/glossy.h"

using std::shared_ptr;
using std::make_shared;

decltype(std::chrono::high_resolution_clock::now()) begin, end;
std::chrono::milliseconds delta;

perlin_256 perlin;
worley_256 worley;

constexpr unsigned cpu_cores = 8; // 16

double aspect_ratio = 1.0;
unsigned image_width = 1200;
unsigned image_height = static_cast<unsigned>(image_width / aspect_ratio);

stbi_uc* out; // output uint8 array

constexpr int samples_per_pixel = 256;
constexpr int max_depth = 50;
#define USE_BVH

vec3 origin, lookat;
const vec3 vup(0, 1, 0);
double vfov, focus_dist, aperture;
double tm[2] = {};

vec3 background(0.0, 0.0, 0.0);

vec3 color(const ray& r, const std::shared_ptr<hittable>& world, const std::shared_ptr<hittable>& lights, int depth)
{
	hit_result result;
	if (depth <= 0)
	{
		return vec3(0.0, 0.0, 0.0);
	}

	if (!world->hit(r, ray::epsilon, std::numeric_limits<double>::infinity(), result))
	{
		return background;
	}

	vec3 emitted = result.mat->emit(r, result);
	scatter_record record{};

	if (result.mat->scatter(r, result, record))
	{
		if (record.is_specular)
		{
			return emitted + record.spec_atten * color(record.spec_scatt, world, lights, depth - 1);
		}

		shared_ptr<pdf> pdf;
		if (lights)
		{
			auto light_pdf = make_shared<hittable_pdf>(result.pos, lights);
			pdf = make_shared<mix_pdf>(record.sample_pdf, light_pdf, record.sample_rate);
		}
		else
		{
			pdf = record.sample_pdf;
		}
		
		auto scatt = ray(result.pos, pdf->generate(r.tm()), r.tm());
		auto proba = pdf->value(scatt.dir(), r.tm());

		// avoid NaN caused by very low probability rays
		if (proba > 1e-8)
		{
			auto atten = result.mat->brdf_cos(r, result, scatt);
			// early out if BRDF yields 0
			if (atten.length_sqr() > 0.0)
			{
				return emitted + atten * color(scatt, world, lights, depth - 1) / proba;
			}
		}
	}
	return emitted;
}

void output(size_t idx, vec3 col, void* data)
{
	col = col.sqrt(); // gamma 2 correction

	stbi_uc* out = static_cast<stbi_uc*>(data) + idx * 3;
	out[0] = static_cast<stbi_uc>(clamp<double>(col.r, 0.0, 0.999) * 256);
	out[1] = static_cast<stbi_uc>(clamp<double>(col.g, 0.0, 0.999) * 256);
	out[2] = static_cast<stbi_uc>(clamp<double>(col.b, 0.0, 0.999) * 256);
}

hittable_list final_scene_part_1()
{
	hittable_list world;
	auto ground_material = std::make_shared<lambertian>(vec3(0.5, 0.5, 0.5));
	// checker material from book 2
	//ground_material = std::make_shared<lambertian>(std::make_shared<checker_texture>(vec3(0.2, 0.3, 0.1), vec3(0.9, 0.9, 0.9), vec3(10, 10, 10)));
	world.add_hittable(std::make_shared<sphere>(vec3(0, -1000, 0), 1000, ground_material));
	for (int a = -11; a < 11; a++) {
		for (int b = -11; b < 11; b++) {
			auto choose_mat = drand();
			vec3 center(a + 0.9 * drand(), 0.2, b + 0.9 * drand());
			if ((center - vec3(4, 0.2, 0)).length() > 0.9) {
				std::shared_ptr<material> sphere_material;
				if (choose_mat < 0.8) {
					// diffuse
					auto albedo = vec3::random_unit_cube() * vec3::random_unit_cube();
					sphere_material = std::make_shared<lambertian>(albedo);
					auto center2 = center + vec3(0, drand(0.0, 0.5), 0);
					world.add_hittable(std::make_shared<moving_sphere>(center, center2, 0.0, 1.0, 0.2, sphere_material));
				}
				else if (choose_mat < 0.95) {
					// metal
					auto albedo = vec3::random_cube(0.5, 1.0);
					auto fuzz = drand(0, 0.5);
					sphere_material = std::make_shared<metal>(albedo, fuzz);
					world.add_hittable(std::make_shared<sphere>(center, 0.2, sphere_material));
				}
				else {
					// glass
					sphere_material = std::make_shared<dielectric>(1.5);
					world.add_hittable(std::make_shared<sphere>(center, 0.2, sphere_material));
				}
			}
		}
	}
	auto material1 = std::make_shared<dielectric>(1.5);
	world.add_hittable(std::make_shared<sphere>(vec3(0, 1, 0), 1.0, material1));
	auto material2 = std::make_shared<lambertian>(vec3(0.4, 0.2, 0.1));
	world.add_hittable(std::make_shared<sphere>(vec3(-4, 1, 0), 1.0, material2));
	auto material3 = std::make_shared<metal>(vec3(0.7, 0.6, 0.5), 0.0);
	world.add_hittable(std::make_shared<sphere>(vec3(4, 1, 0), 1.0, material3));
	return world;
}

shared_ptr<hittable_list> final_scene_part_2() {
	hittable_list boxes1;
	auto ground = make_shared<lambertian>(vec3(0.48, 0.83, 0.53));
	const int boxes_per_side = 20;
	for (int i = 0; i < boxes_per_side; i++) {
		for (int j = 0; j < boxes_per_side; j++) {
			auto w = 100.0;
			auto x0 = -1000.0 + i * w;
			auto z0 = -1000.0 + j * w;
			auto y0 = 0.0;
			auto x1 = x0 + w;
			auto y1 = drand(1, 101);
			auto z1 = z0 + w;
			boxes1.add_hittable(make_shared<aligned_box>(vec3(x0, y0, z0), vec3(x1, y1, z1), ground));
		}
	}
	shared_ptr<hittable_list> objects = make_shared<hittable_list>();
	objects->add_hittable(make_shared<bvh>(boxes1.list, 0, 1));
	auto light = make_shared<diffuse_light>(vec3(7, 7, 7));
	objects->add_hittable(make_shared<zx_rect>(147, 412, 123, 423, 554, false, light));
	auto center1 = vec3(400, 400, 200);
	auto center2 = center1 + vec3(30, 0, 0);
	auto moving_sphere_material = make_shared<lambertian>(vec3(0.7, 0.3, 0.1));
	objects->add_hittable(make_shared<moving_sphere>(center1, center2, 0, 1, 50, moving_sphere_material));
	objects->add_hittable(make_shared<sphere>(vec3(260, 150, 45), 50, make_shared<dielectric>(1.5)));
	objects->add_hittable(make_shared<sphere>(
		vec3(0, 150, 145), 50, make_shared<metal>(vec3(0.8, 0.8, 0.9), 1.0)
		));
	auto boundary = make_shared<sphere>(vec3(360, 150, 145), 70, make_shared<dielectric>(1.5));
	objects->add_hittable(boundary);
	objects->add_hittable(make_shared<uniform_medium>(boundary, 0.2, vec3(0.2, 0.4, 0.9)));
	boundary = make_shared<sphere>(vec3(0, 0, 0), 5000, make_shared<dielectric>(1.5));
	objects->add_hittable(make_shared<uniform_medium>(boundary, .0001, vec3(1, 1, 1)));
	auto emat = make_shared<lambertian>(make_shared<image_texture>("assets/earthmap.jpg"));
	objects->add_hittable(make_shared<sphere>(vec3(400, 200, 400), 100, emat));
	auto pertext = make_shared<noise_texture<perlin_256>>(perlin, vec3(0, 0, 0.1), 10);
	objects->add_hittable(make_shared<sphere>(vec3(220, 280, 300), 80, make_shared<lambertian>(pertext)));
	hittable_list boxes2;
	auto white = make_shared<lambertian>(vec3(.73, .73, .73));
	int ns = 1000;
	for (int j = 0; j < ns; j++) {
		boxes2.add_hittable(make_shared<sphere>(vec3::random_cube(0, 165), 10, white));
	}
	objects->add_hittable(make_shared<translate>(
		make_shared<rotate>(
			make_shared<bvh>(boxes2.list, 0.0, 1.0), vec3(0, 1, 0), 15),
		vec3(-100, 270, 395)
		)
	);
	return objects;
}

void cornell_box(shared_ptr<hittable_list>& cornell, shared_ptr<hittable_list>& lights)
{
	cornell = make_shared<hittable_list>();
	lights = make_shared<hittable_list>();

	auto white = make_shared<lambertian>(vec3(0.73, 0.73, 0.73));
	auto red = make_shared<lambertian>(vec3(0.65, 0.05, 0.05));
	auto green = make_shared<lambertian>(vec3(0.12, 0.45, 0.15));
	
	auto light = make_shared<zx_rect>(227, 332, 213, 343, 554, false, make_shared<diffuse_light>(vec3(15, 15, 15)));
	cornell->add_hittable(light);

	cornell->add_hittable(make_shared<yz_rect>(0, 555, 0, 555, 555, false, green));
	cornell->add_hittable(make_shared<yz_rect>(0, 555, 0, 555, 0, true, red));
	cornell->add_hittable(make_shared<zx_rect>(0, 555, 0, 555, 0, true, white));
	cornell->add_hittable(make_shared<zx_rect>(0, 555, 0, 555, 555, false, white));
	cornell->add_hittable(make_shared<xy_rect>(0, 555, 0, 555, 555, false, white));

	lights->add_hittable(light);
}

void final_scene_part_3(shared_ptr<hittable_list>& world, shared_ptr<hittable_list>& lights)
{
	cornell_box(world, lights);

	auto white = make_shared<lambertian>(vec3(0.73, 0.73, 0.73));

	shared_ptr<material> aluminum = make_shared<metal>(vec3(0.8, 0.85, 0.88), 0.0);
	auto box0 = make_shared<aligned_box>(vec3(0, 0, 0), vec3(165, 330, 165), white);
	auto rot0 = make_shared<rotate>(box0, vec3(0, 1, 0), 15);
	auto trans0 = make_shared<translate>(rot0, vec3(265, 0, 295));
	world->add_hittable(trans0);


	auto glass = make_shared<dielectric>(1.5);
	auto ball = make_shared<sphere>(vec3(190, 90, 190), 90, glass);
	world->add_hittable(ball);
	lights->add_hittable(ball);
}

void sah_bvh(shared_ptr<hittable_list>& world, shared_ptr<hittable_list>& lights)
{
	cornell_box(world, lights);
	
#ifdef DEBUG_BVH
	shared_ptr<hittable> m = make_shared<mesh>("assets/bunny.obj", nullptr);
	m = make_shared<rotate>(m, vec3(0, 1, 0), 180);
	m = make_shared<scale>(m, 120.0);
	m = make_shared<translate>(m, vec3(225, 36, 275));
	world->add_hittable(m);
#endif // DEBUG_BVH
}

void glossy_material(shared_ptr<hittable_list>& world, shared_ptr<hittable_list>& lights)
{
	cornell_box(world, lights);

	/*
	shared_ptr<hittable> m = make_shared<mesh>("assets/bunny.obj", nullptr);
	m = make_shared<rotate>(m, vec3(0, 1, 0), 180);
	m = make_shared<scale>(m, 120.0);
	m = make_shared<translate>(m, vec3(225, 36, 275));
	world->add_hittable(m);
	*/

	auto glossy00 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.0, 0.0, 1.5);
	auto glossy01 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.0, 0.5, 1.5);
	auto glossy02 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.0, 1.0, 1.5);

	auto glossy10 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.5, 0.0, 1.5);
	auto glossy11 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.5, 0.5, 1.5);
	auto glossy12 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.5, 1.0, 1.5);

	auto glossy20 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.0, 1.0, 2.5);
	auto glossy21 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 0.5, 1.0, 2.5);
	auto glossy22 = make_shared<glossy>(vec3(0.8, 0.8, 0.8), 1.0, 1.0, 2.5);
	
	shared_ptr<hittable> sphere00 = make_shared<sphere>(vec3(125, 50, 275), 50, glossy00);
	shared_ptr<hittable> sphere01 = make_shared<sphere>(vec3(275, 50, 275), 50, glossy01);
	shared_ptr<hittable> sphere02 = make_shared<sphere>(vec3(425, 50, 275), 50, glossy02);

	shared_ptr<hittable> sphere10 = make_shared<sphere>(vec3(125, 50, 425), 50, glossy20);
	shared_ptr<hittable> sphere11 = make_shared<sphere>(vec3(275, 50, 425), 50, glossy21);
	shared_ptr<hittable> sphere12 = make_shared<sphere>(vec3(425, 50, 425), 50, glossy22);

	//world->add_hittable(sphere00);
	//world->add_hittable(sphere01);
	//world->add_hittable(sphere02);

	world->add_hittable(sphere10);
	world->add_hittable(sphere11);
	world->add_hittable(sphere12);
}

void demo_scene(shared_ptr<hittable_list>& world, shared_ptr<hittable_list>& lights)
{
	cornell_box(world, lights);	
	
	// cloud block
	shared_ptr<hittable> block = make_shared<aligned_box>(vec3(0, 260, 0), vec3(260, 420, 260), nullptr);
	block = make_shared<translate>(block, vec3(225, 0, 225));
	//objects.add_hittable(make_shared<noise_medium<decltype(perlin)>>(box1, 0.01, vec3(0, 0, 0), perlin, 0, 0.1));
	auto cloud_block = make_shared<cloud<decltype(perlin), decltype(worley)>>(block, 0.08, vec3(1.0, 1.0, 1.0), perlin, worley, 1.0 / 128.0, 0.81);
	cloud_block->set_height_fading(280, 400, 20); // fading based on height
	world->add_hittable(cloud_block);
	// importance sample the block box
	lights->add_hittable(block);

	// crystal bunny
	auto crystal = std::make_shared<dielectric>(1.5, vec3(0.028, 0.014, 0.01));
	shared_ptr<hittable> bunny = make_shared<mesh>("assets/bunny.obj", crystal);
	bunny = make_shared<rotate>(bunny, vec3(0, 1, 0), 162);
	bunny = make_shared<scale>(bunny, 80.0);
	bunny = make_shared<translate>(bunny, vec3(160, 24, 160));
	world->add_hittable(bunny);

	// use a central sphere to importance sample the bunny mesh
	aabb bbox;
	bunny->bounding_box(0.0, 0.0, bbox);
	auto center = 0.5 * (bbox.minimum + bbox.maximum);
	auto central_sphere = make_shared<sphere>(center, center.y); // the sphere must not intersect with the ground plane (y = 0)
	lights->add_hittable(central_sphere);
}

/*
*  beyond the scope of the RayTracing trilogy, we have:
*  Fresnel-Schlick reflectance for metal
*  Beer-Lambert absorption for dielectric
*  optimized random functions
*  expendable threaded rendering and fixed-size thread buffer memory
*  Surface Area Heuristic(SAH) based BVH with visual debugging option
*  non-uniform non-convex cloud volume with 3D Perlin-Worley noise
*  simple obj model loading and triangle mesh support
*  TODO Glossy material using Cook-Torrance BRDF model
* 
*  abandoned ideas:
*  SIMD ray-AABB intersection(MSVC complier auto-vectorization is doing an excellent job, thought profiling shows hand-written SSEs run faster in Debug)
*/

int main()
{
	std::shared_ptr<hittable> scene;
	std::shared_ptr<hittable_list> world;
	std::shared_ptr<hittable_list> lights;

	switch (4)
	{
	case 0: // Ray Tracing in One Weekend
		aspect_ratio = 3.0 / 2.0;
		image_width = 1200;
		image_height = static_cast<int>(image_width / aspect_ratio);

		world = std::make_shared<hittable_list>(final_scene_part_1());
		lights = nullptr;

#ifdef USE_BVH
		begin = std::chrono::high_resolution_clock::now();

		scene = std::make_shared<bvh>(world->list, tm[0], tm[1]);
#ifdef DEBUG_BVH
		bvh::_debug_tint(std::dynamic_pointer_cast<bvh>(world), DEBUG_BVH_LEVEL);
#endif

		end = std::chrono::high_resolution_clock::now();
		delta = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
		std::cout << "bvh construct " << delta.count() / 1000.0f << std::endl;
#else
		scene = world;
#endif // USE_BVH

		origin = vec3(13, 2, 3);
		lookat = vec3(0, 0, 0);
		vfov = 20.0;
		focus_dist = 10.0;
		aperture = 0.1;

		tm[0] = 0.0;
		tm[1] = 1.0;

		background = vec3(0.7, 0.8, 1.0);
		// note that the actual background in book 1 depends on ray direction, use solid color here for simplicity		
		//double t = 0.5 * (vec3::unit(r.dir()).y + 1.0);
		//background = (1 - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);		
		break;
	
	case 1: // Ray Tracing: The Next Week
		aspect_ratio = 1.0;
		image_width = 800;
		image_height = static_cast<int>(image_width / aspect_ratio);

		scene = world = final_scene_part_2();
		lights = nullptr;

		origin = vec3(478, 278, -600);
		lookat = vec3(278, 278, 0);
		vfov = 40.0;
		focus_dist = 10.0;
		aperture = 0.0;

		tm[0] = 0.0;
		tm[1] = 1.0;
		break;

	case 2: // Ray Tracing: The Rest of Your Life
		aspect_ratio = 1.0;
		image_width = 600;
		image_height = static_cast<int>(image_width / aspect_ratio);

		final_scene_part_3(world, lights);
		scene = world;

		origin = vec3(278, 278, -800);
		lookat = vec3(278, 278, 0);
		vfov = 40.0;
		focus_dist = 10.0;
		aperture = 0.0;

		tm[0] = 0.0;
		tm[1] = 0.0;
		break;

	case 3: // SAH-BVH Scene
		sah_bvh(world, lights);
		scene = world;

		origin = vec3(278, 278, -800);
		lookat = vec3(278, 278, 0);
		vfov = 40.0;
		focus_dist = 10.0;
		aperture = 0.0;

		tm[0] = 0.0;
		tm[1] = 0.0;
		break;

	case 4: // Glossy Objects Scene
		glossy_material(world, lights);
		scene = world;

		origin = vec3(278, 278, -800);
		lookat = vec3(278, 278, 0);
		vfov = 40.0;
		focus_dist = 10.0;
		aperture = 0.0;

		tm[0] = 0.0;
		tm[1] = 0.0;
		break;

	case 5: // Demo Scene
		demo_scene(world, lights);
		scene = world;

		origin = vec3(278, 278, -800);
		lookat = vec3(278, 278, 0);
		vfov = 40.0;
		focus_dist = 10.0;
		aperture = 0.0;

		tm[0] = 0.0;
		tm[1] = 0.0;
		break;

	}

	out = static_cast<stbi_uc*>(malloc(image_width * image_height * 3 * sizeof(stbi_uc)));

	camera cam(origin, lookat, vup, vfov, aspect_ratio, aperture, focus_dist);

	{
		begin = std::chrono::high_resolution_clock::now();

		auto tracer = std::make_shared<parallel_tracer<cpu_cores>>(color, output);
		tracer->render<samples_per_pixel, max_depth>(image_width, image_height, cam, scene, lights, tm[0], tm[1], out);

		end = std::chrono::high_resolution_clock::now();
		delta = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
		std::cout << "render and merge " << delta.count() / 1000.0f << std::endl;
	}
	
	char output_file[64];
	std::sprintf(output_file, "output/threaded_output_%d_%d_%d.png", image_width, image_height, samples_per_pixel);

	{
		begin = std::chrono::high_resolution_clock::now();

		stbi_write_png(output_file, image_width, image_height, 3, out, image_width * 3);

		end = std::chrono::high_resolution_clock::now();
		delta = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
		std::cout << "output " << delta.count() / 1000.0f << std::endl;
	}

	free(out);
}