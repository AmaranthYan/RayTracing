#pragma once

#include <array>
#include <atomic>
#include "camera.h"
#include "../hittable/hittable.h"

template<unsigned THREADS, unsigned BUFFER = 32768>
class parallel_tracer
{
public:
	using ray_shader = vec3(*)(const ray& r, const std::shared_ptr<hittable>& world, const std::shared_ptr<hittable>& lights, int depth);
	using data_writer = void(*)(size_t pixel, vec3 color, void* data);

	parallel_tracer(ray_shader trace_func, data_writer output_func) : batch_buffer{}, process_buffer{}, done_buffer{}, trace(trace_func), output(output_func) {}

	template<unsigned SAMPLES, unsigned DEPTH>
	void render(unsigned width, unsigned height, const camera& cam, const std::shared_ptr<hittable> world, const std::shared_ptr<hittable>& lights, double time0, double time1, void* out);

private:
	std::array<std::atomic<unsigned>, BUFFER> batch_buffer;
	std::array<std::atomic<unsigned>, BUFFER> process_buffer;
	std::array<std::atomic<unsigned>, BUFFER> done_buffer;
	vec3 merge_buffer[THREADS * BUFFER]; // shared color buffer that all render threads write to

	ray_shader trace;
	data_writer output;
};


template<unsigned THREADS, unsigned BUFFER>
template<unsigned SAMPLES, unsigned DEPTH>
void parallel_tracer<THREADS, BUFFER>::render(unsigned width, unsigned height, const camera& cam, const std::shared_ptr<hittable> world, const std::shared_ptr<hittable>& lights, double time0, double time1, void* out)
{
	const unsigned data_size = width * height;
	constexpr unsigned samples_per_thread = SAMPLES / THREADS;
	
	std::atomic<unsigned> completed(0);

	auto render_pixel = [&]()
	{
		unsigned finished = 0;
		while (finished < BUFFER)
		{
			finished = 0;
			for (int i = 0; i < BUFFER; i++)
			{
				auto slot = process_buffer[i].load(std::memory_order::memory_order_acquire);

				while (slot < THREADS && !process_buffer[i].compare_exchange_weak(slot, slot + 1, std::memory_order::memory_order_release));

				if (slot < THREADS)
				{
					auto k = batch_buffer[i].load(std::memory_order::memory_order_acquire);
					auto index = k * BUFFER + i;

					assert(index < data_size);
					
					auto w = index % width;
					auto h = height - 1 - index / width;

					// render pixel samples
					vec3 col(0, 0, 0);
					for (unsigned s = 0; s < samples_per_thread; s++)
					{
						double u = double(w + drand()) / width;
						double v = double(h + drand()) / height;
						ray r = cam.get_ray(u, v, time0, time1);
						col += this->trace(r, world, lights, DEPTH);
					}

					// remove NaN components
					if (col.r != col.r) { col.r = 0.0; }
					if (col.g != col.g) { col.g = 0.0; }
					if (col.b != col.b) { col.b = 0.0; }

					col /= samples_per_thread;
					merge_buffer[i * THREADS + slot] = col;

					done_buffer[i].fetch_add(1, std::memory_order::memory_order_acq_rel);
				}
				else if (slot == UINT_MAX)
				{
					finished++;
				}
			}
		}
	};

	auto average_color = [&]()
	{
		vec3 color_buffer[THREADS];

		unsigned c = 0;
		while (c < data_size)
		{
			for (int i = 0; i < BUFFER; i++)
			{
				auto finish = done_buffer[i].load(std::memory_order::memory_order_acquire);
				if (finish < THREADS)
				{
					continue;
				}
				assert(finish == THREADS);

				auto k = batch_buffer[i].fetch_add(1, std::memory_order_consume);
				auto index = k * BUFFER + i;

				memcpy_s(color_buffer, sizeof(color_buffer), merge_buffer + i * static_cast<uintptr_t>(THREADS), THREADS * sizeof(vec3));
				done_buffer[i].store(0, std::memory_order_relaxed);
				process_buffer[i].store(index + BUFFER < data_size ? 0 : UINT_MAX, std::memory_order_release); // release the merge buffer block to render threads

				assert(index < data_size);

				vec3 col(0, 0, 0);
				for (int j = 0; j < THREADS; j++)
				{
					col += color_buffer[j];
				}
				col /= THREADS;

				// color in linear space
				this->output(index, col, out);

				c = completed.fetch_add(1, std::memory_order_release) + 1;
			}
		}
	};

	auto print_progress = [&]()
	{
		char percent[8];

		unsigned c = 0;
		while (c < data_size)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));

			c = completed.load(std::memory_order_consume);
			sprintf(percent, "%.2f", 100.f * c / data_size);
			std::cout << "rendering progress " << percent << "%" << std::endl;
		}
	};

	std::thread reporter(print_progress);
	std::thread renderer[THREADS];
	for (int i = 0; i < THREADS; i++)
	{
		renderer[i] = std::thread(render_pixel);
	}
	std::thread merger(average_color);

	merger.join();
	for (int i = 0; i < THREADS; i++)
	{
		renderer[i].join();
	}
	reporter.join();
}