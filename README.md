# RayTracing
My personal project to implement a **Monte Carlo path tracer** in 1 month following Peter Shirley's ray tracing trilogy *Ray Tracing in One Weekend*, *Ray Tracing: The Next Week* and *Ray Tracing: The Rest of Your Life*.  

This project has covered all the base techniques presented in the books. In addition, I added a few advanced rendering features and corrections to the original code, which makes it a (slightly) advanced ray tracing renderer :)

## Demo Scene
Cornell box with volumetric clouds and a colored crystal bunny, 1024 ray samples per pixel
![DEMO](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/demo_scene_1024.png)

## Advanced Features
Some avanced rendering features are missing from the trilogy or being presented in an introductory way. In this project, I tried to implement a handful of them while reimplementing the others with more optimized/performant algorithms.

### Acceleration Structure
*  BVH using strict Surface Area Heuristic(SAH)  
The original SAH implementation yeilds suboptimal BVH structure when there are sticklike objects along the dominant axis, the strict SAH calculates surface areas along all 3 axes to determine the best possible BVH division. Debug marco has also been added to allow visualization of the BVH structre.  

BVH of scene from the books, spheres|BVH of custom mesh, triangles
:-:|:-:
![BVH1](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_1.png)|![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_2.png)

### Materials
*  Fresnel-Schlick reflectance for metal
*  Beer-Lambert absorption for dielectric  
Add ray absorption inside dielectric materials based on the Beer-Lambert law, absorption rate can be tuned on each channel of RGB to create colored dielectrics.
*  Glossy material using Cook-Torrance BRDF model  
Physically base glossy material, each part of the Cook-Torrance BRDF is chosen as follows:  
F - Fresnel-Schlick  
D - GGX Trowbridge-Reitz NDF  
G - Smith GGX Disney  
GGX sampling is used as importance sampling for this material.

Glossy gold bunny, 1024 samples
:-:
![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/glossy_1024.png)

### Cloud Volume
*  Worley noise  
Worley noise are implemented in addition to Perlin noise.
*  Non-uniform arbitrary cloud volume using 3D noise  
Cloud volume is created with a mixture of Perlin and Worley noises and the ray sampling in non-uniform medium is done thanks to Woodcock tracking. I also extended the medium boundary code to accept non-convex shapes as boundaries.

### Renderer
*  optimized random functions  
The original random 3D vector generation in the books adopted a loop and retry strategy, a loop-free method has been put in place to replace it.
*  expendable threaded rendering and fixed-size thread buffer memory  
A CPU worker thread pool designed for pixel-level rendering jobs that is scalable to maximize CPU core usage, with fixed yet adjustable memory consumption.
*  simple model loading and triangle mesh support  
A simple OBJ model file parser that supports limited OBJ data formats including vertices, normals and texture coordinates.
