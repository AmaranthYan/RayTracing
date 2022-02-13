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
The original SAH yeilds suboptimal BVH structure when there are sticklike objects along the dominant axis, the strict SAH calculates surface areas along all 3 axes to determine the best possible BVH division. Debug marco has also been added to allow visualization of the BVH structre.  

BVH of scene from the books, spheres|BVH of custom mesh, triangles
:-------------------------:|:-------------------------:
![BVH1](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_1.png)|![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_2.png)

### Materials TODO
*  Fresnel-Schlick reflectance for metal
*  Beer-Lambert absorption for dielectric
*  Glossy material using Cook-Torrance BRDF model  
Physically base 

Glossy gold bunny
:-------------------------:
![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/glossy_1024.png)

### Cloud Volume
*  Worley noise  
Worley noise are implemented in addition to Perlin noise.
*  Non-uniform arbitrary cloud volume using 3D noise  
Cloud volume is created with a mixture of Perlin and Worley noises and the ray sampling in non-uniform medium is done thanks to Woodcock tracking. I also extended the medium boundary code to accept non-convex shapes as boundaries.

### Renderer TODO
*  optimized random functions  
The original method to generate random vectors in 3D space has about 50 percent
*  expendable threaded rendering and fixed-size thread buffer memory
*  simple obj model loading and triangle mesh support

TODO:THIS IS WIP
