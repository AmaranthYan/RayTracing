# RayTracing
My personal project to implement a **Monte Carlo path tracer** in 1 month following Peter Shirley's ray tracing trilogy *Ray Tracing in One Weekend*, *Ray Tracing: The Next Week* and *Ray Tracing: The Rest of Your Life*.  

This project has covered all the base techniques presented in the books. In addition, I added a few advanced rendering features and corrections to the original code, which makes it a (slightly) advanced ray tracing renderer :)

## Demo Scene
Cornell box with cloud and a colored crystal bunny, 1024 ray samples per pixel
![DEMO](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/demo_scene_1024.png)

## Advanced Features
### Acceleration Structure
*  Fresnel-Schlick reflectance for metal
*  Beer-Lambert absorption for dielectric
*  optimized random functions  
The original method to generate random vectors in 3D space has about 50 percent
*  expendable threaded rendering and fixed-size thread buffer memory
*  Surface Area Heuristic(SAH) based BVH with visual debugging option
![BVH1](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_1.png)
![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/bvh_2.png)
*  non-uniform non-convex cloud volume with 3D Perlin-Worley noise
*  simple obj model loading and triangle mesh support
*  Glossy material using Cook-Torrance BRDF model



![BVH2](https://github.com/AmaranthYan/RayTracing/blob/main/output/final/glossy_1024.png)

TODO:THIS IS WIP
