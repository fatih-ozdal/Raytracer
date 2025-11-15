# 🌌 Making A Ray Tracer — Let There Be Rays

> **Date:** 2025-11-01   
> **Stage:** 1 - Basic Ray Casting, Reflection, Refraction, Smooth Shading, JSON and PLY parsing

---

## 🚀 Introduction

*Hello dear readers.*

I am currently taking an advanced ray tracing course, and we are tasked with building our own ray tracer through bi-weekly homework assignments. The JSON and PLY scene files used in this project were provided by our instructor, forming the basis of the scenes I render throughout this journey.

 I decided to document this journey as a multi-part blog series, where each part focuses on the progress made in one assignment. My goal is to share what I built, what I learned, the challenges I faced, and the results I achieved along the way.

## 🎯 Goals for This Stage

- Scene loading: JSON & PLY parsing
- Core ray tracing: camera setup, ray generation, intersections
- Level 1 shading: ambient, diffuse, specular
- Level 2 shading: reflections, refractions, smooth shading
- Output & performance: write PNGs and reduce render time

## 🧱 Scene Parser & Data Structures

The first part of the assignment required implementing my own scene parser and defining the data structures that would represent the scene. The given scene files (JSON and PLY) describe everything needed for rendering: cameras, lights, vertex data, materials, and objects such as triangles, spheres, planes, and meshes.

Below is a small excerpt from one of the JSON scene files provided to us (collapsed for readability):

---

<details>
  <summary>Click to view JSON example: cornellbox_recursive.json </summary>

```json
{
  "Scene": {
    "MaxRecursionDepth": "6",
    "BackgroundColor": "0 0 0",
    "Cameras": {
      "Camera": {
        "_id": "1",
        "Position": "0 0 20",
        "Gaze": "0 0 -1",
        "Up": "0 1 0",
        "NearPlane": "-10 10 -10 10",
        "NearDistance": "10",
        "ImageResolution": "800 800",
        "ImageName": "cornellbox_recursive.png"
      }
    },
    "Lights": {
      "AmbientLight": "20 20 20",
      "PointLight": {
        "_id": "1",
        "Position": "0 4 2",
        "Intensity": "350000 350000 350000"
      }
    },
    "Materials": {
      "Material": [
        {
          "_id": "1",
          "AmbientReflectance": "1 1 1",
          "DiffuseReflectance": "0.08 0.08 0.08",
          "SpecularReflectance": "0 0 0"
        },
        {
          "_id": "2",
          "AmbientReflectance": "1 0 0",
          "DiffuseReflectance": "0.1 0 0",
          "SpecularReflectance": "0 0 0"
        },
        {
          "_id": "3",
          "AmbientReflectance": "0 0 1",
          "DiffuseReflectance": "0 0 0.1",
          "SpecularReflectance": "0 0 0"
        },
        {
          "_id": "4",
          "AmbientReflectance": "1 1 1",
          "DiffuseReflectance": "0.08 0.08 0.01",
          "SpecularReflectance": "1 1 1",
          "PhongExponent": "300"
        },
        {
          "_id": "5",
          "_type": "conductor",
          "AmbientReflectance": "0 0 0",
          "DiffuseReflectance": "0 0 0",
          "SpecularReflectance": "0 0 0",
          "MirrorReflectance": "1 0.86 0.57",
          "RefractionIndex": "0.370",
          "AbsorptionIndex": "2.820"
        },
        {
          "_id": "6",
          "_type": "dielectric",
          "AmbientReflectance": "0 0 0",
          "DiffuseReflectance": "0 0 0",
          "SpecularReflectance": "0 0 0",
          "MirrorReflectance": "1 1 1",
          "AbsorptionCoefficient": "0.01 0.01 0.01",
          "RefractionIndex": "1.55"
        }
      ]
    },
    "VertexData": {
      "_data": "-10 -10 10 10 -10 10 10 10 10 -10 10 10 -10 -10 -10 10 -10 -10 10 10 -10 -10 10 -10 5 -6 1 -5 -6 -5",
      "_type": "xyz"
    },
    "Objects": {
      "Mesh": [
        {
          "_id": "1",
          "Material": "1",
          "Faces": {
            "_data": "1 2 6 6 5 1",
            "_type": "triangle"
          }
        },
        {
          "_id": "2",
          "Material": "1",
          "Faces": {
            "_data": "5 6 7 7 8 5",
            "_type": "triangle"
          }
        },
        {
          "_id": "3",
          "Material": "1",
          "Faces": {
            "_data": "7 3 4 4 8 7",
            "_type": "triangle"
          }
        },
        {
          "_id": "4",
          "Material": "2",
          "Faces": {
            "_data": "8 4 1 8 1 5",
            "_type": "triangle"
          }
        },
        {
          "_id": "5",
          "Material": "3",
          "Faces": {
            "_data": "2 3 7 2 7 6",
            "_type": "triangle"
          }
        }
      ],
      "Sphere": [
        {
          "_id": "1",
          "Material": "6",
          "Center": "9",
          "Radius": "4"
        },
        {
          "_id": "2",
          "Material": "5",
          "Center": "10",
          "Radius": "4"
        }
      ]
    }
  }
}

```
</details> 

---

While parsing the scene, I didn’t just store the raw values from the files — I also precomputed and stored some additional data to make ray tracing and shading more efficient later on:

- `face_normal` (in `Face` struct): 

    Computed once during parsing for triangles and mesh faces.  
If smooth shading is **not** used, this normal can be directly used for shading.

- **`plane_dist`** (in `Face` struct)  

  Used for plane intersection. Could be used for implementing z-buffer later on.   
  The plane equation term helps speed up intersection checks in `IntersectsPlane`.

- **`vertex_normal`** (in `Vertex` struct)  

  Accumulated during parsing if smooth shading is enabled.  
  Later used for interpolated shading using barycentric coordinates.

- **`u`, `v`, `w`** (in `Camera` struct)  

  These form the camera’s orthonormal basis vectors.  
  Since they depend only on camera parameters and not on rays or scene objects, I compute them once during parsing and store them for reuse.

By enriching the scene with these extra precomputed values, I reduced repeated calculations during rendering, which helped performance and kept the shading code cleaner.

Also some of the meshes in the scenes were not defined directly in the JSON file, but instead referenced an external PLY file containing the mesh data. A typical PLY file starts with a header describing the number of vertices and faces, followed by the actual binary data.  

Here’s an example of a PLY header:

---
<details> 
  <summary>Click to view PLY header example: lobster.ply </summary>

```ply
ply
format binary_little_endian 1.0
comment File exported by Artec Group 3D Scanning Solutions
comment www.artec-group.com
element vertex 745978
property float x
property float y
property float z
element face 1492052
property list uchar int vertex_indices
end_header
```
</details> 

---

When parsing a mesh, I read the vertex list and face list from the PLY file and append them to my existing vertex_data (in scene struct) and face arrays (in mesh struct). Since multiple meshes can appear in the same scene, I keep track of an offset (`vertex_base_index`) to correctly adjust the face vertex indices. This ensures that the newly loaded vertices do not overwrite or conflict with previously parsed ones.

## 🎯 Core Ray Tracing: Camera, Rays & Intersections

Once the scene was parsed, the next step was to generate rays from the camera and determine which objects they intersected. I used a standard pinhole camera model defined by the fields in the JSON file (`Position`, `Gaze`, `Up`, `NearPlane`, `NearDistance`, and `ImageResolution`). During parsing, I computed the camera’s orthonormal basis vectors **u, v, w** only once, since they do not change throughout rendering.

For each pixel, I mapped its center `(i + 0.5, j + 0.5)` onto the camera’s near plane to compute the ray direction. The ray origin is the camera position, and the direction is normalized before use.

To find what each ray hits, I implemented separate `intersect()` functions for spheres, planes, and triangles. Meshes are handled simply as collections of triangles loaded from PLY files. Each intersection function returns a `t` value representing the distance from the ray origin to the hit point. I loop through all objects, keep the smallest positive `t`, and store the result in a `HitRecord` containing the material id, intersection point, surface normal, and object type.

This provided the essential backbone of the ray tracer: the ability to shoot rays into the scene and identify what they hit—forming the basis for shading, reflections, and all later features.

## 💡 Shading: Adding Realism to Surfaces

With intersections working, I introduced shading to give the scene a sense of depth and realism. I first implemented basic lighting using ambient, diffuse, and specular shadings. Ambient adds a constant base light so that surfaces remain visible, diffuse provides the material lighting effect based on the angle between the surface normal and the light, and specular highlights simulate shiny reflections from light sources.

After establishing these basics, I added a second layer of effects for more realistic materials. Reflections were handled by shooting a secondary ray in the mirror direction. For transparent objects, I implemented refraction using Snell’s Law and Fresnel equations to blend reflection and refraction based on viewing angle. To simulate light entering and exiting a dielectric material, I cast two refraction rays: one for entering the object and one for exiting it. Finally, for meshes, I enabled smooth shading by interpolating vertex normals across triangles, which removed the faceted look and made curved surfaces appear smooth.

| Ambient + Diffuse  + Specular  | Also Reflection + Refraction |
|-----------------|--------------------------|
| ![cornellbox](my-outputs/cornellbox.png)   | ![cornellbox_recursice](my-outputs/cornellbox_recursive.png) |

| Without Smooth Shading | With Smooth Shading |
|-------------------------|---------------------|
| ![no-smooth](my-outputs/berserker.png)     | ![smooth](my-outputs/berserker_smooth.png)     |

## 🖼️ Writing the Output as PNG

To save the rendered images, I used **stb_image_write.h** (`stbi_write_png`).  
Each camera in the scene renders its own image, and after filling the framebuffer, I write it directly to a PNG file.

Here’s the core loop of my renderer:

```cpp
for (const Camera& camera : scene.cameras)
{
    const int width  = camera.width;
    const int height = camera.height;
    auto* image = new unsigned char[(size_t)width * height * 3];

    #pragma omp parallel for collapse(2) schedule(static)
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            Ray   ray   = ComputeRay(scene, camera, j, i);
            Vec3f color = ComputeColor(ray, scene, camera);

            const size_t idx = (static_cast<size_t>(i) * width + j) * 3;
            image[idx + 0] = (unsigned char)clampF(color.x, 0.0f, 255.0f);
            image[idx + 1] = (unsigned char)clampF(color.y, 0.0f, 255.0f);
            image[idx + 2] = (unsigned char)clampF(color.z, 0.0f, 255.0f);
        }
    }

    stbi_write_png(camera.image_name.c_str(), width, height, 3, image, width * 3);
    delete[] image;
}
```

## ⚡ Performance: Small Wins for Now

Before jumping into heavy optimizations, I implemented a few **small but meaningful** improvements to speed up rendering:

- **OpenMP Parallelization**  

    I parallelized the nested pixel loops with `#pragma omp parallel for collapse(2)`, allowing multiple rays to be computed simultaneously across CPU cores.

- **Precomputation of Invariants**  

  Values that don’t change per ray (such as the camera basis `u/v/w`, face normals, and plane equations) are computed once during parsing and reused.

- **Compiler Optimization Flags**  

  Compiling with `-O3` provided an immediate performance boost over unoptimized builds.

These improvements helped to reduce rendering time, but they only provide incremental gains.  

To achieve **significant** speed-ups—especially for scenes with many triangles or recursive rays—the next major step will be implementing an acceleration structure such as a **BVH (Bounding Volume Hierarchy)** or **k-d tree**. These reduce intersection checks from *O(N)* per ray to roughly *O(log N)*, which is essential for fast ray tracing.

## ⏱️ Render Times & Output Gallery

### 1) Cornell Box – With Reflection and Refraction 
**Render Time:** 0.173s  
This was the hardest image to get it right for me. Because the refraction of the dielectric sphere on the right  was wrong. At least I rendered it fast.

The left sphere looks tinted because it is a conductor not a mirror. It absorbs some of the light.

![cornell_rec_](my-outputs/cornellbox_recursive.png)
---

### 2) Spheres - With Reflection  
**Render Time:** 0.147s  
My program had max recursion depth equal to 6 mostly. I wonder what would happen if I set it to like 30 on a better image. I will maybe test it after implementing an acceleration structure.

![spheres-mirror](my-outputs/spheres_mirror.png)

---

### 3) Lobster - 256x256 version
**Render Time:** 38m 24.941s  
This image was given as 1024x1024 originally. But rendering it would take like 4.5 hours for my program. So i rendered the 256x256 version. I belive when I implement a acceleration structure we will be able to see the 1024x1024 version in minutes.

![lobster](my-outputs/lobster.png)

---

### 4) David  
**Render Time:** 6m 53.235s  
I don't know who this guy is but he is definitely someone.

![david](my-outputs/David.png)

---

### 5) Berserker - Smooth
**Render Time:** 5.600s   
Someone with an axe

![berserker](my-outputs/berserker_smooth.png)

---

### 6) White Dragon
**Render Time:** 45m 31.433s   
If you zoom in you can see there are black dots on the dragon. They are called shadow acnes, and caused by shadow rays hitting the same object they are sent from.    

I used *shadow_ray_epsilon* to shift the origin of my shadow rays to avoid this, but unfortunately it didn't work here. I will try to fix this bug until the end of my next assignment.

![chinese-dragon](my-outputs/chinese_dragon.png)

---

### 7) Gold Dragon
**Render Time:** 141m 7.433s    
Golden Legendary. Wasn't worth waiting 2 hours though

![other-dragon](my-outputs/other_dragon.png)

---

### 8) Bunny With A Plane
**Render Time:** 16.713s    
The plane under the rabbit is supposed to show a reflection. Reflections don't work for my planes. I couldn't figure out why.

![bunny-with-plane](my-outputs/bunny_with_plane.png)

---

### 9) Trex Head - 256x256 version
**Render Time:** 24m 29.455s   
This image was 1024x1024 too. Also shadow acnes are here as well

![trex](my-outputs/trex_smooth.png)

---

### 10) Ton Roosendaal
**Render Time:** 5m 53.131s  
Why is he smiling

![roosendaal](my-outputs/ton_Roosendaal_smooth.png)

---

### 11) Science Tree - Glass Version
**Render Time:** 13.251s  

![science-tree-glass](my-outputs/scienceTree_glass.png)

---

### 12) METU Teapots
**Render Time:** 2m 21.160s  

![teapot](my-outputs/UtahTeapotMugCENG.png)

---

### 13) Windmill - Smooth
**Render Time:** 16.001s   
[Song: The Windmills of Your Mind](https://www.youtube.com/watch?v=WEhS9Y9HYjU)

![windmill](my-outputs/windmill_smooth.png)

---

### 14) Tower - Smooth
**Render Time:** 13.251s  

![tower](my-outputs/tower_smooth.png)

---

### 15) Mountain - Smooth
**Render Time:** 22.948  

![poly](my-outputs/low_poly_scene_smooth.png)

---

### 16) Raven
**Render Time:** 11.722   
Raven

![raven](my-outputs/raven.png)

---

## ✅ Closing & What’s Next

This part took the project from “rays that hit things” to **actual images** with lighting, reflections, refractions (with Fresnel), and **smooth shading** on meshes. I also added a handful of small performance wins.

**Next up:** build an acceleration structure (**BVH** or **K-d Tree**) ,and support transformations given in scene.json files.
