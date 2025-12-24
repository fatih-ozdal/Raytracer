# 🌌 Making A Ray Tracer - Texture Mapping & Procedural Noise

> **Date:** 2025-12-24   
> **Stage:** 4 - Image Textures, Perlin Noise, UV Mapping

---

*Welcome back, dear readers.*

In the previous episode, I implemented multisampling and distribution ray tracing for realistic camera effects. This homework adds **texture mapping** - the ability to apply images and procedural patterns to surfaces.

I tried to implement everything according to spec. Some scenes rendered correctly but most didn't. These past 2 weeks I didn't have much time to work on my raytracer so I couldn't fix every bug but nevertheless I will walk you through what I did and what I should do to fix the bugs in the future.

## 🎯 Goals for This Stage

- **Image Textures**: Load images with stb_image, implement bilinear/nearest interpolation
- **Perlin Noise**: Implement Perlin noise from slides (16 gradients, no permutation table)
- **Checkerboard**: Simple procedural pattern for testing
- **UV Mapping**: Parametric coordinates for spheres, barycentric interpolation for meshes
- **Decal Modes**: replace_kd, blend_kd, replace_ks, replace_all, replace_background, replace_normal, bump_normal

## ⏱️ Render Times & Output Gallery

For this part I will give the correct images and wrong ones seperately. Also most renders took less than a second. So I will skip writing their render time, I will give only the ones that are significantly long.

## Correct Renders:

### 1) Brickwall with Normalmap - C

![brickwall_with_normalmap](my_outputs/brickwall_with_normalmap.png)

---

### 2) bump_mapping_transformed - W

![bump_mapping_transformed](my_outputs/bump_mapping_transformed.png)

---

### 3) cube_cushion - W

![cube_cushion](my_outputs/cube_cushion.png)

---

### 4) cube_perlin_bump - W

![cube_perlin_bump](my_outputs/cube_perlin_bump.png)

---

### 5) cube_perlin - C

![cube_perlin](my_outputs/cube_perlin.png)

---

### 5) cube_wall_normal - C

![cube_wall_normal](my_outputs/cube_wall_normal.png)

---

### 5) cube_wall - c

![cube_wall](my_outputs/cube_wall.png)

---

### 5) cube_waves - w

![cube_waves](my_outputs/cube_waves.png)

---

### 5) ellipsoids_texture - c

![ellipsoids_texture](my_outputs/ellipsoids_texture.png)

---

### 5) galactica_dynamic
**Render Time:** 2m 5.952s

![galactica_dynamic](my_outputs/galactica_dynamic.png)

---

### 5) galactica_static

![cube_perlin](my_outputs/galactica_static.png)

---

### 5) killeroo_bump_walls

![killeroo_bump_walls](my_outputs/killeroo_bump_walls.png)

---

### 5) plane_nearest

![plane_nearest](my_outputs/plane_nearest.png)

---

### 5) plane_bilinear

![plane_bilinear](my_outputs/plane_bilinear.png)

---

### 5) plane_trilinear
Note: I didn't implement trilinear interpolation yet so this is just bilinear. 

![plane_trilinear](my_outputs/plane_trilinear.png)

---

### 5) sphere_nearest_bilinear

![sphere_nearest_bilinear](my_outputs/sphere_nearest_bilinear.png)

---

### 5) sphere_nearest_trilinear
Note: I didn't implement trilinear interpolation yet so this is just bilinear. 

![sphere_nearest_trilinear](my_outputs/sphere_nearest_trilinear.png)

---

### 5) sphere_nobump_bump

![sphere_nobump_bump](my_outputs/sphere_nobump_bump.png)

---

### 5) sphere_nobump_justbump

![sphere_nobump_justbump](my_outputs/sphere_nobump_justbump.png)

---

### 5) sphere_normal

![sphere_normal](my_outputs/sphere_normal.png)

---

### 5) sphere_perlin

![sphere_perlin](my_outputs/sphere_perlin.png)

---

### 5) sphere_perlin_bump

![sphere_perlin_bump](my_outputs/sphere_perlin_bump.png)

---

### 5) sphere_perlin_scale

![sphere_perlin_scale](my_outputs/sphere_perlin_scale.png)

---

### 5) wood_box_all

![wood_box_all](my_outputs/wood_box_all.png)

---

### 5) wood_box_no_specular

![wood_box_no_specular](my_outputs/wood_box_no_specular.png)

---

### 5) wood_box

![wood_box](my_outputs/wood_box.png)

---