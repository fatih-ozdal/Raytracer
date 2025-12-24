# 🌌 Making A Ray Tracer - Texture Mapping & Procedural Noise

> **Date:** 2025-12-24   
> **Stage:** 4 - Image Textures, Perlin Noise, UV Mapping

---

*Welcome back, dear readers.*

In the previous episode, I implemented multisampling and distribution ray tracing for realistic camera effects. This homework adds **texture mapping** - the ability to apply images and procedural patterns to surfaces.

I tried to implement everything according to spec. Some scenes rendered correctly but most didn't. These past 2 weeks I didn't have much time to work on my raytracer so I couldn't fix every bug but nevertheless I will show you my results.

## 🎯 Goals for This Stage

- **Image Textures**: Load images with stb_image, implement bilinear/nearest interpolation
- **Perlin Noise**: Implement Perlin noise from slides (16 gradients, no permutation table)
- **Checkerboard**: Simple procedural pattern
- **UV Mapping**: Parametric coordinates for spheres, barycentric interpolation for meshes
- **Decal Modes**: replace_kd, blend_kd, replace_ks, replace_all, replace_background, replace_normal, bump_normal

## ⏱️ Render Times & Output Gallery

For this part I will give my results and the expected ones side by side so you can compare if they are correct. I will put "-C" or "-W" if I think they are correct or wrong respectively.

Also most renders took less than a second. So I will skip writing their render time, I will give only the ones that are significantly long.

### 1) brickwall_with_normalmap

![brickwall_with_normalmap](my_outputs/brickwall_with_normalmap.png)

---

### 2) bump_mapping_transformed

![bump_mapping_transformed](my_outputs/bump_mapping_transformed.png)

---

### 3) cube_cushion

![cube_cushion](my_outputs/cube_cushion.png)

---

### 4) cube_perlin_bump

![cube_perlin_bump](my_outputs/cube_perlin_bump.png)

---

### 5) cube_perlin

![cube_perlin](my_outputs/cube_perlin.png)

---

### 6) cube_wall_normal

![cube_wall_normal](my_outputs/cube_wall_normal.png)

---

### 7) cube_wall

![cube_wall](my_outputs/cube_wall.png)

---

### 8) cube_waves

![cube_waves](my_outputs/cube_waves.png)

---

### 9) ellipsoids_texture

![ellipsoids_texture](my_outputs/ellipsoids_texture.png)

---

### 10) galactica_dynamic
**Render Time:** 2m 5.952s

![galactica_dynamic](my_outputs/galactica_dynamic.png)

---

### 11) galactica_static

![cube_perlin](my_outputs/galactica_static.png)

---

### 12) killeroo_bump_walls

![killeroo_bump_walls](my_outputs/killeroo_bump_walls.png)

---

### 13) plane_nearest

|  |  |
|-------------------------|-------------------------|
| ![plane_nearest](my_outputs/plane_nearest.png) | ![plane_nearest2](outputs/plane_nearest.png)|

### 14) plane_bilinear

![plane_bilinear](my_outputs/plane_bilinear.png)

---

### 15) plane_trilinear
Note: I didn't implement trilinear interpolation yet so this is just bilinear. 

![plane_trilinear](my_outputs/plane_trilinear.png)

---

### 16) sphere_nearest_bilinear

![sphere_nearest_bilinear](my_outputs/sphere_nearest_bilinear.png)

---

### 17) sphere_nobump_bump

![sphere_nobump_bump](my_outputs/sphere_nobump_bump.png)

---

### 18) sphere_nobump_justbump

![sphere_nobump_justbump](my_outputs/sphere_nobump_justbump.png)

---

### 19) sphere_normal

![sphere_normal](my_outputs/sphere_normal.png)

---

### 20) sphere_perlin

![sphere_perlin](my_outputs/sphere_perlin.png)

---

### 21) sphere_perlin_bump

![sphere_perlin_bump](my_outputs/sphere_perlin_bump.png)

---

### 22) sphere_perlin_scale

![sphere_perlin_scale](my_outputs/sphere_perlin_scale.png)

---

### 23) wood_box_all

![wood_box_all](my_outputs/wood_box_all.png)

---

### 24) wood_box_no_specular

![wood_box_no_specular](my_outputs/wood_box_no_specular.png)

---

### 25) wood_box

![wood_box](my_outputs/wood_box.png)

---