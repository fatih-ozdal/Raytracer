#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "parser.h"
#include "HitRecord.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define EPS_INTERSECTION 1e-6f
#define EPS_PARALLEL 1e-8f
#define RAY_MISS_VALUE -1.0f

struct Ray {
    Vec3f origin;
    Vec3f direction;
    int depth;
};

Ray ComputeRay(Scene scene, Camera camera, int j, int i);
Vec3f ComputeColor(const Ray& ray, const Scene& scene, const Camera& camera);

bool FindClosestHit(const Ray& ray, const Scene& scene, const Camera& camera, /*out*/ HitRecord& closestHit);
float IntersectsMesh(const Ray& ray, const Mesh& mesh, const std::vector<Vertex>& vertex_data, const float& minT, /*out*/ Face& hitFace);
float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const std::vector<Vertex>& vertex_data, const float& minT);
float IntersectsPlane(const Ray& ray, const Vec3f& normal, const Vertex& point_on_plane, const float& minT);
float IntersectSphere(const Ray& ray, const Vertex& center, float radius, const float& minT);
Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius);

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit);
bool InShadow(Vec3f point, const PointLight& I, const Vec3f& n, float eps_shadow, Scene scene);
Vec3f ComputeDiffuseAndSpecular(const Vec3f& origin, const Material& material, const PointLight& light, 
    const Vec3f& point, const Vec3f& normal, const Vec3f& w0);

#endif // RAYTRACER_H
