#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "parser.h"
#include "HitRecord.h"
#include "Bvh.h"

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


void BuildTopLevelBVH(const Scene& scene);
void MakeTopLevelPrimsArray(const Scene& scene);
void UpdateNodeBounds( uint32_t nodeIdx );
void Subdivide( uint32_t nodeIdx );

void BuildAllMeshBVHs(Scene& scene);
void BuildMeshBVH(const Scene& scene, size_t meshIdx);
void MakeMeshPrimsArray(const Mesh& mesh, const vector<Vertex>& vertex_data, MeshBVH& bvh);
void UpdateMeshNodeBounds(MeshBVH& bvh, uint32_t nodeIdx);
void SubdivideMesh(MeshBVH& bvh, uint32_t nodeIdx);

Ray ComputeRay(const Scene& scene, const Camera& camera, int j, int i) noexcept;
Vec3f ComputeColor(const Ray& ray, const Scene& scene, const Camera& camera);

bool FindClosestHit(const Ray& ray, const Scene& scene, const Camera& camera, HitRecord& closestHit) noexcept;

void IntersectTopBVH(const Ray& ray, const Scene& scene, float& minT, bool& has_intersected,
                     PrimKind& closestType, int& closestMatId, int& index,
                     Face& hitTriFace, Sphere& closestSphere, Triangle& closestTriangle,
                     Mesh& closestMesh, bool& closest_is_smooth,
                     float& bary_beta, float& bary_gamma) noexcept;

float IntersectMeshBVH(const Ray& ray, const Mesh& mesh, const Scene& scene, 
                       int bvhIndex, float minT, Face& hitFace, 
                       float& beta_out, float& gamma_out) noexcept;

float IntersectAABB(const Ray& ray, const AABB& box, float minT) noexcept;

float IntersectsMesh(const Ray& ray, const Mesh& mesh, const std::vector<Vertex>& vertex_data, float minT, /*out*/ Face& hitFace, /*out*/ float& beta_out, /*out*/ float& gamma_out) noexcept;
float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const std::vector<Vertex>& vertex_data, float minT, /*out*/ float& beta_out, /*out*/ float& gamma_out) noexcept;
float IntersectSphere(const Ray& ray, const Vertex& center, float radius, float minT) noexcept;
float IntersectsPlane(const Ray& ray, const Vec3f& normal, float plane_d, float minT) noexcept;
Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius) noexcept;

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit);
float Fresnel_Dielectric(float cosTheta, float cosPhi, float n1, float n2) noexcept;
float Fresnel_Conductor(float cosTheta, float refractionIndex, float absorptionIndex) noexcept;
bool InShadow(const Vec3f& point, const PointLight& I, const Vec3f& n, float eps_shadow, const Scene& scene) noexcept;
Vec3f ComputeDiffuseAndSpecular(const Vec3f& origin, const Material& material, const PointLight& light, 
    const Vec3f& point, const Vec3f& normal, const Vec3f& w0) noexcept;

#endif // RAYTRACER_H
