#include "raytracer.h"

// For debugging
#include "DebugBvh.h"
#include "Timer.h"
#include <sstream>

// NanoVDB Includes
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/GridHandle.h>

using std::vector;

// ================= GLOBALS =================

vector<TopPrim> topPrims;
vector<uint32_t> topPrimIdx;
vector<BVHNode> topBvhNodes; 
uint32_t rootNodeIdx = 0, nodesUsed = 1;

bool isCorrectPixel = false;
int correct_j = 326;
int correct_i = 228;

bool backfaceCullingEnabled = true;

// ================= MAIN =================

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: %s <scene.json>\n", argv[0]);
        return -1;
    }
    
    parser p;
    string jsonFile = argv[1];
    Scene scene = p.loadFromJson(jsonFile);

    // Build BVHs - mesh BVHs first, then top-level
    BuildAllMeshBVHs(scene);
    BuildTopLevelBVH(scene);
    
    for (const Camera& camera : scene.cameras)
    {
        const int width  = camera.image_width;
        const int height = camera.image_height;
        auto* image = new unsigned char[(size_t) width * height * 3];

        const int num_samples = camera.num_samples;
        const int samples_per_side = camera.samples_per_side;
        const float inv_num_samples = 1.0f / num_samples;

        #pragma omp parallel for collapse(2) schedule(static)
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {

                if ( i == correct_i && j == correct_j )
                    isCorrectPixel = true;
                else
                    isCorrectPixel = false;

                std::mt19937 rng(i * width + j);
                std::uniform_real_distribution<float> dist(0.0f, 1.0f);

                std::vector<int> shuffle_array(num_samples);
                for (int idx = 0; idx < num_samples; ++idx) {
                    shuffle_array[idx] = idx;
                }
                std::shuffle(shuffle_array.begin(), shuffle_array.end(), rng);

                Vec3f color_sum(0.0f, 0.0f, 0.0f);

                for (int s = 0; s < num_samples; ++s) {
                    int sx = s % samples_per_side;
                    int sy = s / samples_per_side;

                    float jitter_x = (sx + dist(rng)) / samples_per_side;
                    float jitter_y = (sy + dist(rng)) / samples_per_side;

                    if (num_samples == 1) {
                        jitter_x = 0.5f;
                        jitter_y = 0.5f;
                    }
                    
                    int aperture_idx = shuffle_array[s];
                    int ax = aperture_idx % samples_per_side;
                    int ay = aperture_idx / samples_per_side;
                    float aperture_u = (ax + dist(rng)) / samples_per_side;  // [0, 1)
                    float aperture_v = (ay + dist(rng)) / samples_per_side;  // [0, 1)

                    if (num_samples == 1) {
                        aperture_u = 0.5f;
                        aperture_v = 0.5f;
                    }
                    
                    // Time for motion blur
                    float time = dist(rng);
                    
                    Ray ray = ComputeRay(scene, camera, j, i, 
                                    jitter_x, jitter_y, 
                                    aperture_u, aperture_v, 
                                    time);
                    Vec3f sample_color = ComputeColor(ray, scene, camera, rng, dist);
                    
                    color_sum = color_sum + sample_color;
                }
                
                Vec3f color = color_sum * inv_num_samples;

                const size_t idx = (static_cast<size_t>(i) * width + j) * 3;
                image[idx + 0] = (unsigned char)clampF(color.x, 0.0f, 255.0f);
                image[idx + 1] = (unsigned char)clampF(color.y, 0.0f, 255.0f);
                image[idx + 2] = (unsigned char)clampF(color.z, 0.0f, 255.0f);
            }
        }

        stbi_write_png(camera.image_name.c_str(), width, height, 3, image, width * 3);
        delete[] image;
    }

    return 0;
}

// ================= BVH BUILDING =================

void BuildTopLevelBVH(const Scene& scene)
{
    MakeTopLevelPrimsArray(scene);

    uint32_t N = topPrims.size();
    if (N == 0) return;
    
    topPrimIdx.resize(N);
    topBvhNodes.resize(N * 2);

    for (uint32_t i = 0; i < N; i++) topPrimIdx[i] = i;

    BVHNode& root = topBvhNodes[rootNodeIdx];
    root.leftNodeIdx = 0;
    root.firstPrimIdx = 0;
    root.primCount = N;

    UpdateNodeBounds(rootNodeIdx);
    Subdivide(rootNodeIdx);
}

void MakeTopLevelPrimsArray(const Scene& scene)
{
    int primCount = scene.meshes.size() + scene.spheres.size() + scene.triangles.size();
    topPrims.reserve(primCount);

    // Add all meshes (including instances)
    for (size_t i = 0; i < scene.meshes.size(); i++)
    {
        const Mesh& mesh = scene.meshes[i];
        // Use worldBounds for transformed meshes, localBounds otherwise
        const AABB& box = mesh.hasTransform ? mesh.worldBounds : mesh.localBounds;
        Vec3f center = (box.min + box.max) * 0.5f;
        topPrims.push_back(TopPrim(i, PrimKind::Mesh, box, center));
    }

    // Add all spheres
    for (size_t i = 0; i < scene.spheres.size(); i++)
    {
        const Sphere& sphere = scene.spheres[i];
        const AABB& box = sphere.hasTransform ? sphere.worldBounds : sphere.localBounds;
        Vec3f center = (box.min + box.max) * 0.5f;
        topPrims.push_back(TopPrim(i, PrimKind::Sphere, box, center));
    }

    // Add all triangles
    for (size_t i = 0; i < scene.triangles.size(); i++)
    {
        const Triangle& tri = scene.triangles[i];
        const AABB& box = tri.hasTransform ? tri.worldBounds : tri.localBounds;
        Vec3f center = (box.min + box.max) * 0.5f;
        topPrims.push_back(TopPrim(i, PrimKind::Triangle, box, center));
    }
}

void UpdateNodeBounds(uint32_t nodeIdx)
{
    BVHNode& node = topBvhNodes[nodeIdx];
    node.bounds.reset();

    for (uint32_t first = node.firstPrimIdx, i = 0; i < node.primCount; i++)
    {
        uint32_t leafPrimIdx = topPrimIdx[first + i];
        TopPrim& leafPrim = topPrims[leafPrimIdx];
        node.bounds.expand(leafPrim.bounds);
    }
}

void Subdivide(uint32_t nodeIdx)
{
    if (topBvhNodes[nodeIdx].primCount <= 2) return;

    Vec3f extent = topBvhNodes[nodeIdx].bounds.max - topBvhNodes[nodeIdx].bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = topBvhNodes[nodeIdx].bounds.min[axis] + extent[axis] * 0.5f;

    int i = topBvhNodes[nodeIdx].firstPrimIdx;
    int j = i + topBvhNodes[nodeIdx].primCount - 1;
    while (i <= j)
    {
        if (topPrims[topPrimIdx[i]].centroid[axis] < splitPos) i++;
        else std::swap(topPrimIdx[i], topPrimIdx[j--]);
    }

    int leftCount = i - topBvhNodes[nodeIdx].firstPrimIdx;
    if (leftCount == 0 || leftCount == topBvhNodes[nodeIdx].primCount) return;

    if (nodesUsed + 2 >= topBvhNodes.size()) {
        topBvhNodes.resize(topBvhNodes.size() * 2);
    }

    int leftChildIdx = nodesUsed++;
    int rightChildIdx = nodesUsed++;
    
    topBvhNodes[leftChildIdx].firstPrimIdx = topBvhNodes[nodeIdx].firstPrimIdx;
    topBvhNodes[leftChildIdx].primCount = leftCount;
    topBvhNodes[rightChildIdx].firstPrimIdx = i;
    topBvhNodes[rightChildIdx].primCount = topBvhNodes[nodeIdx].primCount - leftCount;
    topBvhNodes[nodeIdx].leftNodeIdx = leftChildIdx;
    topBvhNodes[nodeIdx].primCount = 0;

    UpdateNodeBounds(leftChildIdx);
    UpdateNodeBounds(rightChildIdx);

    Subdivide(leftChildIdx);
    Subdivide(rightChildIdx);
}

// ================= MESH BVH BUILDING =================

void BuildAllMeshBVHs(Scene& scene)
{
    meshBVHs.clear();
    
    int meshCount = 0;
    for (const auto& mesh : scene.meshes) {
        if (!mesh.isInstance) meshCount++;
    }
    meshBVHs.reserve(meshCount);
    
    for (size_t i = 0; i < scene.meshes.size(); i++) {
        Mesh& mesh = scene.meshes[i];
        
        if (!mesh.isInstance) {
            mesh.bvhIndex = meshBVHs.size();
            meshBVHs.push_back(MeshBVH());
            BuildMeshBVH(scene, i);
        } else {
            mesh.bvhIndex = scene.meshes[mesh.originalMeshId].bvhIndex;
        }
    }
}

void BuildMeshBVH(const Scene& scene, size_t meshIdx)
{
    const Mesh& mesh = scene.meshes[meshIdx];
    MeshBVH& bvh = meshBVHs[mesh.bvhIndex];
    
    MakeMeshPrimsArray(mesh, scene.vertex_data, bvh);
    
    uint32_t N = bvh.prims.size();
    if (N == 0) return;

    bvh.nodes.resize(N * 2);
    bvh.primIdx.resize(N);
    
    for (uint32_t i = 0; i < N; i++) {
        bvh.primIdx[i] = i;
    }
    
    BVHNode& root = bvh.nodes[bvh.rootNodeIdx];
    root.leftNodeIdx = 0;
    root.firstPrimIdx = 0;
    root.primCount = N;
    
    UpdateMeshNodeBounds(bvh, bvh.rootNodeIdx);
    SubdivideMesh(bvh, bvh.rootNodeIdx);
}

void MakeMeshPrimsArray(const Mesh& mesh, const vector<Vertex>& vertex_data, MeshBVH& bvh)
{
    bvh.prims.clear();
    bvh.prims.reserve(mesh.faces.size());
    
    for (size_t i = 0; i < mesh.faces.size(); i++) {
        const Face& face = mesh.faces[i];
        
        const Vec3f& v0 = vertex_data[face.i0 - 1].pos;
        const Vec3f& v1 = vertex_data[face.i1 - 1].pos;
        const Vec3f& v2 = vertex_data[face.i2 - 1].pos;
        
        AABB box;
        box.reset();
        box.expand(v0);
        box.expand(v1);
        box.expand(v2);
        
        Vec3f center = (v0 + v1 + v2) * (1.0f / 3.0f);
        
        bvh.prims.push_back(MeshPrim(i, box, center));
    }
}

void UpdateMeshNodeBounds(MeshBVH& bvh, uint32_t nodeIdx)
{
    BVHNode& node = bvh.nodes[nodeIdx];
    node.bounds.reset();
    
    for (uint32_t first = node.firstPrimIdx, i = 0; i < node.primCount; i++)
    {
        uint32_t leafPrimIdx = bvh.primIdx[first + i];
        MeshPrim& leafPrim = bvh.prims[leafPrimIdx];
        node.bounds.expand(leafPrim.bounds);
    }
}

void SubdivideMesh(MeshBVH& bvh, uint32_t nodeIdx)
{
    if (bvh.nodes[nodeIdx].primCount <= 2) return;
    
    Vec3f extent = bvh.nodes[nodeIdx].bounds.max - bvh.nodes[nodeIdx].bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = bvh.nodes[nodeIdx].bounds.min[axis] + extent[axis] * 0.5f;
    
    int i = bvh.nodes[nodeIdx].firstPrimIdx;
    int j = i + bvh.nodes[nodeIdx].primCount - 1;
    while (i <= j)
    {
        if (bvh.prims[bvh.primIdx[i]].centroid[axis] < splitPos)
            i++;
        else
            std::swap(bvh.primIdx[i], bvh.primIdx[j--]);
    }
    
    int leftCount = i - bvh.nodes[nodeIdx].firstPrimIdx;
    if (leftCount == 0 || leftCount == bvh.nodes[nodeIdx].primCount) return;
    
    if (bvh.nodesUsed + 2 >= bvh.nodes.size()) {
        bvh.nodes.resize(bvh.nodes.size() * 2);
    }
    
    int leftChildIdx = bvh.nodesUsed++;
    int rightChildIdx = bvh.nodesUsed++;
    
    bvh.nodes[leftChildIdx].firstPrimIdx = bvh.nodes[nodeIdx].firstPrimIdx;
    bvh.nodes[leftChildIdx].primCount = leftCount;
    bvh.nodes[rightChildIdx].firstPrimIdx = i;
    bvh.nodes[rightChildIdx].primCount = bvh.nodes[nodeIdx].primCount - leftCount;
    
    bvh.nodes[nodeIdx].leftNodeIdx = leftChildIdx;
    bvh.nodes[nodeIdx].primCount = 0;
    
    UpdateMeshNodeBounds(bvh, leftChildIdx);
    UpdateMeshNodeBounds(bvh, rightChildIdx);
    
    SubdivideMesh(bvh, leftChildIdx);
    SubdivideMesh(bvh, rightChildIdx);
}

// ================= RAY GENERATION =================

Ray ComputeRay(const Scene& scene, const Camera& camera, int j, int i, float jitter_x, float jitter_y, float aperture_u, float aperture_v,  float time) noexcept
{   
    float su = (j + jitter_x) * camera.pixel_width;  
    float sv = (i + jitter_y) * camera.pixel_height;
    Vec3f sample_point = camera.q + su * camera.u - sv * camera.v;

    Ray ray;
    ray.depth = 0;
    ray.time = time;
    ray.isInside = false;

    if (!camera.has_depth_of_field) { // Pinhole camera (no DOF)
        ray.origin = camera.position;
        ray.direction = (sample_point - camera.position).normalize();
    } 
    else {
        Vec3f dir = (camera.position - sample_point).normalize();
        float t_focal = camera.focus_distance / dir.dotProduct(-1 * camera.w);
        Vec3f focal_point = camera.position + dir * t_focal;
        
        float lens_u = (aperture_u - 0.5f) * camera.aperture_size;
        float lens_v = (aperture_v - 0.5f) * camera.aperture_size;
        Vec3f lens_sample = camera.position + lens_u * camera.u + lens_v * camera.v;
        
        ray.origin = lens_sample;
        ray.direction = (focal_point - lens_sample).normalize();
    }

    return ray;
}

// ================= VOLUME HELPERS =================

float phase_function(const Vec3f& dir_in, const Vec3f& dir_out, float g) noexcept
{
    Vec3f wi = dir_in.normalize();
    Vec3f wo = dir_out.normalize();

    float cos_theta = wi.dotProduct(wo);
    cos_theta = clampF(cos_theta, -1.0f, 1.0f);

    float denominator = 1.0f + g * g - 2.0f * g * cos_theta;
    return (1.0f - g * g) / (4.0f * PI * denominator * std::sqrt(denominator));
}

float SampleDensity(const Volume& vol, const Vec3f& p)
{
    if (!vol.grid) return 0.0f;

    nanovdb::Vec3f objP(p.x, p.y, p.z);
    nanovdb::Vec3f indexP = vol.grid->worldToIndexF(objP);
    auto accessor = vol.grid->getAccessor();

    // Use nearest neighbor for simplicity/speed
    nanovdb::Coord ijk(std::floor(indexP[0] + 0.5f), 
                       std::floor(indexP[1] + 0.5f), 
                       std::floor(indexP[2] + 0.5f));
                       
    float density = accessor.getValue(ijk);
    return density * vol.scale;
}

float GetVolumeTransmittance(const Vec3f& p, const Vec3f& lightPos, const Scene& scene)
{
    Vec3f dir = lightPos - p;
    float dist = dir.length();
    dir = dir.normalize();

    float transmittance = 1.0f;
    float step_size = 0.5f; 

    for (float t = 0; t < dist; t += step_size) {
        Vec3f current_p = p + dir * t;
        
        for (const auto& vol : scene.volumes) {
            float density = SampleDensity(vol, current_p);
            
            if (density > 0.0f) {
                Vec3f sigma_t = (vol.sigma_a + vol.sigma_s) * density;
                // Average extinction for single-channel transmittance
                float ext = (sigma_t.x + sigma_t.y + sigma_t.z) / 3.0f;
                transmittance *= std::exp(-ext * step_size);
            }
        }

        if (transmittance < 0.01f) return 0.0f;
    }

    return transmittance;
}

Vec3f SampleLightRadiance(const PointLight& light, const Vec3f& point, const Scene& scene) noexcept
{
    // 1. Check solid occlusion (Standard Shadow Ray)
    if (InShadow(point, light, Vec3f(0,0,0), 0.001f, scene, 0.0f)) {
        return Vec3f(0, 0, 0);
    }

    // 2. Check Volume Attenuation
    float T = GetVolumeTransmittance(point, light.position, scene);
    if (T <= 0.0f) return Vec3f(0, 0, 0);

    // 3. Distance Falloff
    Vec3f L = light.position - point;
    float d_sq = L.dotProduct(L);

    return (light.intensity * T) / d_sq;
}

Vec3f integrate_volume(const Ray& ray, const Scene& scene, float t_entry, float t_exit, const Vec3f& background_color)
{
    if (t_entry >= t_exit) return background_color;

    Vec3f L(0, 0, 0); 
    Vec3f T(1, 1, 1); 
    float step_size = 0.1f; 
    
    for (float t = t_entry; t < t_exit; t += step_size) {
        
        float dt = std::min(step_size, t_exit - t);
        Vec3f p = ray.origin + ray.direction * (t + 0.5f * dt);

        for (const auto& vol : scene.volumes) {
            float density = SampleDensity(vol, p);

            if (density > 0.0f) {
                Vec3f sigma_a = vol.sigma_a * density;
                Vec3f sigma_s = vol.sigma_s * density;
                Vec3f sigma_t = sigma_a + sigma_s; 

                // Transmittance over step dt
                Vec3f step_transmittance;
                step_transmittance.x = std::exp(-sigma_t.x * dt);
                step_transmittance.y = std::exp(-sigma_t.y * dt);
                step_transmittance.z = std::exp(-sigma_t.z * dt);

                // In-Scattering
                Vec3f in_scattered_light(0, 0, 0);

                for (const auto& light : scene.point_lights) {
                    Vec3f dir_to_light = light.position - p;
                    float dist_light = dir_to_light.length();
                    dir_to_light = dir_to_light.normalize();

                    float phase = phase_function(ray.direction, dir_to_light, vol.g);
                    
                    // Light visibility (Shadows + Volume Attenuation)
                    Vec3f Li = SampleLightRadiance(light, p, scene);

                    // Li is already attenuated by distance and transmittance
                    in_scattered_light = in_scattered_light + 
                                         Li.elwiseMult(sigma_s) * phase;
                }

                L = L + T.elwiseMult(in_scattered_light) * dt;
                T = T.elwiseMult(step_transmittance);
            }
        }
        
        if (T.x < 0.001f && T.y < 0.001f && T.z < 0.001f) break;
    }

    return L + T.elwiseMult(background_color);
}

// ================= COLOR COMPUTATION =================

Vec3f ComputeColor(const Ray& ray, const Scene& scene, const Camera& camera, std::mt19937& rng, std::uniform_real_distribution<float>& dist) 
{
    if (ray.depth > scene.max_recursion_depth) {
        return Vec3f(0, 0, 0);
    }

    // 1. Surface Rendering
    float tSurface = FLT_MAX; 
    Vec3f surface_color = scene.background_color; 
    HitRecord closestHit;

    if (FindClosestHit(ray, scene, camera, closestHit))
    {
        tSurface = closestHit.t;
        surface_color = ApplyShading(ray, scene, camera, closestHit, rng, dist);
    }
    
    // 2. Volume Integration
    if (!scene.volumes.empty()) {
        float tVolEnter = FLT_MAX;
        float tVolExit = -FLT_MAX;
        bool hitVolume = false;

        for (const auto& vol : scene.volumes) {
            AABBHit boxHit = IntersectAABB_EnterExit(ray, vol.worldBounds, 0.0f, FLT_MAX);
            if (boxHit.hit && boxHit.tEnter < boxHit.tExit) {
                hitVolume = true;
                if (boxHit.tEnter < tVolEnter) tVolEnter = boxHit.tEnter;
                if (boxHit.tExit > tVolExit)   tVolExit = boxHit.tExit;
            }
        }

        if (hitVolume) {
            tVolEnter = std::max(tVolEnter, 0.0f);
            float tEnd = std::min(tVolExit, tSurface);

            if (tVolEnter < tEnd) {
                return integrate_volume(ray, scene, tVolEnter, tEnd, surface_color);
            }
        }
    }

    return surface_color;
}

// ================= INTERSECTION =================

bool FindClosestHit(const Ray& ray, const Scene& scene, const Camera& camera, HitRecord& closestHit) noexcept
{
    float minT = FLT_MAX;
    bool has_intersected = false;
    PrimKind closestType;
    int closestMatId;
    float bary_beta, bary_gamma;
    int index;
    
    Face hitTriFace;
    Plane closestPlane;
    Sphere closestSphere;
    Triangle closestTriangle;

    int closestMeshId;
    bool closest_is_smooth = false;
    
    // 1. Test all planes first
    for (size_t i = 0; i < scene.planes.size(); i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(ray, plane.n_unit, plane.plane_d, minT);
        
        if (t < minT && t != RAY_MISS_VALUE) {
            minT = t;
            closestMatId = plane.material_id;
            closestType = PrimKind::Plane;
            closestPlane = plane;
            has_intersected = true;
            index = i;
        }
    }
    
    // 2. Traverse top-level BVH
    IntersectTopBVH(ray, scene, minT, has_intersected, closestType, closestMatId, 
                    index, hitTriFace, closestSphere, closestTriangle, closestMeshId,
                    closest_is_smooth, bary_beta, bary_gamma);
    
    if (!has_intersected) {
        return false;
    }
    
    // 3. Compute hit point and normal
    Vec3f hit_x = ray.origin + ray.direction * minT;
    Vec3f normal;
    
    switch(closestType) {
        case PrimKind::Mesh: {
            const Mesh& closestMesh = scene.meshes[closestMeshId];
            Vec3f objNormal;
            if (closest_is_smooth) {
                const Vec3f& nA = scene.vertex_data[hitTriFace.i0 - 1].normal;
                const Vec3f& nB = scene.vertex_data[hitTriFace.i1 - 1].normal;
                const Vec3f& nC = scene.vertex_data[hitTriFace.i2 - 1].normal;
                
                float alpha = 1.f - bary_beta - bary_gamma;
                objNormal = (nA * alpha + nB * bary_beta + nC * bary_gamma).normalize();
            }
            else {
                objNormal = hitTriFace.n_unit;
            }
            
            if (closestMesh.hasTransform && closestMesh.transformation.determinant3x3() < 0.0f) {
                objNormal = objNormal * -1.0f;
            }
            
            if (closestMesh.hasTransform) {
                normal = closestMesh.invTransformation.transpose().transformVector(objNormal).normalize();
            } else {
                normal = objNormal;
            }
            break;
        }
        case PrimKind::Triangle: {
            Vec3f objNormal = hitTriFace.n_unit;
             
            if (closestTriangle.hasTransform) {
                normal = closestTriangle.invTransformation.transpose().transformVector(objNormal).normalize();
            } else {
                normal = objNormal;
            }

            if (closestTriangle.hasTransform && closestTriangle.transformation.determinant3x3() < 0.0f) {
                normal = normal * -1.0f;
            }
            break;
        }
        case PrimKind::Sphere: {
            if (closestSphere.hasTransform) {
                Vec3f objHit = closestSphere.invTransformation.transformPoint(hit_x);
                Vertex center = scene.vertex_data[closestSphere.center_vertex_id - 1];
                Vec3f objNormal = (objHit - center.pos).normalize();
                normal = closestSphere.invTransformation.transpose().transformVector(objNormal).normalize();
            } else {
                Vertex center = scene.vertex_data[closestSphere.center_vertex_id - 1];
                normal = (hit_x - center.pos).normalize();
            }

            if (closestSphere.hasTransform && closestSphere.transformation.determinant3x3() < 0.0f) {
                normal = normal * -1.0f;
            }
            break;
        }
        case PrimKind::Plane: {
            normal = closestPlane.n_unit;
            break;
        }
        default: {
            normal = Vec3f(0, 0, 0);
        }
    }
    
    closestHit.materialId = closestMatId;
    closestHit.intersectionPoint = hit_x;
    closestHit.normal = normal;
    closestHit.kind = closestType;
    closestHit.t = minT;
    closestHit.primId = index;
    
    return true;
}

void IntersectTopBVH(const Ray& ray, const Scene& scene, float& minT, bool& has_intersected,
                     PrimKind& closestType, int& closestMatId, int& index,
                     Face& hitTriFace, Sphere& closestSphere, Triangle& closestTriangle,
                     int& closestMeshId, bool& closest_is_smooth,
                     float& bary_beta, float& bary_gamma) noexcept
{
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        BVHNode& node = topBvhNodes[nodeIdx];
        
        if (IntersectAABB(ray, node.bounds, minT) == RAY_MISS_VALUE) {
            continue;
        }
        
        if (node.primCount > 0)  // Leaf node
        {
            for (uint32_t i = 0; i < node.primCount; i++)
            {
                uint32_t primIdx = topPrimIdx[node.firstPrimIdx + i];
                TopPrim& prim = topPrims[primIdx];
                
                switch(prim.kind)
                {
                    case PrimKind::Mesh: {
                        const Mesh& mesh = scene.meshes[prim.index];
                        Ray testRay = ray;
                        float testMinT = minT;

                        if (mesh.hasTransform) {
                            Vec3f rayOrigin = ray.origin;
                            Vec3f rayDirection = ray.direction;
                            
                            if (mesh.has_motion_blur) {
                                rayOrigin = rayOrigin - mesh.motion_blur * ray.time;
                            }
                            
                            testRay.origin = mesh.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = mesh.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;
                            testRay.isInside = ray.isInside;
                            
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (mesh.has_motion_blur) {
                            testRay.origin = ray.origin - mesh.motion_blur * ray.time;
                            testRay.direction = ray.direction;
                            testRay.time = ray.time;
                            testRay.isInside = ray.isInside;
                        }
                        
                        float temp_b, temp_g;
                        Face tempFace;
                        
                        float t = IntersectMeshBVH(testRay, mesh, scene, mesh.bvhIndex, 
                                                   testMinT, tempFace, temp_b, temp_g);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (mesh.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = mesh.transformation.transformPoint(objHit);
                                if (mesh.has_motion_blur) {
                                    worldHit = worldHit + mesh.motion_blur * ray.time;
                                }
                                worldT = (worldHit - ray.origin).length();
                            }
                            
                            if (worldT < minT) {
                                minT = worldT;
                                closestMatId = mesh.material_id;
                                closestType = PrimKind::Mesh;
                                hitTriFace = tempFace;
                                closest_is_smooth = mesh.is_smooth;
                                closestMeshId = prim.index;
                                bary_beta = temp_b;
                                bary_gamma = temp_g;
                                index = prim.index;
                                has_intersected = true;
                            }
                        }
                        break;
                    }
                    
                    case PrimKind::Sphere: {
                        const Sphere& sphere = scene.spheres[prim.index];
                        Ray testRay = ray;
                        float testMinT = minT;

                        if (sphere.hasTransform) {
                            Vec3f rayOrigin = ray.origin;
                            Vec3f rayDirection = ray.direction;
                            
                            if (sphere.has_motion_blur) {
                                rayOrigin = rayOrigin - sphere.motion_blur * ray.time;
                            }
                            
                            testRay.origin = sphere.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = sphere.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;
                            
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = sphere.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (sphere.has_motion_blur) {
                            testRay.origin = ray.origin - sphere.motion_blur * ray.time;
                            testRay.direction = ray.direction;
                            testRay.time = ray.time;
                        }
                        
                        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
                        float t = IntersectSphere(testRay, center, sphere.radius, testMinT);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (sphere.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = sphere.transformation.transformPoint(objHit);
                                if (sphere.has_motion_blur) {
                                    worldHit = worldHit + sphere.motion_blur * ray.time;
                                }
                                worldT = (worldHit - ray.origin).length();
                            }
                            
                            if (worldT < minT) {
                                minT = worldT;
                                closestMatId = sphere.material_id;
                                closestType = PrimKind::Sphere;
                                closestSphere = sphere;
                                index = prim.index;
                                has_intersected = true;
                            }
                        }
                        break;
                    }
                    
                    case PrimKind::Triangle: {
                        const Triangle& triangle = scene.triangles[prim.index];
                        Ray testRay = ray;
                        float testMinT = minT;
                        
                        if (triangle.hasTransform) {
                            Vec3f rayOrigin = ray.origin;
                            Vec3f rayDirection = ray.direction;
                            
                            if (triangle.has_motion_blur) {
                                rayOrigin = rayOrigin - triangle.motion_blur * ray.time;
                            }
                            
                            testRay.origin = triangle.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = triangle.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;
                            
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = triangle.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (triangle.has_motion_blur) {
                            testRay.origin = ray.origin - triangle.motion_blur * ray.time;
                            testRay.direction = ray.direction;
                            testRay.time = ray.time;
                        }
                        
                        float dummy_b, dummy_g;
                        float t = IntersectsTriangle_Bary(testRay, triangle.face, 
                                                          scene.vertex_data, testMinT, 
                                                          dummy_b, dummy_g);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (triangle.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = triangle.transformation.transformPoint(objHit);
                                if (triangle.has_motion_blur) {
                                    worldHit = worldHit + triangle.motion_blur * ray.time;
                                }
                                worldT = (worldHit - ray.origin).length();
                            }
                            
                            if (worldT < minT) {
                                minT = worldT;
                                closestMatId = triangle.material_id;
                                closestType = PrimKind::Triangle;
                                hitTriFace = triangle.face;
                                closestTriangle = triangle;
                                index = prim.index;
                                has_intersected = true;
                            }
                        }
                        break;
                    }
                }
            }
        }
        else  // Interior node
        {
            uint32_t leftChild = node.leftNodeIdx;
            uint32_t rightChild = node.leftNodeIdx + 1;
            
            stack[stackPtr++] = rightChild;
            stack[stackPtr++] = leftChild;
        }
    }
}

float IntersectMeshBVH(const Ray& ray, const Mesh& mesh, const Scene& scene, 
                       int bvhIndex, float minT, Face& hitFace, 
                       float& beta_out, float& gamma_out) noexcept
{
    const MeshBVH& bvh = meshBVHs[bvhIndex];
    float meshMinT = minT;
    bool found = false;
    
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = bvh.rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        const BVHNode& node = bvh.nodes[nodeIdx];
        
        if (IntersectAABB(ray, node.bounds, meshMinT) == RAY_MISS_VALUE) {
            continue;
        }
        
        if (node.primCount > 0)  // Leaf node
        {
            for (uint32_t i = 0; i < node.primCount; i++)
            {
                uint32_t primIdx = bvh.primIdx[node.firstPrimIdx + i];
                const MeshPrim& meshPrim = bvh.prims[primIdx];
                const Face& face = mesh.faces[meshPrim.faceIdx];
                
                float temp_b, temp_g;
                float t = IntersectsTriangle_Bary(ray, face, scene.vertex_data, 
                                                  meshMinT, temp_b, temp_g);
                
                if (t < meshMinT && t != RAY_MISS_VALUE) {
                    meshMinT = t;
                    hitFace = face;
                    beta_out = temp_b;
                    gamma_out = temp_g;
                    found = true;
                }
            }
        }
        else  // Interior node
        {
            uint32_t leftChild = node.leftNodeIdx;
            uint32_t rightChild = node.leftNodeIdx + 1;
            
            stack[stackPtr++] = rightChild;
            stack[stackPtr++] = leftChild;
        }
    }
    
    return found ? meshMinT : RAY_MISS_VALUE;
}

// ================= PRIMITIVE INTERSECTION =================

AABBHit IntersectAABB_EnterExit(const Ray& ray, const AABB& box, float tMinAllowed, float tMaxAllowed) noexcept
{
    float tEnter = tMinAllowed;
    float tExit  = tMaxAllowed;

    if (ray.direction.x != 0.0f) {
        float invD = 1.0f / ray.direction.x;
        float t0 = (box.min.x - ray.origin.x) * invD;
        float t1 = (box.max.x - ray.origin.x) * invD;
        if (t0 > t1) std::swap(t0, t1);
        tEnter = std::max(tEnter, t0);
        tExit  = std::min(tExit,  t1);
        if (tExit < tEnter) return {0,0,false};
    } else {
        if (ray.origin.x < box.min.x || ray.origin.x > box.max.x) return {0,0,false};
    }

    if (ray.direction.y != 0.0f) {
        float invD = 1.0f / ray.direction.y;
        float t0 = (box.min.y - ray.origin.y) * invD;
        float t1 = (box.max.y - ray.origin.y) * invD;
        if (t0 > t1) std::swap(t0, t1);
        tEnter = std::max(tEnter, t0);
        tExit  = std::min(tExit,  t1);
        if (tExit < tEnter) return {0,0,false};
    } else {
        if (ray.origin.y < box.min.y || ray.origin.y > box.max.y) return {0,0,false};
    }

    if (ray.direction.z != 0.0f) {
        float invD = 1.0f / ray.direction.z;
        float t0 = (box.min.z - ray.origin.z) * invD;
        float t1 = (box.max.z - ray.origin.z) * invD;
        if (t0 > t1) std::swap(t0, t1);
        tEnter = std::max(tEnter, t0);
        tExit  = std::min(tExit,  t1);
        if (tExit < tEnter) return {0,0,false};
    } else {
        if (ray.origin.z < box.min.z || ray.origin.z > box.max.z) return {0,0,false};
    }

    return {tEnter, tExit, true};
}

float IntersectAABB(const Ray& ray, const AABB& box, float minT) noexcept
{
    float tMin = 0.0f;
    float tMax = minT;

    if (ray.direction.x != 0.0f) {
        float invD = 1.0f / ray.direction.x;
        float t0 = (box.min.x - ray.origin.x) * invD;
        float t1 = (box.max.x - ray.origin.x) * invD;
        if (t1 < t0) std::swap(t0, t1);
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) return RAY_MISS_VALUE;
    } else {
        if (ray.origin.x < box.min.x || ray.origin.x > box.max.x) return RAY_MISS_VALUE;
    }

    if (ray.direction.y != 0.0f) {
        float invD = 1.0f / ray.direction.y;
        float t0 = (box.min.y - ray.origin.y) * invD;
        float t1 = (box.max.y - ray.origin.y) * invD;
        if (t1 < t0) std::swap(t0, t1);
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) return RAY_MISS_VALUE;
    } else {
        if (ray.origin.y < box.min.y || ray.origin.y > box.max.y) return RAY_MISS_VALUE;
    }

    if (ray.direction.z != 0.0f) {
        float invD = 1.0f / ray.direction.z;
        float t0 = (box.min.z - ray.origin.z) * invD;
        float t1 = (box.max.z - ray.origin.z) * invD;
        if (t1 < t0) std::swap(t0, t1);
        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) return RAY_MISS_VALUE;
    } else {
        if (ray.origin.z < box.min.z || ray.origin.z > box.max.z) return RAY_MISS_VALUE;
    }

    float tHit = (tMin > 0.0f) ? tMin : tMax;
    if (tHit <= 0.0f || tHit > minT) return RAY_MISS_VALUE;

    return tHit;
}

float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const vector<Vertex>& vertex_data, float minT, float& beta_out, float& gamma_out) noexcept
{
   Vec3f tri_va = vertex_data[tri_face.i0 - 1].pos;
   Vec3f tri_vb = vertex_data[tri_face.i1 - 1].pos;
   Vec3f tri_vc = vertex_data[tri_face.i2 - 1].pos;

   if (!ray.isInside && backfaceCullingEnabled) {
       if (ray.direction.dotProduct(tri_face.n_unit) > 0) return RAY_MISS_VALUE;
   } 
   if (ray.isInside) {
       if (ray.direction.dotProduct(tri_face.n_unit) < 0) return RAY_MISS_VALUE;
   }

   Vec3f col_A0 = tri_va - tri_vb;
   Vec3f col_A1 = tri_va - tri_vc;
   Vec3f col_A2 = ray.direction;
   Vec3f col_b = tri_va - ray.origin;

    float det_A = det3x3(col_A0.x, col_A1.x, col_A2.x, col_A0.y, col_A1.y, col_A2.y, col_A0.z, col_A1.z, col_A2.z);
    if (std::fabs(det_A) < EPS_PARALLEL) return RAY_MISS_VALUE;
    
    float beta = det3x3(col_b.x, col_A1.x, col_A2.x, col_b.y, col_A1.y, col_A2.y, col_b.z, col_A1.z, col_A2.z) / det_A;
    if (beta < -EPS_PARALLEL) return RAY_MISS_VALUE;
    
    float gamma = det3x3(col_A0.x, col_b.x, col_A2.x, col_A0.y, col_b.y, col_A2.y, col_A0.z, col_b.z, col_A2.z) / det_A;
    if ((gamma < -EPS_PARALLEL) || ((beta + gamma) > (1.0f + EPS_PARALLEL))) return RAY_MISS_VALUE;
    
    float t = det3x3(col_A0.x, col_A1.x, col_b.x, col_A0.y, col_A1.y, col_b.y, col_A0.z, col_A1.z, col_b.z) / det_A;
    if (t < -EPS_PARALLEL || t >= minT) return RAY_MISS_VALUE;

    beta_out = beta;
    gamma_out = gamma;
    return t;
}

float IntersectSphere(const Ray& ray, const Vertex& center, float radius, float minT) noexcept
{
    Vec3f oc = ray.origin - center.pos;
    Vec3f d = ray.direction;
    float A = d.dotProduct(d);
    float B = 2 * d.dotProduct(oc);
    float C = oc.dotProduct(oc) - radius * radius;
    float delta = B * B - 4 * A * C;
    
    if (delta < 0.0f) return RAY_MISS_VALUE;
    
    float sqrtDelta = sqrt(delta);
    float t1 = (-B - sqrtDelta) / (2 * A);
    float t2 = (-B + sqrtDelta) / (2 * A);
    
    float t;
    if (t1 > 0.0f) t = t1;
    else if (t2 > 0.0f) t = t2;
    else return RAY_MISS_VALUE;
    
    if (t >= minT) return RAY_MISS_VALUE;
    return t;
}

float IntersectsPlane(const Ray& ray, const Vec3f& normal, float plane_d, float minT) noexcept
{
    float t_formula_denom = ray.direction.dotProduct(normal);
    if (std::fabs(t_formula_denom) < EPS_PARALLEL) return RAY_MISS_VALUE;
    float t = -(ray.origin.dotProduct(normal) + plane_d) / t_formula_denom;
    if (t < 0.0f || t >= minT) return RAY_MISS_VALUE;
    return t;
}

Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius) noexcept
{
   return (point - center.pos).normalize();
}

// ================= SHADOW RAYS =================

bool InShadow(const Vec3f& point, const PointLight& I, const Vec3f& n, float eps_shadow, const Scene& scene, float time) noexcept
{
    Vec3f toLight = I.position - point;
    float distToLight = toLight.length();
    
    Ray shadowRay;
    shadowRay.origin = point + n * eps_shadow;
    shadowRay.direction = toLight / distToLight;
    shadowRay.depth = 0;
    shadowRay.time = time; 
    shadowRay.isInside = false;

    float minT = distToLight + eps_shadow;

    // Test planes
    for (size_t i = 0; i < scene.planes.size(); i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(shadowRay, plane.n_unit, plane.plane_d, minT);
        if (t < minT && t != RAY_MISS_VALUE) return true;
    }

    // Traverse top-level BVH
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        BVHNode& node = topBvhNodes[nodeIdx];
        
        if (IntersectAABB(shadowRay, node.bounds, minT) == RAY_MISS_VALUE) continue;
        
        if (node.primCount > 0)
        {
            for (uint32_t i = 0; i < node.primCount; i++)
            {
                uint32_t primIdx = topPrimIdx[node.firstPrimIdx + i];
                TopPrim& prim = topPrims[primIdx];
                
                switch(prim.kind)
                {
                    case PrimKind::Mesh: {
                        const Mesh& mesh = scene.meshes[prim.index];
                        Ray testRay = shadowRay;
                        float testMinT = minT;
                        
                        if (mesh.hasTransform) {
                            testRay.origin = mesh.invTransformation.transformPoint(shadowRay.origin);
                            testRay.direction = mesh.invTransformation.transformVector(shadowRay.direction).normalize();
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }

                        if (mesh.hasTransform) {
                            Vec3f rayOrigin = shadowRay.origin;
                            Vec3f rayDirection = shadowRay.direction;
                            if (mesh.has_motion_blur) rayOrigin = rayOrigin - mesh.motion_blur * shadowRay.time;
                            testRay.origin = mesh.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = mesh.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;
                            testRay.isInside = shadowRay.isInside;
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (mesh.has_motion_blur) {
                            testRay.origin = shadowRay.origin - mesh.motion_blur * shadowRay.time;
                            testRay.direction = shadowRay.direction;
                            testRay.time = shadowRay.time;
                            testRay.isInside = shadowRay.isInside;
                        }
                        
                        float temp_b, temp_g;
                        Face tempFace;
                        
                        float t = IntersectMeshBVH(testRay, mesh, scene, mesh.bvhIndex, 
                                                   testMinT, tempFace, temp_b, temp_g);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (mesh.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = mesh.transformation.transformPoint(objHit);
                                if (mesh.has_motion_blur) worldHit = worldHit + mesh.motion_blur * shadowRay.time;
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            if (worldT < minT) return true;
                        }
                        break;
                    }
                    
                    case PrimKind::Sphere: {
                        const Sphere& sphere = scene.spheres[prim.index];
                        Ray testRay = shadowRay;
                        float testMinT = minT;

                        if (sphere.hasTransform) {
                            Vec3f rayOrigin = shadowRay.origin;
                            Vec3f rayDirection = shadowRay.direction;
                            if (sphere.has_motion_blur) rayOrigin = rayOrigin - sphere.motion_blur * shadowRay.time;
                            testRay.origin = sphere.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = sphere.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = sphere.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (sphere.has_motion_blur) {
                            testRay.origin = shadowRay.origin - sphere.motion_blur * shadowRay.time;
                            testRay.direction = shadowRay.direction;
                            testRay.time = shadowRay.time;
                        }
                        
                        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
                        float t = IntersectSphere(testRay, center, sphere.radius, testMinT);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (sphere.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = sphere.transformation.transformPoint(objHit);
                                if (sphere.has_motion_blur) worldHit = worldHit + sphere.motion_blur * shadowRay.time;
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            if (worldT < minT) return true;
                        }
                        break;
                    }
                    
                    case PrimKind::Triangle: {
                        const Triangle& triangle = scene.triangles[prim.index];
                        Ray testRay = shadowRay;
                        float testMinT = minT;
                        
                        if (triangle.hasTransform) {
                            Vec3f rayOrigin = shadowRay.origin;
                            Vec3f rayDirection = shadowRay.direction;
                            if (triangle.has_motion_blur) rayOrigin = rayOrigin - triangle.motion_blur * shadowRay.time;
                            testRay.origin = triangle.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = triangle.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = triangle.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (triangle.has_motion_blur) {
                            testRay.origin = shadowRay.origin - triangle.motion_blur * shadowRay.time;
                            testRay.direction = shadowRay.direction;
                            testRay.time = shadowRay.time;
                        }
                        
                        float dummy_b, dummy_g;
                        float t = IntersectsTriangle_Bary(testRay, triangle.face, 
                                                          scene.vertex_data, testMinT, 
                                                          dummy_b, dummy_g);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            float worldT = t;
                            if (triangle.hasTransform) {
                                Vec3f objHit = testRay.origin + testRay.direction * t;
                                Vec3f worldHit = triangle.transformation.transformPoint(objHit);
                                if (triangle.has_motion_blur) worldHit = worldHit + triangle.motion_blur * shadowRay.time;
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            if (worldT < minT) return true;
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            uint32_t leftChild = node.leftNodeIdx;
            uint32_t rightChild = node.leftNodeIdx + 1;
            stack[stackPtr++] = rightChild;
            stack[stackPtr++] = leftChild;
        }
    }
    
    return false;
}

// ================= SHADING =================

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit, std::mt19937& rng, std::uniform_real_distribution<float>& dist)
{
    Material mat = scene.materials[closestHit.materialId - 1];
    Vec3f color = Vec3f(0, 0, 0);

    Vec3f n_original = closestHit.normal;
    Vec3f d_inc = ray.direction;
    Vec3f n_shading = (n_original.dotProduct(d_inc) < 0.0f) ? n_original : (n_original * -1);

    Vec3f x = closestHit.intersectionPoint;
    float eps_shift = scene.shadow_ray_epsilon;
    Vec3f w0 = (ray.origin - x).normalize();

    if (mat.type == MaterialType::Mirror)
    {
        Vec3f wr_perfect = (n_shading * 2 * (n_shading.dotProduct(w0)) - w0).normalize();
        Vec3f wr = PerturbReflection(wr_perfect, mat.roughness, rng, dist);
        Ray reflectionRay;
        reflectionRay.origin = x + n_shading * eps_shift;
        reflectionRay.direction = wr;
        reflectionRay.depth = ray.depth + 1;
        reflectionRay.time = ray.time;
        reflectionRay.isInside = ray.isInside;
        color = color + mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera, rng, dist));
    }
    else if (mat.type == MaterialType::Conductor)
    {
        Vec3f wr_perfect = (n_shading * 2 * (n_shading.dotProduct(w0)) - w0).normalize();
        Vec3f wr = PerturbReflection(wr_perfect, mat.roughness, rng, dist);
        Ray reflectionRay;
        reflectionRay.origin = x + n_shading * eps_shift;
        reflectionRay.direction = wr;
        reflectionRay.depth = ray.depth + 1;
        reflectionRay.time = ray.time;
        reflectionRay.isInside = ray.isInside;
        float cosTheta = w0.dotProduct(n_shading);
        float Fresnel_r = Fresnel_Conductor(cosTheta, mat.refraction_index, mat.absorption_index);
        Vec3f reflectColor = mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera, rng, dist));
        color = color + reflectColor * Fresnel_r;
    }
    else if (mat.type == MaterialType::Dielectric)
    {
        bool entering = ray.isInside == false;
        Vec3f normal = entering ? n_original : (n_original * -1.0f);
        Vec3f w0_local = (ray.origin - x).normalize();
        
        float etaI = entering ? 1.0f : mat.refraction_index;
        float etaT = entering ? mat.refraction_index : 1.0f;
        float eta = etaI / etaT;
        
        float cosThetaI = w0_local.dotProduct(normal);
        float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
        float sin2ThetaT = eta * eta * sin2ThetaI;
        
        if (sin2ThetaT >= 1.0f) {
            Vec3f wr_perfect = (normal * 2.0f * cosThetaI - w0_local).normalize();
            Vec3f wr = PerturbReflection(wr_perfect, mat.roughness, rng, dist);
            Ray reflectionRay;
            reflectionRay.origin = x + normal * eps_shift;
            reflectionRay.direction = wr;
            reflectionRay.depth = ray.depth + 1;
            reflectionRay.time = ray.time;
            reflectionRay.isInside = ray.isInside;
            Vec3f reflectColor = ComputeColor(reflectionRay, scene, camera, rng, dist);
            color = color + reflectColor * mat.mirror_refl;
        }
        else {
            float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
            float Fr = Fresnel_Dielectric(cosThetaI, cosThetaT, etaI, etaT);
            
            Vec3f wr_perfect = (normal * 2.0f * cosThetaI - w0_local).normalize();
            Vec3f wr = PerturbReflection(wr_perfect, mat.roughness, rng, dist);
            Ray reflectionRay;
            reflectionRay.origin = x + normal * eps_shift;
            reflectionRay.direction = wr;
            reflectionRay.depth = ray.depth + 1;
            reflectionRay.time = ray.time;
            reflectionRay.isInside = ray.isInside;
            Vec3f reflectColor = ComputeColor(reflectionRay, scene, camera, rng, dist);
            reflectColor = reflectColor * mat.mirror_refl;

            Vec3f wt_perfect = ((w0_local * -1.0f) * eta + normal * (eta * cosThetaI - cosThetaT)).normalize();
            Vec3f wt = PerturbReflection(wt_perfect, mat.roughness, rng, dist);
            Ray refractionRay;
            refractionRay.origin = x - normal * eps_shift;
            refractionRay.direction = wt;
            refractionRay.depth = ray.depth + 1;
            refractionRay.time = ray.time;
            refractionRay.isInside = !ray.isInside;
            Vec3f refractColor = ComputeColor(refractionRay, scene, camera, rng, dist);
            
            if (!entering) {
                float d = closestHit.t;
                refractColor.x *= std::exp(-mat.absorption_coef.x * d);
                refractColor.y *= std::exp(-mat.absorption_coef.y * d);
                refractColor.z *= std::exp(-mat.absorption_coef.z * d);
            }
            color = color + reflectColor * Fr + refractColor * (1.0f - Fr);
        }
        if (!entering) return color;
    }

    color = color + scene.ambient_light.elwiseMult(mat.ambient_refl);

    for (const auto& point_light: scene.point_lights)
    {
        // New SampleLightRadiance handles InShadow and Volume Attenuation internally
        Vec3f incoming_radiance = SampleLightRadiance(point_light, closestHit.intersectionPoint, scene);
        
        bool visible = incoming_radiance.x > 0.0f || incoming_radiance.y > 0.0f || incoming_radiance.z > 0.0f;
        Vec3f to_light = (point_light.position - x).normalize();

        if (visible) 
        {
            color += ComputeDiffuseAndSpecular(mat, incoming_radiance, to_light, n_shading, w0);
        }
    }

    // Area lights
    for (const auto& area_light : scene.area_lights)
    {
        float u = (dist(rng) - 0.5f) * area_light.size;
        float v = (dist(rng) - 0.5f) * area_light.size;
        
        Vec3f n_light = area_light.normal;
        Vec3f tangent, bitangent;
        CreateOrthonormalBasis(n_light, tangent, bitangent);
        
        Vec3f light_sample_point = area_light.position + tangent * u + bitangent * v;
        Vec3f to_light = light_sample_point - x;
        float dist_to_light = to_light.length();
        Vec3f wi = to_light / dist_to_light; 
        
        Ray shadow_ray;
        shadow_ray.origin = x + n_shading * eps_shift;
        shadow_ray.direction = wi;
        shadow_ray.depth = 0;
        shadow_ray.time = ray.time;
        
        HitRecord shadow_hit;
        bool in_shadow = false;
        if (FindClosestHit(shadow_ray, scene, camera, shadow_hit)) {
            float dist_to_hit = (shadow_hit.intersectionPoint - shadow_ray.origin).length();
            if (dist_to_hit < dist_to_light - eps_shift) {
                in_shadow = true;
            }
        }
        
        if (!in_shadow) {
            float cos_light = std::abs(area_light.normal.dotProduct(wi * -1.0f));
            float area = area_light.size * area_light.size;
            float solid_angle = (area * cos_light) / (dist_to_light * dist_to_light);
            Vec3f irradiance = area_light.radiance * solid_angle;
            
            float cos_theta = std::max(0.0f, n_shading.dotProduct(wi));
            Vec3f diffuse = mat.diffuse_refl.elwiseMult(irradiance) * cos_theta;
            
            Vec3f h = (wi + w0).normalize();
            float cos_alpha = std::max(0.0f, n_shading.dotProduct(h));
            float spec_factor = std::pow(cos_alpha, mat.phong_exponent);
            Vec3f specular = mat.specular_refl.elwiseMult(irradiance) * spec_factor;
            
            color = color + diffuse + specular;
        }
    }

    return color;
}

Vec3f PerturbReflection(const Vec3f& perfect_reflection, float roughness,
                        std::mt19937& rng, std::uniform_real_distribution<float>& dist)
{
    if (roughness <= 0.0f) return perfect_reflection;
    
    Vec3f r = perfect_reflection;
    Vec3f u, v;
    CreateOrthonormalBasis(r, u, v);
    
    float xi1 = dist(rng);
    float xi2 = dist(rng);
    
    Vec3f r_real = r + u * (roughness * (xi1 - 0.5f)) + v * (roughness * (xi2 - 0.5f));
    return r_real.normalize();
}

float Fresnel_Dielectric(float cosTheta, float cosPhi, float n1, float n2) noexcept
{
    float r_par_num = n2 * cosTheta - n1 * cosPhi;
    float r_par_den = n2 * cosTheta + n1 * cosPhi;
    float R_paralel = r_par_num / r_par_den;

    float r_perp_num = n1 * cosTheta - n2 * cosPhi;
    float r_perp_den = n1 * cosTheta + n2 * cosPhi;
    float R_perp = r_perp_num / r_perp_den;

    return (R_paralel * R_paralel + R_perp * R_perp) * 0.5f;
}

float Fresnel_Conductor(float cosTheta, float refractionIndex, float absorptionIndex) noexcept
{
    float cos_sq = cosTheta * cosTheta;
    float n_sq = refractionIndex * refractionIndex;
    float k_sq = absorptionIndex * absorptionIndex;
    float two_n_cos = 2.0f * refractionIndex * cosTheta;

    float rs_num = (n_sq + k_sq) - two_n_cos + cos_sq;
    float rs_den = (n_sq + k_sq) + two_n_cos + cos_sq;
    float R_S = rs_num / rs_den;

    float rp_num = (n_sq + k_sq) * cos_sq - two_n_cos + 1;
    float rp_den = (n_sq + k_sq) * cos_sq + two_n_cos + 1;
    float R_P = rp_num / rp_den;

    return (R_S + R_P) * 0.5f;
}

Vec3f ComputeDiffuseAndSpecular(const Vec3f& origin, const Material& material, const PointLight& light, 
    const Vec3f& point, const Vec3f& normal, const Vec3f& w0) noexcept
{
    Vec3f L = light.position - point;
    Vec3f wi = L.normalize();

    float cos_theta = wi.dotProduct(normal);
    if (cos_theta < 0.0f) return Vec3f(0.0f, 0.0f, 0.f);

    float d_sq = L.dotProduct(L);
    Vec3f irradiance = light.intensity / d_sq;

    Vec3f diffuseTerm = (material.diffuse_refl * cos_theta).elwiseMult(irradiance);
    Vec3f h = (wi + w0).normalize();
    float cos_alpha = normal.dotProduct(h);

    if (cos_alpha < 0.0f) return diffuseTerm;

    Vec3f specularTerm = (material.specular_refl * std::pow(cos_alpha, material.phong_exponent)).elwiseMult(irradiance);
    return diffuseTerm + specularTerm;
}

Vec3f ComputeDiffuseAndSpecular(const Material& material, const Vec3f& incoming_radiance, 
                               const Vec3f& light_dir, const Vec3f& normal, const Vec3f& w0) noexcept {
    float cos_theta = std::max(0.0f, light_dir.dotProduct(normal));
    if (cos_theta <= 0.0f) return Vec3f(0.0f, 0.0f, 0.0f);

    Vec3f diffuseTerm = (material.diffuse_refl * cos_theta).elwiseMult(incoming_radiance);
    Vec3f h = (light_dir + w0).normalize();
    float cos_alpha = std::max(0.0f, normal.dotProduct(h));
    Vec3f specularTerm = (material.specular_refl * std::pow(cos_alpha, material.phong_exponent)).elwiseMult(incoming_radiance);

    return diffuseTerm + specularTerm;
}

void CreateOrthonormalBasis(const Vec3f& n, Vec3f& tangent, Vec3f& bitangent)
{
    Vec3f helper;
    if (std::abs(n.x) < std::abs(n.y) && std::abs(n.x) < std::abs(n.z)) {
        helper = Vec3f(1.0f, 0.0f, 0.0f);
    } else if (std::abs(n.y) < std::abs(n.z)) {
        helper = Vec3f(0.0f, 1.0f, 0.0f);
    } else {
        helper = Vec3f(0.0f, 0.0f, 1.0f);
    }
    
    tangent = n.crossProduct(helper).normalize();
    bitangent = n.crossProduct(tangent); 
}