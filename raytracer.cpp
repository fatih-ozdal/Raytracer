#include "raytracer.h"

// For debugging
#include "DebugBvh.h"
#include "Timer.h"
#include <sstream>

vector<TopPrim> topPrims;
vector<uint32_t> topPrimIdx;
vector<BVHNode> topBvhNodes; 
uint32_t rootNodeIdx = 0, nodesUsed = 1;

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
    
    // PrintBvhStats(scene, topBvhNodes, rootNodeIdx);
    
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

                Vec3f color;

                if (num_samples == 1) {
                    std::mt19937 rng(i * width + j);
                    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
                    
                    // Time for motion blur
                    float time = dist(rng);
                    
                    Ray ray = ComputeRay(scene, camera, j, i, 0.5, 0.5, 0.5, 0.5, time);
                    color = ComputeColor(ray, scene, camera, rng, dist);
                }
                else {
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
                        
                        int aperture_idx = shuffle_array[s];
                        int ax = aperture_idx % samples_per_side;
                        int ay = aperture_idx / samples_per_side;
                        float aperture_u = (ax + dist(rng)) / samples_per_side;  // [0, 1)
                        float aperture_v = (ay + dist(rng)) / samples_per_side;  // [0, 1)
                        
                        // Time for motion blur
                        float time = dist(rng);
                        
                        Ray ray = ComputeRay(scene, camera, j, i, 
                                        jitter_x, jitter_y, 
                                        aperture_u, aperture_v, 
                                        time);
                        Vec3f sample_color = ComputeColor(ray, scene, camera, rng, dist);
                        
                        color_sum = color_sum + sample_color;
                    }
                    
                    color = color_sum * inv_num_samples;
                }

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

// ============== BVH BUILDING ==============

void BuildTopLevelBVH(const Scene& scene)
{
    MakeTopLevelPrimsArray(scene);

    uint32_t N = topPrims.size();
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

// ============== MESH BVH BUILDING ==============

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
            // This is an original mesh - build its BVH
            mesh.bvhIndex = meshBVHs.size();
            meshBVHs.push_back(MeshBVH());
            BuildMeshBVH(scene, i);
        } else {
            // This is an instance - share the original's BVH
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

// ============== RAY GENERATION ==============

Ray ComputeRay(const Scene& scene, const Camera& camera, int j, int i, float jitter_x, float jitter_y, float aperture_u, float aperture_v,  float time) noexcept
{   
    float su = (j + jitter_x) * camera.pixel_width;  
    float sv = (i + jitter_y) * camera.pixel_height;
    Vec3f sample_point = camera.q + su * camera.u - sv * camera.v;

    Ray ray;
    ray.depth = 0;
    ray.time = time;

    ray.pixel_i = i;
    ray.pixel_j = j;

    if (!camera.has_depth_of_field) { // Pinhole camera (no DOF)
        ray.origin = camera.position;
        ray.direction = (sample_point - camera.position).normalize();
    } 
    else {
        // 1. Compute direction through pixel center
        Vec3f dir = (camera.position - sample_point).normalize();
        
        // 2. Find focal plane intersection point
        // t = focusDistance / dot(dir, -w)
        float t_focal = camera.focus_distance / dir.dotProduct(-1 * camera.w);
        Vec3f focal_point = camera.position + dir * t_focal;
        
        // 3. Sample point on aperture (square lens)
        // Map [0,1) x [0,1) to [-aperture_size/2, aperture_size/2]
        float lens_u = (aperture_u - 0.5f) * camera.aperture_size;
        float lens_v = (aperture_v - 0.5f) * camera.aperture_size;
        Vec3f lens_sample = camera.position + lens_u * camera.u + lens_v * camera.v;
        
        // 4. Ray from lens sample to focal point
        ray.origin = lens_sample;
        ray.direction = (focal_point - lens_sample).normalize();
    }

    return ray;
}

// ============== COLOR COMPUTATION ==============

Vec3f ComputeColor(const Ray& ray, const Scene& scene, const Camera& camera, std::mt19937& rng, std::uniform_real_distribution<float>& dist) 
{
    if (ray.depth > scene.max_recursion_depth) {
        return Vec3f(0, 0, 0);
    }

    HitRecord closestHit;
    if (FindClosestHit(ray, scene, camera, closestHit))
    {
        return ApplyShading(ray, scene, camera, closestHit, rng, dist);
    }
    else if (ray.depth == 0) // Background texture or color
    {
        if (scene.background_texture_id >= 0) {
            TextureMap* bgTex = scene.texture_maps[scene.background_texture_id].get();
            
            if (bgTex->needsUV()) {
                // Use pixel coordinates from ray
                float u = (float)ray.pixel_j / (float)camera.image_width;
                float v = (float)ray.pixel_i / (float)camera.image_height;
                
                return bgTex->getValueUV(u, v, scene.image_data);
            } else {
                // Procedural: use ray direction
                return bgTex->getValuePos(ray.direction);
            }
        }
        else {
            return scene.background_color;
        }
    }
    else
    {
        return Vec3f(0, 0, 0);
    }
}

// ============== INTERSECTION ==============

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
    
    // 1. Test all planes first (not in BVH - infinite primitives)
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
            // Step 1: Compute normal in object space
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
            
            // Step 2: Flip if negative determinant (MOVED OUTSIDE if/else)
            if (closestMesh.hasTransform && closestMesh.transformation.determinant3x3() < 0.0f) {
                objNormal = objNormal * -1.0f;
            }
            
            // Step 3: Transform to world space
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

    closestHit.beta = bary_beta;
    closestHit.gamma = bary_gamma;
    closestHit.hitFace = hitTriFace;
    closestHit.meshId = closestMeshId;
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
                        
                        // Transform ray to object space if needed
                        Ray testRay = ray;
                        float testMinT = minT;

                        if (mesh.hasTransform) {
                            Vec3f rayOrigin = ray.origin;
                            Vec3f rayDirection = ray.direction;
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (mesh.has_motion_blur) {
                                rayOrigin = rayOrigin - mesh.motion_blur * ray.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = mesh.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = mesh.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (mesh.has_motion_blur) { //  && !mesh.hasTransform
                            // Ray is already in world space
                            testRay.origin = ray.origin - mesh.motion_blur * ray.time;
                            testRay.direction = ray.direction;
                            testRay.time = ray.time;
                        }
                        
                        float temp_b, temp_g;
                        Face tempFace;
                        
                        // Use mesh's BVH index (shared for instances)
                        float t = IntersectMeshBVH(testRay, mesh, scene, mesh.bvhIndex, 
                                                   testMinT, tempFace, temp_b, temp_g);
                        
                        if (t < testMinT && t != RAY_MISS_VALUE) {
                            // Transform t back to world space if needed
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
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (sphere.has_motion_blur) {
                                rayOrigin = rayOrigin - sphere.motion_blur * ray.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = sphere.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = sphere.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = sphere.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (sphere.has_motion_blur) { //  && !sphere.hasTransform
                            // Ray is already in world space
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
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (triangle.has_motion_blur) {
                                rayOrigin = rayOrigin - triangle.motion_blur * ray.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = triangle.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = triangle.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = ray.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = ray.direction;
                            Vec3f objDir = triangle.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (triangle.has_motion_blur) { //  && !triangle.hasTransform
                            // Ray is already in world space
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

// ============== PRIMITIVE INTERSECTION ==============

float IntersectAABB(const Ray& ray, const AABB& box, float minT) noexcept
{
    float tMin = 0.0f;
    float tMax = minT;

    // X axis
    if (ray.direction.x != 0.0f) {
        float invD = 1.0f / ray.direction.x;
        float t0 = (box.min.x - ray.origin.x) * invD;
        float t1 = (box.max.x - ray.origin.x) * invD;
        if (t1 < t0) std::swap(t0, t1);

        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);

        if (tMax < tMin) {
            return RAY_MISS_VALUE;
        }
    } 
    else {
        if (ray.origin.x < box.min.x || ray.origin.x > box.max.x) {
            return RAY_MISS_VALUE;
        }
    }

    // Y axis
    if (ray.direction.y != 0.0f) {
        float invD = 1.0f / ray.direction.y;
        float t0 = (box.min.y - ray.origin.y) * invD;
        float t1 = (box.max.y - ray.origin.y) * invD;
        if (t1 < t0) std::swap(t0, t1);

        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) {
            return RAY_MISS_VALUE;
        }
    } 
    else {
        if (ray.origin.y < box.min.y || ray.origin.y > box.max.y) {
            return RAY_MISS_VALUE;
        }
    }

    // Z axis
    if (ray.direction.z != 0.0f) {
        float invD = 1.0f / ray.direction.z;
        float t0 = (box.min.z - ray.origin.z) * invD;
        float t1 = (box.max.z - ray.origin.z) * invD;
        if (t1 < t0) std::swap(t0, t1);

        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) {
            return RAY_MISS_VALUE;
        }
    } 
    else {
        if (ray.origin.z < box.min.z || ray.origin.z > box.max.z) {
            return RAY_MISS_VALUE;
        }
    }

    float tHit;
    if (tMin > 0.0f) {
        tHit = tMin;
    } 
    else {
        if (tMax <= 0.0f) {
            return RAY_MISS_VALUE;
        }
        tHit = tMax;
    }
    
    if (tHit > minT) {  // Changed from >= to >
        return RAY_MISS_VALUE;
    }

    return tHit;
}

float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const vector<Vertex>& vertex_data, float minT, float& beta_out, float& gamma_out) noexcept
{
   Vec3f tri_va = vertex_data[tri_face.i0 - 1].pos;
   Vec3f tri_vb = vertex_data[tri_face.i1 - 1].pos;
   Vec3f tri_vc = vertex_data[tri_face.i2 - 1].pos;

   Vec3f col_A0 = tri_va - tri_vb;
   Vec3f col_A1 = tri_va - tri_vc;
   Vec3f col_A2 = ray.direction;

   Vec3f col_b = tri_va - ray.origin;

    float det_A = det3x3(col_A0.x, col_A1.x, col_A2.x,
                        col_A0.y, col_A1.y, col_A2.y,
                        col_A0.z, col_A1.z, col_A2.z);

    if (std::fabs(det_A) < EPS_PARALLEL)
    { 
        return RAY_MISS_VALUE;
    }
    
    float beta = det3x3(col_b.x, col_A1.x, col_A2.x,
                        col_b.y, col_A1.y, col_A2.y,
                        col_b.z, col_A1.z, col_A2.z) / det_A;

    if (beta < -EPS_PARALLEL)
    { 
        return RAY_MISS_VALUE;
    }
    
    float gamma = det3x3(col_A0.x, col_b.x, col_A2.x,
                        col_A0.y, col_b.y, col_A2.y,
                        col_A0.z, col_b.z, col_A2.z) / det_A;

    if ((gamma < -EPS_PARALLEL) || ((beta + gamma) > (1.0f + EPS_PARALLEL)))
    { 
        return RAY_MISS_VALUE;
    }
    
    float t = det3x3(col_A0.x, col_A1.x, col_b.x,
                    col_A0.y, col_A1.y, col_b.y,
                    col_A0.z, col_A1.z, col_b.z) / det_A;

    if (t < -EPS_PARALLEL || t >= minT) 
    { 
        return RAY_MISS_VALUE;
    }

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
    
    if (delta < 0.0f) {
        return RAY_MISS_VALUE;
    }
    
    float sqrtDelta = sqrt(delta);
    float t1 = (-B - sqrtDelta) / (2 * A);
    float t2 = (-B + sqrtDelta) / (2 * A);
    
    float t;
    if (t1 > 0.0f) {
        t = t1;
    } else if (t2 > 0.0f) {
        t = t2;
    } else {
        return RAY_MISS_VALUE;
    }
    
    if (t >= minT) {
        return RAY_MISS_VALUE;
    }
    
    return t;
}

float IntersectsPlane(const Ray& ray, const Vec3f& normal, float plane_d, float minT) noexcept
{
    float t_formula_denom = ray.direction.dotProduct(normal);

    if (std::fabs(t_formula_denom) < EPS_PARALLEL)
    { 
        return RAY_MISS_VALUE;
    }

    float t = -(ray.origin.dotProduct(normal) + plane_d) / t_formula_denom;

    if (t < 0.0f || t >= minT) 
    { 
        return RAY_MISS_VALUE;
    }

    return t;
}

Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius) noexcept
{
   return (point - center.pos).normalize();
}

// ============== SHADOW RAYS ==============

bool InShadow(const Vec3f& point, const PointLight& I, const Vec3f& n, float eps_shadow, const Scene& scene, float time) noexcept
{
    Vec3f toLight = I.position - point;
    float distToLight = toLight.length();
    
    Ray shadowRay;
    shadowRay.origin = point + n * eps_shadow;
    shadowRay.direction = toLight / distToLight;
    shadowRay.depth = 0;
    shadowRay.time = time; 

    float minT = distToLight;

    // Test planes
    for (size_t i = 0; i < scene.planes.size(); i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(shadowRay, plane.n_unit, plane.plane_d, minT);
        if (t < minT && t != RAY_MISS_VALUE) {
            return true;
        }
    }

    // Traverse top-level BVH
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        BVHNode& node = topBvhNodes[nodeIdx];
        
        if (IntersectAABB(shadowRay, node.bounds, minT) == RAY_MISS_VALUE) {
            continue;
        }
        
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
                            // transform ray
                            testRay.origin = mesh.invTransformation.transformPoint(shadowRay.origin);
                            testRay.direction = mesh.invTransformation.transformVector(shadowRay.direction).normalize();
                            // transform testMinT
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();  // Transform'un scale etkisi
                            testMinT = minT * scale;
                        }

                        if (mesh.hasTransform) {
                            Vec3f rayOrigin = shadowRay.origin;
                            Vec3f rayDirection = shadowRay.direction;
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (mesh.has_motion_blur) {
                                rayOrigin = rayOrigin - mesh.motion_blur * shadowRay.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = mesh.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = mesh.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = mesh.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (mesh.has_motion_blur) { //  && !mesh.hasTransform
                            // Ray is already in world space
                            testRay.origin = shadowRay.origin - mesh.motion_blur * shadowRay.time;
                            testRay.direction = shadowRay.direction;
                            testRay.time = shadowRay.time;
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
                                    worldHit = worldHit + mesh.motion_blur * shadowRay.time;
                                }
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            
                            if (worldT < minT) {
                                return true;
                            }
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
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (sphere.has_motion_blur) {
                                rayOrigin = rayOrigin - sphere.motion_blur * shadowRay.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = sphere.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = sphere.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = sphere.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (sphere.has_motion_blur) { //  && !sphere.hasTransform
                            // Ray is already in world space
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
                                if (sphere.has_motion_blur) {
                                    worldHit = worldHit + sphere.motion_blur * shadowRay.time;
                                }
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            
                            if (worldT < minT) {
                                return true;
                            }
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
                            
                            // Apply motion blur offset in world space BEFORE inverse transform
                            if (triangle.has_motion_blur) {
                                rayOrigin = rayOrigin - triangle.motion_blur * shadowRay.time;
                            }
                            
                            // Now transform ray to object space
                            testRay.origin = triangle.invTransformation.transformPoint(rayOrigin);
                            testRay.direction = triangle.invTransformation.transformVector(rayDirection).normalize();
                            testRay.time = shadowRay.time;  // Keep time for secondary rays
                            
                            // Transform testMinT
                            Vec3f worldDir = shadowRay.direction;
                            Vec3f objDir = triangle.invTransformation.transformVector(worldDir);
                            float scale = objDir.length();
                            testMinT = minT * scale;
                        }
                        else if (triangle.has_motion_blur) { //  && !triangle.hasTransform
                            // Ray is already in world space
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
                                if (triangle.has_motion_blur) {
                                    worldHit = worldHit + triangle.motion_blur * shadowRay.time;
                                }
                                worldT = (worldHit - shadowRay.origin).length();
                            }
                            
                            if (worldT < minT) {
                                return true;
                            }
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

// ============== SHADING ==============

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit, std::mt19937& rng, std::uniform_real_distribution<float>& dist)
{
    Vec3f color = Vec3f(0, 0, 0);
    Material mat = scene.materials[closestHit.materialId - 1];

    // Check For Textures
    Vec2f uv = ComputeUV(closestHit, scene);
    const std::vector<int>* textureIds = GetTextureIds(closestHit, scene);

    Vec3f T, B;
    bool tangentBasisComputed = false;
    Vec3f n_original = closestHit.normal;

    if (textureIds && !textureIds->empty()) {
        for (int texId : *textureIds) {
            if (texId < 0 || texId >= (int)scene.texture_maps.size()) continue;
            
            TextureMap* tex = scene.texture_maps[texId].get();
            Vec3f texValue;
            
            if (tex->needsUV()) {
                texValue = tex->getValueUV(uv.u, uv.v, scene.image_data);
            } else {
                texValue = tex->getValuePos(closestHit.intersectionPoint);
            }
            
            switch (tex->decal_mode) {
                case DecalMode::ReplaceKd:
                    mat.diffuse_refl = texValue;
                    break;
                    
                case DecalMode::BlendKd:
                    mat.diffuse_refl = (mat.diffuse_refl + texValue) * 0.5f;
                    break;
                    
                case DecalMode::ReplaceKs:
                    mat.specular_refl = texValue;
                    break;
                    
                case DecalMode::ReplaceAll:
                    return texValue;
                    
                case DecalMode::ReplaceNormal: {
                    // NORMAL MAPPING
                    if (!tangentBasisComputed) {
                        ComputeTangentBasis(closestHit, scene, T, B, n_original);
                        tangentBasisComputed = true;
                    }
                    n_original = ApplyNormalMap(texValue, T, B, n_original);
                    break;
                }
                    
                case DecalMode::BumpNormal: {
                    // BUMP MAPPING
                    if (!tangentBasisComputed) {
                        ComputeTangentBasis(closestHit, scene, T, B, n_original);
                        tangentBasisComputed = true;
                    }
                    
                    const PerlinNoiseMap* perlinTex = dynamic_cast<const PerlinNoiseMap*>(tex);
                    if (perlinTex) {
                        n_original = ApplyPerlinBumpMap(closestHit, perlinTex, n_original, perlinTex->bump_factor);
                    } else {
                        n_original = ApplyBumpMap(closestHit, scene, tex, T, B, n_original, uv);
                    }
                    break;
                }
                    
                case DecalMode::ReplaceBackground:
                    break;
            }
        }
    }

    Vec3f n_shading = n_original;
    Vec3f d_inc = ray.direction;

    if (n_original.dotProduct(d_inc) < 0.0f)
    {
        n_shading = n_original;
    }
    else
    {
        n_shading = n_original * -1;
    }

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

        float cosTheta = w0.dotProduct(n_shading);
        float Fresnel_r = Fresnel_Conductor(cosTheta, mat.refraction_index, mat.absorption_index);

        Vec3f reflectColor = mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera, rng, dist));
        color = color + reflectColor * Fresnel_r;
    }
    else if (mat.type == MaterialType::Dielectric)
    {
        Vec3f d_inc_local = ray.direction;
        
        bool entering = n_original.dotProduct(d_inc_local) < 0.0f;
        
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
            color = color + mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera, rng, dist));
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
            Vec3f reflectColor = mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera, rng, dist));
            
            Vec3f wt_perfect = ((w0_local * -1.0f) * eta + normal * (eta * cosThetaI - cosThetaT)).normalize();
            Vec3f wt = PerturbReflection(wt_perfect, mat.roughness, rng, dist);

            Ray refractionRay;
            refractionRay.origin = x - normal * eps_shift;
            refractionRay.direction = wt;
            refractionRay.depth = ray.depth + 1;
            refractionRay.time = ray.time;
            Vec3f refractColor = ComputeColor(refractionRay, scene, camera, rng, dist);
            
            if (!entering) {
                float d = (x - ray.origin).length();
                refractColor.x *= std::exp(-mat.absorption_coef.x * d);
                refractColor.y *= std::exp(-mat.absorption_coef.y * d);
                refractColor.z *= std::exp(-mat.absorption_coef.z * d);
            }
            
            color = color + reflectColor * Fr + refractColor * (1.0f - Fr);
        }
        
        if (!entering) {
            return color;
        }
    }

    color = color + scene.ambient_light.elwiseMult(mat.ambient_refl);

    for (const auto& point_light: scene.point_lights)
    {
        if (!InShadow(x, point_light, n_shading, eps_shift, scene, ray.time))
        {
            color = color + ComputeDiffuseAndSpecular(ray.origin, mat, point_light, closestHit.intersectionPoint, n_shading, w0);
        }
    }

    for (const auto& area_light : scene.area_lights)
    {
        // Sample random point on area light square
        float u = (dist(rng) - 0.5f) * area_light.size;  // [-size/2, size/2]
        float v = (dist(rng) - 0.5f) * area_light.size;
        
        // Create orthonormal basis for area light
        Vec3f n_light = area_light.normal;
        Vec3f tangent, bitangent;
        CreateOrthonormalBasis(n_light, tangent, bitangent);
        
        Vec3f light_sample_point = area_light.position + tangent * u + bitangent * v;
        Vec3f to_light = light_sample_point - x;
        float dist_to_light = to_light.length();
        Vec3f wi = to_light / dist_to_light;  // Normalized direction to light
        
        // Check visibility with shadow ray
        Ray shadow_ray;
        shadow_ray.origin = x + n_shading * eps_shift;
        shadow_ray.direction = wi;
        shadow_ray.depth = 0;
        shadow_ray.time = ray.time;
        
        // Check if shadow ray hits anything before reaching the light
        HitRecord shadow_hit;
        bool in_shadow = false;
        if (FindClosestHit(shadow_ray, scene, camera, shadow_hit)) {
            float dist_to_hit = (shadow_hit.intersectionPoint - shadow_ray.origin).length();
            if (dist_to_hit < dist_to_light - eps_shift) {
                in_shadow = true;
            }
        }
        
        if (!in_shadow) {
            // Compute solid angle (from PDF: dw = A * (n_l · w_i) / d^2)
            float cos_light = std::abs(area_light.normal.dotProduct(wi * -1.0f));
            float area = area_light.size * area_light.size;
            float solid_angle = (area * cos_light) / (dist_to_light * dist_to_light);
            
            // Irradiance = Radiance x solid_angle
            Vec3f irradiance = area_light.radiance * solid_angle;
            
            // Diffuse component
            float cos_theta = std::max(0.0f, n_shading.dotProduct(wi));
            Vec3f diffuse = mat.diffuse_refl.elwiseMult(irradiance) * cos_theta;
            
            // Specular component (Blinn-Phong)
            Vec3f h = (wi + w0).normalize();
            float cos_alpha = std::max(0.0f, n_shading.dotProduct(h));
            float spec_factor = std::pow(cos_alpha, mat.phong_exponent);
            Vec3f specular = mat.specular_refl.elwiseMult(irradiance) * spec_factor;
            
            color = color + diffuse + specular;
        }
    }

    return color;
}

Vec2f ComputeUV(const HitRecord& hit, const Scene& scene) {
    switch (hit.kind) {
        case PrimKind::Sphere: {
            const Sphere& sphere = scene.spheres[hit.primId];
            
            // Hit point'i object space'e transform et
            Vec3f hitLocalSpace = hit.intersectionPoint;
            
            if (sphere.hasTransform) {
                // World space → Object space
                Vec3f pLocal = sphere.invTransformation.transformPoint(hitLocalSpace);
                hitLocalSpace = Vec3f(pLocal.x, pLocal.y, pLocal.z);
            }
            
            // Object space'de center ve local coordinates
            Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1].pos;
            Vec3f local = (hitLocalSpace - center) / sphere.radius;  // Normalize
            
            // Parametric coordinates
            float theta = acos(clampF(local.y, -1.0f, 1.0f));
            float phi = atan2(local.z, local.x);
            
            float u = (-phi + M_PI) / (2.0f * M_PI);
            float v = theta / M_PI;
            
            return Vec2f(u, v);
        }
        
        case PrimKind::Triangle: {
            // Barycentric interpolation
            const Triangle& tri = scene.triangles[hit.primId];
            const Vertex& a = scene.vertex_data[tri.face.i0 - 1];
            const Vertex& b = scene.vertex_data[tri.face.i1 - 1];
            const Vertex& c = scene.vertex_data[tri.face.i2 - 1];
            
            float u = a.uv.u + hit.beta * (b.uv.u - a.uv.u)+ hit.gamma * (c.uv.u - a.uv.u);
            float v = a.uv.v + hit.beta * (b.uv.v - a.uv.v)+ hit.gamma * (c.uv.v - a.uv.v);

            return Vec2f(u, v);
        }
        
        case PrimKind::Mesh: {
            // Same as triangle but from mesh
            const Mesh& mesh = scene.meshes[hit.meshId];
            const Face& face = hit.hitFace;
            
            const Vertex& a = scene.vertex_data[face.i0 - 1];
            const Vertex& b = scene.vertex_data[face.i1 - 1];
            const Vertex& c = scene.vertex_data[face.i2 - 1];
            
            float u = a.uv.u + hit.beta * (b.uv.u - a.uv.u)+ hit.gamma * (c.uv.u - a.uv.u);
            float v = a.uv.v + hit.beta * (b.uv.v - a.uv.v)+ hit.gamma * (c.uv.v - a.uv.v);

            return Vec2f(u, v);
        }
        
        default:
            return Vec2f(0, 0);
    }
}

const std::vector<int>* GetTextureIds(const HitRecord& hit, const Scene& scene) {
    switch (hit.kind) {
        case PrimKind::Sphere:
            return &scene.spheres[hit.primId].texture_ids;
            
        case PrimKind::Triangle:
            return &scene.triangles[hit.primId].texture_ids;
            
        case PrimKind::Mesh: {
            const Mesh& mesh = scene.meshes[hit.meshId];
            if (mesh.isInstance) {
                return &scene.meshes[mesh.originalMeshId].texture_ids;
            }
            return &mesh.texture_ids;
        }
        
        case PrimKind::Plane:
            return &scene.planes[hit.primId].texture_ids;
            
        default:
            return nullptr;
    }
}


void ComputeTangentBasis(const HitRecord& hit, const Scene& scene, Vec3f& T, Vec3f& B, const Vec3f& N)
{
    switch (hit.kind) {
        case PrimKind::Triangle:
        case PrimKind::Mesh: {
            // Get vertices and UV coordinates
            const Face& face = hit.hitFace;
            const Vertex& v0 = scene.vertex_data[face.i0 - 1];
            const Vertex& v1 = scene.vertex_data[face.i1 - 1];
            const Vertex& v2 = scene.vertex_data[face.i2 - 1];
            
            // Edge vectors in world space
            Vec3f E1 = v1.pos - v0.pos;
            Vec3f E2 = v2.pos - v0.pos;
            
            // UV differences
            float du1 = v1.uv.u - v0.uv.u;
            float dv1 = v1.uv.v - v0.uv.v;
            float du2 = v2.uv.u - v0.uv.u;
            float dv2 = v2.uv.v - v0.uv.v;
            
            float det = du1 * dv2 - du2 * dv1;
            
            if (std::abs(det) < 1e-8f) {
                // Degenerate UV mapping - create arbitrary tangent basis
                CreateOrthonormalBasis(N, T, B);
                return;
            }
            
            float invDet = 1.0f / det;
            
            // From slides:
            // | T |   1   | dv2  -dv1 | | E1 |
            // | B | = --- | -du2  du1 | | E2 |
            //         det
            T = (E1 * dv2 - E2 * dv1) * invDet;
            B = (E2 * du1 - E1 * du2) * invDet;
            
            // Gram-Schmidt orthogonalization
            T = (T - N * N.dotProduct(T)).normalize();
            B = N.crossProduct(T);
            break;
        }
        
        case PrimKind::Sphere: {
            // For spheres: compute from spherical parametrization
            // p(θ,φ) = center + r*(sinθ cosφ, cosθ, sinθ sinφ)
            // u = (-φ + π) / (2π),  v = θ / π
            
            const Sphere& sphere = scene.spheres[hit.primId];
            Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1].pos;
            Vec3f local = (hit.intersectionPoint - center) / sphere.radius;
            
            float theta = acos(std::clamp(local.y, -1.0f, 1.0f));
            float phi = atan2(local.z, local.x);
            
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);
            
            // ∂p/∂φ (tangent in U direction) - note: u is related to -φ
            // T = r * sinθ * (sinφ, 0, -cosφ) → normalized: (sinφ, 0, -cosφ)
            T = Vec3f(sinPhi, 0.0f, -cosPhi);
            
            // ∂p/∂θ (tangent in V direction)
            // B = r * (cosθ cosφ, -sinθ, cosθ sinφ) → normalized
            B = Vec3f(cosTheta * cosPhi, -sinTheta, cosTheta * sinPhi);
            
            // Orthonormalize
            T = (T - N * N.dotProduct(T)).normalize();
            B = N.crossProduct(T);
            break;
        }
        
        default:
            CreateOrthonormalBasis(N, T, B);
            break;
    }
}

// ============== NORMAL MAPPING ==============

Vec3f ApplyNormalMap(const Vec3f& texValue, const Vec3f& T, const Vec3f& B, const Vec3f& N)
{
    // texValue is in [0, 1] range (already normalized by /255)
    // Convert to direction: map [0,1] → [-1,1]
    Vec3f n_tangent;
    n_tangent.x = texValue.x * 2.0f - 1.0f;  // R → X
    n_tangent.y = texValue.y * 2.0f - 1.0f;  // G → Y  
    n_tangent.z = texValue.z * 2.0f - 1.0f;  // B → Z
    n_tangent = n_tangent.normalize();
    
    // TBN matrix transforms from tangent space to world space
    // n_world = T * n.x + B * n.y + N * n.z
    Vec3f n_world = T * n_tangent.x + B * n_tangent.y + N * n_tangent.z;
    
    return n_world.normalize();
}

// ============== BUMP MAPPING (Image Texture) ==============

Vec3f ApplyBumpMap(const HitRecord& hit, const Scene& scene, const TextureMap* tex,
                   const Vec3f& T, const Vec3f& B, const Vec3f& N, const Vec2f& uv)
{
    const ImageTextureMap* imgTex = dynamic_cast<const ImageTextureMap*>(tex);
    if (!imgTex || imgTex->image_id < 0 || imgTex->image_id >= (int)scene.image_data.size()) {
        return N;
    }
    
    const ImageData& img = scene.image_data[imgTex->image_id];
    float bumpFactor = imgTex->bump_factor;
    
    // Step sizes for finite differences (in UV space)
    float du = 1.0f / static_cast<float>(img.width);
    float dv = 1.0f / static_cast<float>(img.height);
    
    // Height function: grayscale value
    auto getHeight = [&](float u, float v) -> float {
        Vec3f color = img.sample(u, v, imgTex->interpolation);
        // Grayscale: (R + G + B) / 3, normalized to [0, 1]
        return (color.x + color.y + color.z) / (3.0f * 255.0f);
    };
    
    // Sample heights for forward differences
    float h = getHeight(uv.u, uv.v);
    float h_u = getHeight(uv.u + du, uv.v);
    float h_v = getHeight(uv.u, uv.v + dv);
    
    // Partial derivatives: ∂h/∂u and ∂h/∂v
    float dh_du = (h_u - h) / du * bumpFactor;
    float dh_dv = (h_v - h) / dv * bumpFactor;
    
    // Perturbed normal (from slides):
    // n' = N - ∂h/∂u * T - ∂h/∂v * B
    Vec3f n_perturbed = N - T * dh_du - B * dh_dv;
    
    return n_perturbed.normalize();
}

// ============== PERLIN NOISE GRADIENT ==============

Vec3f ComputePerlinGradient(const Vec3f& pos, float noiseScale)
{
    // Finite differences for gradient computation
    const float eps = 0.001f;
    
    // We need raw Perlin noise values, not the converted [0,1] version
    // This is a helper that computes turbulence at a point
    // You may need to expose PerlinNoiseMap::turbulence or compute inline
    
    // For now, using central differences with getValuePos
    // Note: This is approximate since getValuePos applies conversion
    
    // Actually, for proper gradient, we need the raw noise before conversion
    // Let's compute using finite differences on the final value
    // The conversion is monotonic, so gradient direction is preserved
    
    auto sampleNoise = [&](const Vec3f& p) -> float {
        // Inline Perlin turbulence computation
        // This should match your PerlinNoiseMap implementation
        
        static const Vec3f gradients[16] = {
            Vec3f(1, 1, 0),   Vec3f(-1, 1, 0),   Vec3f(1, -1, 0),   Vec3f(-1, -1, 0),
            Vec3f(1, 0, 1),   Vec3f(-1, 0, 1),   Vec3f(1, 0, -1),   Vec3f(-1, 0, -1),
            Vec3f(0, 1, 1),   Vec3f(0, -1, 1),   Vec3f(0, 1, -1),   Vec3f(0, -1, -1),
            Vec3f(1, 1, 0),   Vec3f(-1, 1, 0),   Vec3f(0, -1, 1),   Vec3f(0, -1, -1)
        };
        
        // Use your shuffled table here!
        static const int table[16] = {
            10, 2, 7, 14, 1, 13, 4, 9, 6, 0, 15, 8, 3, 11, 5, 12
        };
        
        Vec3f scaled = p * noiseScale;
        
        int i = static_cast<int>(std::floor(scaled.x));
        int j = static_cast<int>(std::floor(scaled.y));
        int k = static_cast<int>(std::floor(scaled.z));
        
        float dx = scaled.x - i;
        float dy = scaled.y - j;
        float dz = scaled.z - k;
        
        auto fadeWeight = [](float d) -> float {
            float absD = std::abs(d);
            if (absD >= 1.0f) return 0.0f;
            return -6*pow(absD,5) + 15*pow(absD,4) - 10*pow(absD,3) + 1;
        };
        
        auto getGradient = [&](int ii, int jj, int kk) -> Vec3f {
            int idx = table[std::abs(kk) % 16];
            idx = table[std::abs(jj + idx) % 16];
            idx = table[std::abs(ii + idx) % 16];
            return gradients[idx];
        };
        
        float sum = 0.0f;
        for (int di = 0; di <= 1; di++) {
            for (int dj = 0; dj <= 1; dj++) {
                for (int dk = 0; dk <= 1; dk++) {
                    Vec3f g = getGradient(i + di, j + dj, k + dk);
                    Vec3f dist(dx - di, dy - dj, dz - dk);
                    float dot = g.x * dist.x + g.y * dist.y + g.z * dist.z;
                    float w = fadeWeight(dx - di) * fadeWeight(dy - dj) * fadeWeight(dz - dk);
                    sum += w * dot;
                }
            }
        }
        
        return sum;
    };
    
    // Central differences
    float gx = (sampleNoise(pos + Vec3f(eps, 0, 0)) - sampleNoise(pos - Vec3f(eps, 0, 0))) / (2.0f * eps);
    float gy = (sampleNoise(pos + Vec3f(0, eps, 0)) - sampleNoise(pos - Vec3f(0, eps, 0))) / (2.0f * eps);
    float gz = (sampleNoise(pos + Vec3f(0, 0, eps)) - sampleNoise(pos - Vec3f(0, 0, eps))) / (2.0f * eps);
    
    return Vec3f(gx, gy, gz);
}

// ============== BUMP MAPPING (Perlin Noise) ==============

Vec3f ApplyPerlinBumpMap(const HitRecord& hit, const PerlinNoiseMap* perlinTex, const Vec3f& N, float bumpFactor)
{
    Vec3f pos = hit.intersectionPoint;
    
    // Compute gradient of Perlin noise at this point
    Vec3f gradient = ComputePerlinGradient(pos, perlinTex->noise_scale) * bumpFactor;
    
    // Surface gradient: project gradient onto surface plane
    // ∇_s h = ∇h - (∇h · N) * N
    float dotGradN = gradient.dotProduct(N);
    Vec3f surface_gradient = gradient - N * dotGradN;
    
    // Perturbed normal: n' = N - ∇_s h
    Vec3f n_perturbed = N - surface_gradient;
    
    return n_perturbed.normalize();
}

Vec3f PerturbReflection(const Vec3f& perfect_reflection, float roughness,
                        std::mt19937& rng, std::uniform_real_distribution<float>& dist)
{
    if (roughness <= 0.0f) {
        return perfect_reflection;
    }
    
    // Create orthonormal basis around perfect reflection
    Vec3f r = perfect_reflection;
    Vec3f u, v;
    CreateOrthonormalBasis(r, u, v);
    
    // Generate two random numbers in [0, 1)
    float xi1 = dist(rng);
    float xi2 = dist(rng);
    
    // Center them to [-0.5, 0.5) and apply roughness
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

    if (cos_theta < 0.0f)
    {
        return Vec3f(0.0f, 0.0f, 0.f);
    }

    float d_sq = L.dotProduct(L);
    Vec3f irradiance = light.intensity / d_sq;

    Vec3f diffuseTerm = (material.diffuse_refl * cos_theta).elwiseMult(irradiance);

    Vec3f h = (wi + w0).normalize();

    float cos_alpha = normal.dotProduct(h);

    if (cos_alpha < 0.0f)
    {
        return diffuseTerm;
    }

    Vec3f specularTerm = (material.specular_refl * std::pow(cos_alpha, material.phong_exponent)).elwiseMult(irradiance);

    return diffuseTerm + specularTerm;
}

void CreateOrthonormalBasis(const Vec3f& n, Vec3f& tangent, Vec3f& bitangent)
{
    // Find the component with minimum absolute value
    Vec3f helper;
    if (std::abs(n.x) < std::abs(n.y) && std::abs(n.x) < std::abs(n.z)) {
        helper = Vec3f(1.0f, 0.0f, 0.0f);
    } else if (std::abs(n.y) < std::abs(n.z)) {
        helper = Vec3f(0.0f, 1.0f, 0.0f);
    } else {
        helper = Vec3f(0.0f, 0.0f, 1.0f);
    }
    
    // Create orthonormal basis
    tangent = n.crossProduct(helper).normalize();
    bitangent = n.crossProduct(tangent);  // Already normalized if n and tangent are normalized
}
