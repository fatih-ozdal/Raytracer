#include "raytracer.h"
#include <iostream>

vector<TopPrim> topPrims;
vector<uint32_t> topPrimIdx;
vector<BVHNode> topBvhNodes; 
uint32_t rootNodeIdx = 0, nodesUsed = 1;

struct BvhStats {
    int nodeCount = 0;
    int leafCount = 0;
    int interiorCount = 0;
    int maxDepth = 0;
    int totalLeafPrims = 0;
    int maxLeafPrims = 0;
};

void GatherStats(uint32_t nodeIdx, int depth, BvhStats& stats)
{
    if (nodeIdx == UINT32_MAX) return;
    const BVHNode& node = topBvhNodes[nodeIdx];

    stats.nodeCount++;
    stats.maxDepth = std::max(stats.maxDepth, depth);

    if (node.primCount > 0) { // leaf
        stats.leafCount++;
        stats.totalLeafPrims += node.primCount;
        stats.maxLeafPrims = std::max(stats.maxLeafPrims, (int)node.primCount);
    } else { // interior
        stats.interiorCount++;
        uint32_t left = node.leftNodeIdx;
        uint32_t right = node.leftNodeIdx + 1;
        GatherStats(left, depth + 1, stats);
        GatherStats(right, depth + 1, stats);
    }
}

struct MeshBvhStats {
    int nodeCount       = 0;
    int leafCount       = 0;
    int interiorCount   = 0;
    int maxDepth        = 0;
    int totalLeafPrims  = 0;
    int maxLeafPrims    = 0;
};

void GatherMeshBvhStats(const MeshBVH& bvh, uint32_t nodeIdx, int depth, MeshBvhStats& stats)
{
    if (nodeIdx >= bvh.nodes.size())
        return;

    const BVHNode& node = bvh.nodes[nodeIdx];

    // Skip unused nodes (primCount == 0 and no children set) if you want:
    if (node.primCount == 0 && node.leftNodeIdx == 0 && nodeIdx != bvh.rootNodeIdx) {
        return;
    }

    stats.nodeCount++;
    if (depth > stats.maxDepth) stats.maxDepth = depth;

    if (node.primCount > 0) {
        // Leaf
        stats.leafCount++;
        stats.totalLeafPrims += node.primCount;
        if (node.primCount > stats.maxLeafPrims)
            stats.maxLeafPrims = node.primCount;
    } else {
        // Interior
        stats.interiorCount++;

        uint32_t leftChild  = node.leftNodeIdx;
        uint32_t rightChild = node.leftNodeIdx + 1;

        GatherMeshBvhStats(bvh, leftChild,  depth + 1, stats);
        GatherMeshBvhStats(bvh, rightChild, depth + 1, stats);
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: %s <scene.json>\n", argv[0]);
        return -1;
    }
    
    parser p;
    string jsonFile = argv[1];
    Scene scene = p.loadFromJson(jsonFile);

    BuildAllMeshBVHs(scene);
    BuildTopLevelBVH(scene);

    BvhStats s;
    GatherStats(rootNodeIdx, 0, s);
    std::cout << "nodes=" << s.nodeCount
            << " leaves=" << s.leafCount
            << " interior=" << s.interiorCount
            << " maxDepth=" << s.maxDepth
            << " avgLeafPrims=" << (s.leafCount ? (float)s.totalLeafPrims / s.leafCount : 0)
            << " maxLeafPrims=" << s.maxLeafPrims
            << std::endl;

    for (size_t m = 0; m < meshBVHs.size(); ++m) {
        const MeshBVH& bvh = meshBVHs[m];
        MeshBvhStats stats;

        GatherMeshBvhStats(bvh, bvh.rootNodeIdx, 0, stats);

        float avgLeafPrims = stats.leafCount ? 
            (float)stats.totalLeafPrims / (float)stats.leafCount : 0.0f;

        std::cout << "Mesh " << m << " BVH stats:\n"
                << "  nodes       = " << stats.nodeCount << "\n"
                << "  leaves      = " << stats.leafCount << "\n"
                << "  interior    = " << stats.interiorCount << "\n"
                << "  maxDepth    = " << stats.maxDepth << "\n"
                << "  avgLeafPrim = " << avgLeafPrims << "\n"
                << "  maxLeafPrim = " << stats.maxLeafPrims << "\n\n";
    }

    for (const Camera& camera : scene.cameras)  // read-only ref is fine
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

    return 0;
}

void BuildTopLevelBVH(const Scene& scene)
{
    MakeTopLevelPrimsArray(scene);

    uint32_t N = topPrims.size();
    topPrimIdx.resize(2*N);
    topBvhNodes.resize(2*N);

    for (int i = 0; i < N; i++) topPrimIdx[i] = i;  // these will get swapped around

    // assign all prims to root node
    BVHNode& root = topBvhNodes[rootNodeIdx];
    root.leftNodeIdx = 0;
    root.firstPrimIdx = 0;
    root.primCount = N;

    UpdateNodeBounds( rootNodeIdx );
    Subdivide( rootNodeIdx );
}

void MakeTopLevelPrimsArray(const Scene& scene)
{
    size_t i = 0;
    for (const auto& mesh: scene.meshes)
    {
        const AABB box = mesh.localBounds;
        Vec3f center = 0.5f * (box.min + box.max);
        topPrims.push_back(TopPrim(i, PrimKind::Mesh, box, center));
        i++;
    }

    i = 0;
    for (const auto& sphere: scene.spheres)
    {
        const AABB box = sphere.localBounds;
        Vec3f center = 0.5f * (box.min + box.max);
        topPrims.push_back(TopPrim(i, PrimKind::Sphere, box, center));
        i++;
    }

    i = 0;
    for (const auto& tri: scene.triangles)
    {
        const AABB box = tri.localBounds;
        Vec3f center = 0.5f * (box.min + box.max);
        topPrims.push_back(TopPrim(i, PrimKind::Triangle, box, center));
        i++;
    }
    
    // TODO: ADD MESH INSTANCE
}

void UpdateNodeBounds( uint32_t nodeIdx )
{
    BVHNode& node = topBvhNodes[nodeIdx];
    node.bounds.reset();    // TODO: this reset might be wrong

    for (uint32_t first = node.firstPrimIdx, i = 0; i < node.primCount; i++)
    {
        uint32_t leafPrimIdx = topPrimIdx[first + i];
        TopPrim& leafPrim = topPrims[leafPrimIdx];
        node.bounds.expand(leafPrim.bounds);
    }
}

void Subdivide( uint32_t nodeIdx )
{
    // DON'T hold a reference - access by index instead
    
    // terminate recursion
    if (topBvhNodes[nodeIdx].primCount <= 2) return;

    // determine split axis and position
    Vec3f extent = topBvhNodes[nodeIdx].bounds.max - topBvhNodes[nodeIdx].bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = topBvhNodes[nodeIdx].bounds.min[axis] + extent[axis] * 0.5f;

    // in-place partition
    int i = topBvhNodes[nodeIdx].firstPrimIdx;
    int j = i + topBvhNodes[nodeIdx].primCount - 1;
    while (i <= j)
    {
        if (topPrims[topPrimIdx[i]].centroid[axis] < splitPos) i++;
        else std::swap( topPrimIdx[i], topPrimIdx[j--] );
    }

    // abort split if one of the sides is empty
    int leftCount = i - topBvhNodes[nodeIdx].firstPrimIdx;
    if (leftCount == 0 || leftCount == topBvhNodes[nodeIdx].primCount) return;

    // RESIZE IF NEEDED
    if (nodesUsed + 2 >= topBvhNodes.size()) {
        topBvhNodes.resize(topBvhNodes.size() * 2);
    }

    // create child nodes
    int leftChildIdx = nodesUsed++;
    int rightChildIdx = nodesUsed++;
    
    topBvhNodes[leftChildIdx].firstPrimIdx = topBvhNodes[nodeIdx].firstPrimIdx;
    topBvhNodes[leftChildIdx].primCount = leftCount;
    topBvhNodes[rightChildIdx].firstPrimIdx = i;
    topBvhNodes[rightChildIdx].primCount = topBvhNodes[nodeIdx].primCount - leftCount;
    topBvhNodes[nodeIdx].leftNodeIdx = leftChildIdx;
    topBvhNodes[nodeIdx].primCount = 0;

    UpdateNodeBounds( leftChildIdx );
    UpdateNodeBounds( rightChildIdx );

    // recurse
    Subdivide( leftChildIdx );
    Subdivide( rightChildIdx );
}

void BuildAllMeshBVHs(const Scene& scene)
{
    meshBVHs.resize(scene.meshes.size());
    
    for (size_t i = 0; i < scene.meshes.size(); i++) {
        BuildMeshBVH(scene, i);
    }
}

void BuildMeshBVH(const Scene& scene, size_t meshIdx)
{
    const Mesh& mesh = scene.meshes[meshIdx];
    MeshBVH& bvh = meshBVHs[meshIdx];
    
    // Make primitives array from mesh faces
    MakeMeshPrimsArray(mesh, scene.vertex_data, bvh);
    
    uint32_t N = bvh.prims.size();
    if (N == 0) return;  // Empty mesh

    bvh.nodes.resize(2*N);
    bvh.primIdx.resize(2*N);
    
    // Initialize index array
    for (uint32_t i = 0; i < N; i++) {
        bvh.primIdx[i] = i;
    }
    
    // Assign all prims to root node
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
        
        // Get triangle vertices
        const Vec3f& v0 = vertex_data[face.i0 - 1].pos;
        const Vec3f& v1 = vertex_data[face.i1 - 1].pos;
        const Vec3f& v2 = vertex_data[face.i2 - 1].pos;
        
        // Compute triangle AABB
        AABB box;
        box.reset();
        box.expand(v0);
        box.expand(v1);
        box.expand(v2);
        
        // Compute centroid
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
    // DON'T hold a reference at the top - it becomes invalid after resize!
    
    // Terminate recursion
    if (bvh.nodes[nodeIdx].primCount <= 2) return;
    
    // Determine split axis and position
    Vec3f extent = bvh.nodes[nodeIdx].bounds.max - bvh.nodes[nodeIdx].bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float splitPos = bvh.nodes[nodeIdx].bounds.min[axis] + extent[axis] * 0.5f;
    
    // In-place partition
    int i = bvh.nodes[nodeIdx].firstPrimIdx;
    int j = i + bvh.nodes[nodeIdx].primCount - 1;
    while (i <= j)
    {
        if (bvh.prims[bvh.primIdx[i]].centroid[axis] < splitPos)
            i++;
        else
            std::swap(bvh.primIdx[i], bvh.primIdx[j--]);
    }
    
    // Abort split if one of the sides is empty
    int leftCount = i - bvh.nodes[nodeIdx].firstPrimIdx;
    if (leftCount == 0 || leftCount == bvh.nodes[nodeIdx].primCount) return;
    
    // Create child nodes - RESIZE IF NEEDED
    if (bvh.nodesUsed + 2 >= bvh.nodes.size()) {
        bvh.nodes.resize(bvh.nodes.size() * 2);  // This invalidates references!
    }
    
    int leftChildIdx = bvh.nodesUsed++;
    int rightChildIdx = bvh.nodesUsed++;
    
    // Now it's safe to access and modify
    bvh.nodes[leftChildIdx].firstPrimIdx = bvh.nodes[nodeIdx].firstPrimIdx;
    bvh.nodes[leftChildIdx].primCount = leftCount;
    bvh.nodes[rightChildIdx].firstPrimIdx = i;
    bvh.nodes[rightChildIdx].primCount = bvh.nodes[nodeIdx].primCount - leftCount;
    
    bvh.nodes[nodeIdx].leftNodeIdx = leftChildIdx;
    bvh.nodes[nodeIdx].primCount = 0;
    
    UpdateMeshNodeBounds(bvh, leftChildIdx);
    UpdateMeshNodeBounds(bvh, rightChildIdx);
    
    // Recurse
    SubdivideMesh(bvh, leftChildIdx);
    SubdivideMesh(bvh, rightChildIdx);
}

Ray ComputeRay(const Scene& scene, const Camera& camera, int j, int i) noexcept
{   
    Vec3f e = camera.position;
    Vec3f m = e - camera.w * camera.near_distance;
    Vec3f q = m + camera.u * camera.near_plane.l + camera.v * camera.near_plane.t;
    
    float su = (j + 0.5) * (camera.near_plane.r - camera.near_plane.l) / camera.width;  
    float sv = (i + 0.5) * (camera.near_plane.t - camera.near_plane.b) / camera.height;  

    Vec3f s = q + camera.u * su - camera.v * sv;

    Ray ray = {e, (s - e).normalize(), 0};

    return ray;
}

Vec3f ComputeColor(const Ray& ray, const Scene& scene, const Camera& camera) 
{
    if (ray.depth > scene.max_recursion_depth) {
        return {0, 0, 0};
    }

    HitRecord closestHit;
    if (FindClosestHit(ray, scene, camera, /*ref*/ closestHit))
    {
        return ApplyShading(ray, scene, camera, closestHit);
    }
    else if (ray.depth == 0)
    {
        return {(float) scene.background_color.x, (float) scene.background_color.y, (float) scene.background_color.z};
    }
    else
    {
        return {0, 0, 0};
    } 
}

bool FindClosestHit(const Ray& ray, const Scene& scene, const Camera& camera, HitRecord& closestHit) noexcept
{
    float minT = FLT_MAX;
    bool has_intersected = false;
    PrimKind closestType;
    int closestMatId;
    float bary_beta, bary_gamma;
    int index;
    
    // Misc vars to find normal at the end
    Face hitTriFace;
    Plane closestPlane;
    Sphere closestSphere;
    bool closest_is_smooth = false;
    
    // 1. Test all planes first (not in BVH - infinite primitives)
    for (size_t i = 0; i < scene.planes.size(); i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(ray, plane.n_unit, plane.plane_d, minT);
        
        if (t < minT && t > 0 && t != RAY_MISS_VALUE) {
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
                    index, hitTriFace, closestSphere, closest_is_smooth, 
                    bary_beta, bary_gamma);
    
    if (!has_intersected) {
        return false;
    }
    
    // 3. Compute hit point and normal
    Vec3f hit_x = ray.origin + ray.direction * minT;
    Vec3f normal;
    
    switch(closestType) {
        case PrimKind::Mesh: {
            if (closest_is_smooth) {
                const Vec3f& nA = scene.vertex_data[hitTriFace.i0 - 1].normal;
                const Vec3f& nB = scene.vertex_data[hitTriFace.i1 - 1].normal;
                const Vec3f& nC = scene.vertex_data[hitTriFace.i2 - 1].normal;
                
                float alpha = 1.f - bary_beta - bary_gamma;
                normal = (nA * alpha + nB * bary_beta + nC * bary_gamma).normalize();
            }
            else {
                normal = hitTriFace.n_unit;
            }
            break;
        }
        case PrimKind::Triangle: {
            normal = hitTriFace.n_unit;
            break;
        }
        case PrimKind::Sphere: {
            Vertex center = scene.vertex_data[closestSphere.center_vertex_id - 1];
            normal = FindNormal_Sphere(center, hit_x, closestSphere.radius);
            break;
        }
        case PrimKind::Plane: {
            normal = closestPlane.n_unit;
            break;
        }
        default: {
            normal = {0, 0, 0};
        }
    }
    
    closestHit = {closestMatId, hit_x, normal, closestType};
    return true;
}

void IntersectTopBVH(const Ray& ray, const Scene& scene, float& minT, bool& has_intersected,
                     PrimKind& closestType, int& closestMatId, int& index,
                     Face& hitTriFace, Sphere& closestSphere, bool& closest_is_smooth,
                     float& bary_beta, float& bary_gamma) noexcept
{
    // Stack for iterative traversal
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        BVHNode& node = topBvhNodes[nodeIdx];
        
        // Test AABB intersection
        if (IntersectAABB(ray, node.bounds, minT) == RAY_MISS_VALUE) {
            continue;
        }
        
        if (node.primCount > 0)  // Leaf node
        {
            // Test all primitives in this leaf
            for (uint32_t i = 0; i < node.primCount; i++)
            {
                uint32_t primIdx = topPrimIdx[node.firstPrimIdx + i];
                TopPrim& prim = topPrims[primIdx];
                
                switch(prim.kind)
                {
                    case PrimKind::Mesh: {
                        const Mesh& mesh = scene.meshes[prim.index];
                        float temp_b, temp_g;
                        Face tempFace;
                        
                        // Traverse mesh BVH
                        float t = IntersectMeshBVH(ray, mesh, scene, prim.index, 
                                                   minT, tempFace, temp_b, temp_g);
                        
                        if (t < minT && t > 0 && t != RAY_MISS_VALUE) {
                            minT = t;
                            closestMatId = mesh.material_id;
                            closestType = PrimKind::Mesh;
                            hitTriFace = tempFace;
                            closest_is_smooth = mesh.is_smooth;
                            bary_beta = temp_b;
                            bary_gamma = temp_g;
                            index = prim.index;
                            has_intersected = true;
                        }
                        break;
                    }
                    
                    case PrimKind::Sphere: {
                        const Sphere& sphere = scene.spheres[prim.index];
                        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
                        float t = IntersectSphere(ray, center, sphere.radius, minT);
                        
                        if (t < minT && t > 0 && t != RAY_MISS_VALUE) {
                            minT = t;
                            closestMatId = sphere.material_id;
                            closestType = PrimKind::Sphere;
                            closestSphere = sphere;
                            index = prim.index;
                            has_intersected = true;
                        }
                        break;
                    }
                    
                    case PrimKind::Triangle: {
                        const Triangle& triangle = scene.triangles[prim.index];
                        float dummy_b, dummy_g;
                        float t = IntersectsTriangle_Bary(ray, triangle.face, 
                                                          scene.vertex_data, minT, 
                                                          dummy_b, dummy_g);
                        
                        if (t < minT && t > 0 && t != RAY_MISS_VALUE) {
                            minT = t;
                            closestMatId = triangle.material_id;
                            closestType = PrimKind::Triangle;
                            hitTriFace = triangle.face;
                            index = prim.index;
                            has_intersected = true;
                        }
                        break;
                    }
                }
            }
        }
        else  // Interior node
        {
            // Push children onto stack (right first, so left is processed first)
            uint32_t leftChild = node.leftNodeIdx;
            uint32_t rightChild = node.leftNodeIdx + 1;
            
            stack[stackPtr++] = rightChild;
            stack[stackPtr++] = leftChild;
        }
    }
}

float IntersectMeshBVH(const Ray& ray, const Mesh& mesh, const Scene& scene, 
                       size_t meshIdx, float minT, Face& hitFace, 
                       float& beta_out, float& gamma_out) noexcept
{
    const MeshBVH& bvh = meshBVHs[meshIdx];
    float meshMinT = minT;
    bool found = false;
    
    // Stack for iterative traversal
    uint32_t stack[64];
    uint32_t stackPtr = 0;
    stack[stackPtr++] = bvh.rootNodeIdx;
    
    while (stackPtr > 0)
    {
        uint32_t nodeIdx = stack[--stackPtr];
        const BVHNode& node = bvh.nodes[nodeIdx];
        
        // Test AABB intersection
        if (IntersectAABB(ray, node.bounds, meshMinT) == RAY_MISS_VALUE) {
            continue;
        }
        
        if (node.primCount > 0)  // Leaf node
        {
            // Test all triangles in this leaf
            for (uint32_t i = 0; i < node.primCount; i++)
            {
                uint32_t primIdx = bvh.primIdx[node.firstPrimIdx + i];
                const MeshPrim& meshPrim = bvh.prims[primIdx];
                const Face& face = mesh.faces[meshPrim.faceIdx];
                
                float temp_b, temp_g;
                float t = IntersectsTriangle_Bary(ray, face, scene.vertex_data, 
                                                  meshMinT, temp_b, temp_g);
                
                if (t < meshMinT && t > 0 && t != RAY_MISS_VALUE) {
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
            // Push children onto stack
            uint32_t leftChild = node.leftNodeIdx;
            uint32_t rightChild = node.leftNodeIdx + 1;
            
            stack[stackPtr++] = rightChild;
            stack[stackPtr++] = leftChild;
        }
    }
    
    return found ? meshMinT : RAY_MISS_VALUE;
}

float IntersectAABB(const Ray& ray, const AABB& box, float minT) noexcept
{
    float tMin = 0.0f;
    float tMax = minT;

    // --- X axis ---
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
    else { // Ray is parallel to X slabs; origin must be inside the slab
        if (ray.origin.x < box.min.x || ray.origin.x > box.max.x) {
            return RAY_MISS_VALUE;
        }
    }

    // --- Y axis ---
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

    // --- Z axis ---
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
    else { // We're starting inside the box or at its boundary
        if (tMax <= 0.0f) {
            return RAY_MISS_VALUE;
        }
        tHit = tMax;
    }
    
    if (tHit >= minT) {
        return RAY_MISS_VALUE;
    }

    return tHit;
}

float IntersectsMesh(const Ray& ray, const Mesh& mesh, const std::vector<Vertex>& vertex_data, float minT, /*out*/ Face& hitFace, /*out*/ float& beta_out, /*out*/ float& gamma_out) noexcept
{
    float meshMinT = minT, t;
    bool noIntersects = true;

    for (size_t i = 0, n = mesh.faces.size(); i < n; i++) {
        const Face& currTriFace = mesh.faces[i];
        float temp_b, temp_g;
        float t = IntersectsTriangle_Bary(ray, currTriFace, vertex_data, meshMinT, temp_b, temp_g);
        if (t < meshMinT && t != RAY_MISS_VALUE) {
            meshMinT = t;
            noIntersects = false;
            hitFace = currTriFace;
            beta_out = temp_b;
            gamma_out = temp_g;
        }
    }

    if (noIntersects) {
        return RAY_MISS_VALUE;
    }

    return meshMinT;
}

float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const std::vector<Vertex>& vertex_data, float minT, /*out*/ float& beta_out, /*out*/ float& gamma_out) noexcept
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

    // ray is parallel to the plane
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
    float t1 = (-B - sqrtDelta) / (2 * A);  // Near intersection
    float t2 = (-B + sqrtDelta) / (2 * A);  // Far intersection
    
    // Find the smallest positive t
    float t;
    if (t1 > 0.0f) {
        t = t1;  // Use near intersection if it's in front
    } else if (t2 > 0.0f) {
        t = t2;  // Otherwise use far intersection (we're inside sphere)
    } else {
        return RAY_MISS_VALUE;  // Both behind us
    }
    
    if (t >= minT) {
        return RAY_MISS_VALUE;  // Too far or not closer than previous hit
    }
    
    return t;
}

float IntersectsPlane(const Ray& ray, const Vec3f& normal, float plane_d, float minT) noexcept
{
    float t_formula_denom = ray.direction.dotProduct(normal);

    // ray is parallel to the plane
    if (std::fabs(t_formula_denom) < EPS_PARALLEL)
    { 
        return RAY_MISS_VALUE;
    }

    float t = -(ray.origin.dotProduct(normal) + plane_d) / t_formula_denom;

    // intersection is behind the ray origin or we have closer intersection
    if (t < 0.0f || t >= minT) 
    { 
        return RAY_MISS_VALUE;
    }

    return t;
}

Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius) noexcept
{
   return ((point - center.pos) * (1 / radius)).normalize();
}

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit)
{
    Material mat = scene.materials[closestHit.materialId - 1];
    Vec3f color = {0, 0, 0};

    Vec3f n_original = closestHit.normal;
    Vec3f d_inc = ray.direction;

    Vec3f n_shading = n_original;

    if (n_original.dotProduct(d_inc) < 0.0f) // front-face
    {
        n_shading = n_original;
    }
    else // back-face
    {
        n_shading = n_original * -1;
    }

    Vec3f x = closestHit.intersectionPoint;
    float eps_shift = scene.shadow_ray_epsilon;

    Vec3f w0 = (ray.origin - x).normalize();

    if (mat.type == MaterialType::Mirror)
    {
        Vec3f wr = (n_shading * 2 * (n_shading.dotProduct(w0)) - w0).normalize();
        const Ray reflectionRay = {x + n_shading * eps_shift, wr, ray.depth + 1};
        color += mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera));
    }
    else if (mat.type == MaterialType::Conductor)
    {
        Vec3f wr = (n_shading * 2 * (n_shading.dotProduct(w0)) - w0).normalize();
        const Ray reflectionRay = {x + n_shading * eps_shift, wr, ray.depth + 1};

        float cosTheta = w0.dotProduct(n_shading);
        float Fresnel_r = Fresnel_Conductor(cosTheta, mat.refraction_index, mat.absorption_index);

        color += Fresnel_r * mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera));
    }
    else if (mat.type == MaterialType::Dielectric)
    {
        // DON'T use n_shading here - work with n_original directly
        Vec3f d_inc = ray.direction;
        
        // Check entering/exiting using the ORIGINAL normal
        bool entering = n_original.dotProduct(d_inc) < 0.0f;
        
        // Flip the normal to point against the ray (for consistent math)
        Vec3f normal = entering ? n_original : (n_original * -1.0f);
        
        // Now w0 should point back along the ray
        Vec3f w0 = (ray.origin - x).normalize();
        
        float etaI = entering ? 1.0f : mat.refraction_index;
        float etaT = entering ? mat.refraction_index : 1.0f;
        float eta = etaI / etaT;
        
        float cosThetaI = w0.dotProduct(normal);
        float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
        float sin2ThetaT = eta * eta * sin2ThetaI;
        
        // Total internal reflection
        if (sin2ThetaT >= 1.0f) {
            Vec3f wr = (normal * 2.0f * cosThetaI - w0).normalize();
            Ray reflectionRay = {x + normal * eps_shift, wr, ray.depth + 1};
            color += mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera));
        }
        else {
            float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
            float Fr = Fresnel_Dielectric(cosThetaI, cosThetaT, etaI, etaT);
            
            // Reflection
            Vec3f wr = (normal * 2.0f * cosThetaI - w0).normalize();
            Ray reflectionRay = {x + normal * eps_shift, wr, ray.depth + 1};
            Vec3f reflectColor = mat.mirror_refl.elwiseMult(ComputeColor(reflectionRay, scene, camera));
            
            // Refraction  
            Vec3f wt = ((w0 * -1.0f) * eta + normal * (eta * cosThetaI - cosThetaT)).normalize();
            Ray refractionRay = {x - normal * eps_shift, wt, ray.depth + 1};
            Vec3f refractColor = ComputeColor(refractionRay, scene, camera);
            
            // Apply Beer's Law when exiting
            if (!entering) {
                float d = (x - ray.origin).length();
                refractColor.x *= std::exp(-mat.absorption_coef.x * d);
                refractColor.y *= std::exp(-mat.absorption_coef.y * d);
                refractColor.z *= std::exp(-mat.absorption_coef.z * d);
            }
            
            color += Fr * reflectColor + (1.0f - Fr) * refractColor;
        }
        
        // Only add local lighting when entering
        if (!entering) {
            return color;
        }
    }

    color += scene.ambient_light.elwiseMult(mat.ambient_refl);

    // Compute local lighting (Blinn-Phong) for all materials except when exiting dielectrics
    for (const auto& point_light: scene.point_lights)
    {
        if (!InShadow(x, point_light, n_shading, eps_shift, scene))
        {
            color += ComputeDiffuseAndSpecular(ray.origin, mat, point_light, closestHit.intersectionPoint, n_shading, w0);
        }
    }

    return color;
}

float Fresnel_Dielectric(float cosTheta, float cosPhi, float n1, float n2) noexcept
{
    float r_par_num = n2 * cosTheta - n1 * cosPhi;
    float r_par_den = n2 * cosTheta + n1 * cosPhi;
    float R_paralel = r_par_num / r_par_den;

    float r_perp_num = n1 * cosTheta - n2 * cosPhi;
    float r_perp_den = n1 * cosTheta + n2 * cosPhi;
    float R_perp = r_perp_num / r_perp_den;

    return (R_paralel * R_paralel + R_perp * R_perp) * 0.5;
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

    return (R_S + R_P) * 0.5;
}

bool InShadow(const Vec3f& point, const PointLight& I, const Vec3f& n, float eps_shadow, const Scene& scene) noexcept
{
    Ray shadowRay;
    shadowRay.origin = point + n * eps_shadow;
    shadowRay.direction = I.position - shadowRay.origin;  // Unnormalized
    shadowRay.depth = 0;

    float minT = 1.0f;  // At t=1 we're at the light

    // 1. Test planes first
    for (size_t i = 0; i < scene.planes.size(); i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(shadowRay, plane.n_unit, plane.plane_d, minT);
        if (t < minT && t != RAY_MISS_VALUE) {  // REMOVED t > 0 check
            return true;
        }
    }

    // 2. Traverse top-level BVH
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
                        float temp_b, temp_g;
                        Face tempFace;
                        
                        float t = IntersectMeshBVH(shadowRay, mesh, scene, prim.index, 
                                                   minT, tempFace, temp_b, temp_g);
                        
                        if (t < minT && t != RAY_MISS_VALUE) {  // REMOVED t > 0
                            return true;
                        }
                        break;
                    }
                    
                    case PrimKind::Sphere: {
                        const Sphere& sphere = scene.spheres[prim.index];
                        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
                        float t = IntersectSphere(shadowRay, center, sphere.radius, minT);
                        
                        if (t < minT && t != RAY_MISS_VALUE) {  // REMOVED t > 0
                            return true;
                        }
                        break;
                    }
                    
                    case PrimKind::Triangle: {
                        const Triangle& triangle = scene.triangles[prim.index];
                        float dummy_b, dummy_g;
                        float t = IntersectsTriangle_Bary(shadowRay, triangle.face, 
                                                          scene.vertex_data, minT, 
                                                          dummy_b, dummy_g);
                        
                        if (t < minT && t != RAY_MISS_VALUE) {  // REMOVED t > 0
                            return true;
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
    
    return false;
}

Vec3f ComputeDiffuseAndSpecular(const Vec3f& origin, const Material& material, const PointLight& light, 
    const Vec3f& point, const Vec3f& normal, const Vec3f& w0) noexcept
{
    Vec3f L = light.position - point; // Unnormalized vector
    Vec3f wi = L.normalize();         // Normalized direction

    // theta is angle bw wi and normal
    float cos_theta = wi.dotProduct(normal);

    if (cos_theta < 0.0f)
    {
        return {0.0f, 0.0f, 0.f};
    }

    float d_sq = L.dotProduct(L);             // Squared distance d^2
    Vec3f irradiance = light.intensity / d_sq;

    Vec3f diffuseTerm = (material.diffuse_refl * cos_theta).elwiseMult(irradiance);

    Vec3f h = (wi + w0).normalize();

    // theta is angle bw normal and h
    float cos_alpha = normal.dotProduct(h);

    if (cos_alpha < 0.0f)
    {
        return diffuseTerm;
    }

    // Note: if theta > 90, specularTerm should be 0. we do this check and early exit (with setting terms to 0) above
    Vec3f specularTerm = (material.specular_refl * std::pow(cos_alpha, material.phong_exponent)).elwiseMult(irradiance);

    return diffuseTerm + specularTerm;
}
