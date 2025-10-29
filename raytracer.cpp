#include "raytracer.h"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: %s <scene.json>\n", argv[0]);
        return -1;
    }
    
    parser p;
    string jsonFile = argv[1];
    Scene scene = p.loadFromJson(jsonFile);

    for (Camera camera : scene.cameras)
    {
        int width = camera.width;
        int height = camera.height;
        unsigned char *image = new unsigned char [width * height * 3];
        
        int idx = 0;
        HitRecord closestHit;

        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                Ray ray = ComputeRay(scene, camera, j, i); 
                Vec3f color = ComputeColor(ray, scene, camera);

                image[idx++] = (unsigned char) clampF(color.x, 0.0f, 255.0f);
                image[idx++] = (unsigned char) clampF(color.y, 0.0f, 255.0f);
                image[idx++] = (unsigned char) clampF(color.z, 0.0f, 255.0f);         
            }
        }
        
        stbi_write_png(camera.image_name.c_str(), width, height, 3, image, width * 3);
    }

    return 0;
}

Ray ComputeRay(Scene scene, Camera camera, int j, int i)
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

bool FindClosestHit(const Ray& ray, const Scene& scene, const Camera& camera, /*out*/ HitRecord& closestHit)
{
    float minT = FLT_MAX, hitT;
    bool has_intersected = false; // means obj is null
    ObjectType closestType;
    int closestMaterialId;

    // misc vars to find normal at the end
    Face hitTriFace;
    Plane closestPlane;
    Sphere closestSphere;

    for (size_t i = 0, n = scene.meshes.size(); i < n; i++) {
        const Mesh& mesh = scene.meshes[i];
        float baryBeta, baryGamma;
        float t = IntersectsMesh(ray, mesh, scene.vertex_data, minT, /*ref*/ hitTriFace);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE)
        {
            hitT = t;
            minT = hitT;
            closestMaterialId = mesh.material_id;
            closestType = ObjectType::Mesh;
            // hitTriFace set by IntersectsMesh
            has_intersected = true;
        }
    }

    for (size_t i = 0, n = scene.triangles.size(); i < n; i++) {
        const Triangle& triangle = scene.triangles[i];
        float t = IntersectsTriangle_Bary(ray, triangle.face, scene.vertex_data, minT);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE)
        {
            hitT = t;
            minT = hitT;
            closestMaterialId = triangle.material_id;
            closestType = ObjectType::Triangle;
            hitTriFace = triangle.face;
            has_intersected = true;
        }
    }

    for(size_t i = 0, n = scene.spheres.size(); i < n; i++) {
        const Sphere& sphere = scene.spheres[i];
        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
        float t = IntersectSphere(ray, center, sphere.radius, minT);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE) 
        {
            hitT = t;
            minT = hitT;
            closestMaterialId = sphere.material_id;
            closestType = ObjectType::Sphere;
            closestSphere = sphere;
            has_intersected = true;
        }
    }

    for (size_t i = 0, n = scene.planes.size(); i < n; i++) {
        const Plane& plane = scene.planes[i];
        Vertex center_point = scene.vertex_data[plane.vertex_id - 1];
        float t = IntersectsPlane(ray, plane.n_face, center_point, minT);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE){
            hitT = t;
            minT = hitT;
            closestMaterialId = plane.material_id;
            closestType = ObjectType::Plane;
            closestPlane = plane;
            has_intersected = true;
        }
    }
    
    if (!has_intersected)
    {
        return false;
    }

    // since intersection happened, closestType and minT is set correctly
    Vec3f hit_x = ray.origin + ray.direction * minT;
    Vec3f normal;

    switch(closestType) {
        case ObjectType::Mesh:  // TODO: Use vertex normal for smooth mesh
        case ObjectType::Triangle: {
            normal = hitTriFace.n_face;
            break;
        }
        case ObjectType::Sphere: {
            Vertex center = scene.vertex_data[closestSphere.center_vertex_id - 1];
            normal = FindNormal_Sphere(center, hit_x, closestSphere.radius);
            break;
        }
        case ObjectType::Plane: {
            normal = closestPlane.n_face;
            break;
        }
        default: {
            normal = {0, 0, 0};
        }
    }

    closestHit = {closestMaterialId, hit_x, normal, closestType};
    return true;
}

// TODO: store bary vars
float IntersectsMesh(const Ray& ray, const Mesh& mesh, const std::vector<Vertex>& vertex_data, const float& minT, /*out*/ Face& hitFace) 
{
    float meshMinT = minT;
    float t, currBaryBeta, currBaryGamma;
    bool noIntersects = true;

    for (size_t i = 0, n = mesh.faces.size(); i < n; i++) {
        const Face& currTriFace = mesh.faces[i];
        float t = IntersectsTriangle_Bary(ray, currTriFace, vertex_data, meshMinT);
        if (t < meshMinT && t != RAY_MISS_VALUE) {
            meshMinT = t;
            noIntersects = false;
            hitFace = currTriFace;
        }
    }

    if (noIntersects) {
        return RAY_MISS_VALUE;
    }

    return meshMinT;
}

float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const std::vector<Vertex>& vertex_data, const float& minT) 
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

    return t;
}

float IntersectSphere(const Ray& ray, const Vertex& center, float radius, const float& minT) 
{
    Vec3f oc =  ray.origin - center.pos;
    Vec3f d = ray.direction;
    float A = d.dotProduct(d);
    float B = 2 * d.dotProduct(oc);
    float C = oc.dotProduct(oc) - radius * radius;
    float delta = B * B - 4 * A * C;
    if (delta < 0.0f) {
        return  RAY_MISS_VALUE;
    }
    if (fabs(delta) < EPS_PARALLEL && delta >= 0.0f) {
        float t = (-B + sqrt(delta)) / (2 * A); 
        if (t < 0.0f || t >= minT) 
        { 
            return RAY_MISS_VALUE;
        }
        return t;
    }
    float t1 = (-B + sqrt(delta)) / (2 * A);
    float t2 = (-B - sqrt(delta)) / (2 * A);
    t1 = std::min(t1, t2);
    if (t1 < 0.0f || t1 >= minT) 
    { 
        return RAY_MISS_VALUE;
    }
    return t1;
}

// TODO: use plane_d
float IntersectsPlane(const Ray& ray, const Vec3f& normal, const Vertex& point_on_plane, const float& minT) 
{
    float t_formula_denom = ray.direction.dotProduct(normal);

    // ray is parallel to the plane
    if (std::fabs(t_formula_denom) < EPS_PARALLEL)
    { 
        return RAY_MISS_VALUE;
    }

    float t = (point_on_plane.pos - ray.origin).dotProduct(normal) / t_formula_denom;

    // intersection is behind the ray origin or we have closer intersection
    if (t < 0.0f || t >= minT) 
    { 
        return RAY_MISS_VALUE;
    }

    return t;
}

Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius)
{
   return ((point - center.pos) * (1 / radius)).normalize();
}

Vec3f ApplyShading(const Ray& ray, const Scene& scene, const Camera& camera, const HitRecord& closestHit)
{   
    // TODO: implement
    return {0, 0, 0};
}