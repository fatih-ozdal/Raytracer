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

        #pragma omp parallel
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
    int closestMatId;
    float bary_beta, bary_gamma;
    int index;

    // misc vars to find normal at the end
    Face hitTriFace;
    Plane closestPlane;
    Sphere closestSphere;
    bool closest_is_smooth = false;

    for (size_t i = 0, n = scene.meshes.size(); i < n; i++) {
        const Mesh& mesh = scene.meshes[i];
        float temp_b, temp_g;
        float t = IntersectsMesh(ray, mesh, scene.vertex_data, minT, /*ref*/ hitTriFace, /*ref*/ temp_b, /*ref*/ temp_g);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE)
        {
            hitT = t;
            minT = hitT;
            closestMatId = mesh.material_id;
            closestType = ObjectType::Mesh;
            // hitTriFace set by IntersectsMesh
            has_intersected = true;
            closest_is_smooth = mesh.is_smooth;
            bary_beta = temp_b;
            bary_gamma = temp_g;
            index = i;
        }
    }

    for (size_t i = 0, n = scene.triangles.size(); i < n; i++) {
        const Triangle& triangle = scene.triangles[i];
        float dummy_b, dummy_g;
        float t = IntersectsTriangle_Bary(ray, triangle.face, scene.vertex_data, minT, /*ref*/ dummy_b, /*ref*/ dummy_g);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE)
        {
            hitT = t;
            minT = hitT;
            closestMatId = triangle.material_id;
            closestType = ObjectType::Triangle;
            hitTriFace = triangle.face;
            has_intersected = true;
            index = i;
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
            closestMatId = sphere.material_id;
            closestType = ObjectType::Sphere;
            closestSphere = sphere;
            has_intersected = true;
            index = i;
        }
    }

    for (size_t i = 0, n = scene.planes.size(); i < n; i++) {
        const Plane& plane = scene.planes[i];
        float t = IntersectsPlane(ray, plane.n_face, plane.plane_d, minT);

        if (t < minT && t > 0 && t != RAY_MISS_VALUE){
            hitT = t;
            minT = hitT;
            closestMatId = plane.material_id;
            closestType = ObjectType::Plane;
            closestPlane = plane;
            has_intersected = true;
            index = i;
        }
    }
    
    if (!has_intersected)
    {
        return false;
    }

    // since intersection happened, closestType and minT is set correctly
    Vec3f hit_x = ray.origin + ray.direction * minT;
    Vec3f normal;

    // find normal to store in hit record
    switch(closestType) {
        case ObjectType::Mesh: {
            if (closest_is_smooth) {
                const Vec3f& nA = scene.vertex_data[hitTriFace.i0 - 1].normal;
                const Vec3f& nB = scene.vertex_data[hitTriFace.i1 - 1].normal;
                const Vec3f& nC = scene.vertex_data[hitTriFace.i2 - 1].normal;

                float alpha = 1.f - bary_beta - bary_gamma;
                normal = (nA * alpha + nB * bary_beta + nC * bary_gamma).normalize();
            }
            else {
                normal = hitTriFace.n_face;
            }
            break;
        }
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

    closestHit = {closestMatId, hit_x, normal, closestType, index};
    return true;
}

float IntersectsMesh(const Ray& ray, const Mesh& mesh, const std::vector<Vertex>& vertex_data, const float& minT, /*out*/ Face& hitFace, /*out*/ float& beta_out, /*out*/ float& gamma_out) 
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

float IntersectsTriangle_Bary(const Ray& ray, const Face& tri_face, const std::vector<Vertex>& vertex_data, const float& minT, /*out*/ float& beta_out, /*out*/ float& gamma_out) 
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

float IntersectSphere(const Ray& ray, const Vertex& center, float radius, const float& minT) 
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

float IntersectsPlane(const Ray& ray, const Vec3f& normal, float plane_d, float minT) 
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

Vec3f FindNormal_Sphere(const Vertex& center, const Vec3f& point, float radius)
{
   return ((point - center.pos) * (1 / radius)).normalize();
}

#include <iostream>

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
        std::cout << entering << std::endl;
        
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

float Fresnel_Dielectric(float cosTheta, float cosPhi, float n1, float n2)
{
    float r_par_num = n2 * cosTheta - n1 * cosPhi;
    float r_par_den = n2 * cosTheta + n1 * cosPhi;
    float R_paralel = r_par_num / r_par_den;

    float r_perp_num = n1 * cosTheta - n2 * cosPhi;
    float r_perp_den = n1 * cosTheta + n2 * cosPhi;
    float R_perp = r_perp_num / r_perp_den;

    return (R_paralel * R_paralel + R_perp * R_perp) * 0.5;
}

float Fresnel_Conductor(float cosTheta, float refractionIndex, float absorptionIndex)
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

bool InShadow(Vec3f point, const PointLight& I, const Vec3f& n, float eps_shadow, Scene scene) 
{
    Ray shadowRay;
    shadowRay.origin = point + n * eps_shadow;
    shadowRay.direction = I.position - point;
    shadowRay.depth = 0;

    Face hitTriFace;
    float hitT, minT = 1; // at t=1 we are at the light source. so we search for obj that have hitT < 1

    for (size_t i = 0, n = scene.meshes.size(); i < n; i++) {
        const Mesh& mesh = scene.meshes[i];
        float dummy_b, dummy_g;
        hitT = IntersectsMesh(shadowRay, mesh, scene.vertex_data, minT, /*ref*/ hitTriFace, dummy_b, dummy_g);
        if (hitT < minT && hitT != RAY_MISS_VALUE)
        {
            return true;
        }
    }

    for (size_t i = 0, n = scene.triangles.size(); i < n; i++) {
        const Triangle& triangle = scene.triangles[i];
        float dummy_b, dummy_g;
        hitT = IntersectsTriangle_Bary(shadowRay, triangle.face, scene.vertex_data, minT, dummy_b, dummy_g);
        if (hitT < minT && hitT != RAY_MISS_VALUE)
        {
            return true;
        }
    }

    for(size_t i = 0, n = scene.spheres.size(); i < n; i++) {
        const Sphere& sphere = scene.spheres[i];
        const Vertex& center = scene.vertex_data[sphere.center_vertex_id - 1];
        hitT = IntersectSphere(shadowRay, center, sphere.radius, minT);
        if (hitT < minT && hitT != RAY_MISS_VALUE) 
        {
            return true;
        }
    }

    for (size_t i = 0, n = scene.planes.size(); i < n; i++) {
        const Plane& plane = scene.planes[i];
        hitT = IntersectsPlane(shadowRay, plane.n_face, plane.plane_d, minT);
        if (hitT < minT && hitT != RAY_MISS_VALUE){
            return true;
        }
    }

    return false;
}

Vec3f ComputeDiffuseAndSpecular(const Vec3f& origin, const Material& material, const PointLight& light, 
    const Vec3f& point, const Vec3f& normal, const Vec3f& w0)
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
