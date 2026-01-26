#ifndef PARSER_H
#define PARSER_H

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <random>
#include "Vec3f.h"
#include "Aabb.h"
#include "Mat4f.h"
#include "Vec2f.h"
#include "texture.h"

using json = nlohmann::json;
using std::vector;
using std::string;

struct LRBT_vec
{
    float l, r, b, t;
};

struct Camera
{
    Vec3f position;
    Vec3f gaze, up;
    Vec3f u, v, w;

    LRBT_vec near_plane;
    float near_distance;
    Vec3f m, q;

    int num_samples;
    int samples_per_side;
    
    float aperture_size;      // Edge length of square aperture
    float focus_distance;     // Distance to focal plane
    bool has_depth_of_field;
 
    int image_width, image_height;
    float pixel_width, pixel_height;
    string image_name;
};

struct PointLight
{
    Vec3f position;
    Vec3f intensity;
    PointLight() : position(0, 0, 0), intensity(0, 0, 0) {}
};

struct AreaLight {
    Vec3f position;
    Vec3f normal;
    float size;
    Vec3f radiance;
};

enum class MaterialType : uint32_t
{
    None = 0,
    Mirror,
    Conductor,
    Dielectric,
};

struct Material
{
    MaterialType type;
    Vec3f ambient_refl;
    Vec3f diffuse_refl;
    Vec3f specular_refl;
    Vec3f mirror_refl;          // for mirror and conductor
    float phong_exponent;
    float refraction_index;     // for conductor and dielectric (Fresnel)
    float absorption_index;     // for conductor and dielectric (Fresnel)
    Vec3f absorption_coef;      // for dielectric
    float roughness;
};

struct Vertex 
{
    Vec3f pos;          // 3D position
    Vec3f normal;       // Per-vertex shading normal (optional)
    Vec2f uv;           // Texture coordinates (optional)
};

struct Face
{
    int i0, i1, i2;     // vertex indices (1-based)
    int t0, t1, t2;     // texture coordinate indices (1-based, -1 if none)
    Vec3f n_unit;       // is a unit vector
    float plane_d;

    Face() : i0(0), i1(0), i2(0), t0(-1), t1(-1), t2(-1), plane_d(0) {}
};

struct Mesh
{
    bool is_smooth;
    int material_id;
    vector<Face> faces;
    
    // Transformation
    Mat4f transformation;
    Mat4f invTransformation;
    bool hasTransform;

    // Motion Blur
    Vec3f motion_blur;
    bool has_motion_blur;
    
    // Instancing support
    bool isInstance;          // true if this is an instance of another mesh
    int originalMeshId;       // Index of original mesh (-1 if this IS the original)
    
    // BVH
    int bvhIndex;             // Index into meshBVHs array (-1 if not built)
    AABB localBounds;         // Bounds in object space
    AABB worldBounds;         // Bounds in world space (after transformation)

    std::vector<int> texture_ids;
    
    Mesh() : is_smooth(false), material_id(-1), hasTransform(false), 
             isInstance(false), originalMeshId(-1), bvhIndex(-1) {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};

struct Triangle
{
    int material_id;
    Face face;
    
    // Transformation
    Mat4f transformation;
    Mat4f invTransformation;
    bool hasTransform;

    // Motion Blur
    Vec3f motion_blur;
    bool has_motion_blur;
    
    AABB localBounds;
    AABB worldBounds;

    std::vector<int> texture_ids;
    
    Triangle() : material_id(-1), hasTransform(false) {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};

struct Sphere
{
    int material_id;
    int center_vertex_id;
    float radius;
    
    // Transformation
    Mat4f transformation;
    Mat4f invTransformation;
    bool hasTransform;
    
    // Motion Blur
    Vec3f motion_blur;
    bool has_motion_blur;
    
    AABB localBounds;
    AABB worldBounds;
    
    std::vector<int> texture_ids;

    Sphere() : material_id(-1), center_vertex_id(-1), radius(0.0f), hasTransform(false) {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};

struct Plane 
{
    int material_id;
    int vertex_id;
    Vec3f n_unit;
    float plane_d;
    
    // Transformation
    Mat4f transformation;
    Mat4f invTransformation;
    bool hasTransform;
    
    // Motion Blur
    Vec3f motion_blur;
    bool has_motion_blur;

    std::vector<int> texture_ids;
    
    Plane() : material_id(-1), vertex_id(-1), n_unit(0, 0, 1), plane_d(0), hasTransform(false) {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};

struct Scene
{
    // Data
    Vec3f background_color;
    int background_texture_id;

    float shadow_ray_epsilon;
    int max_recursion_depth;
    vector<Camera> cameras;
    Vec3f ambient_light;
    vector<PointLight> point_lights;
    std::vector<AreaLight> area_lights;
    vector<Material> materials;
    vector<Vertex> vertex_data;
    vector<Vec2f> tex_coord_data;  // Separate texture coordinates
    vector<Mesh> meshes;
    vector<Triangle> triangles;
    vector<Sphere> spheres;
    vector<Plane> planes;
    std::unordered_map<int, int> meshIdToIndex;
    
    // Transformations
    vector<Mat4f> translations;
    vector<Mat4f> scalings;
    vector<Mat4f> rotations;
    vector<Mat4f> composites;

    // Textures
    std::vector<ImageData> image_data;
    std::vector<std::unique_ptr<TextureMap>> texture_maps;

    Scene() : background_texture_id(-1) {}
};

struct PlyData {
    std::vector<Vec3f> verts;
    std::vector<Vec3f> normals;
    std::vector<Vec2f> uvs;
    std::vector<std::array<int,3>> faces;
};

struct parser
{
public:
    // API
    Scene loadFromJson(const string &filepath);
    
private:
    // Transformation helpers
    static Mat4f ParseTransformations(const string& transformStr, const Scene& scene);
    static Mat4f MakeTranslation(const Vec3f& t);
    static Mat4f MakeScaling(const Vec3f& s);
    static Mat4f MakeRotation(float angleDegrees, const Vec3f& axis);
    static AABB TransformAABB(const AABB& box, const Mat4f& transform);
    
    // Small parser helpers
    static Vec3f parseVec3f(const std::string& s);
    static float parseFloat(const std::string& s);
    static DecalMode parseDecalMode(const std::string& str);
    static InterpolationMode parseInterpolationMode(const std::string& str);
    static NoiseConversion parseNoiseConversion(const std::string& str);
    static std::vector<std::array<int,3>> load_ply_faces(const std::string& ply_path);
    static std::string flatten_faces_to_string(const std::vector<std::array<int,3>>& tris);
    static std::string join_with_json_dir(const std::string& scene_path, const std::string& rel_or_abs);
    static PlyData load_ply(const std::string& path);
};

#endif // PARSER_H
