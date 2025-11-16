#ifndef PARSER_H
#define PARSER_H

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "Vec3f.h"
#include "Aabb.h"
#include "Mat4f.h"

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
    int width, height;
    string image_name;
};

struct PointLight
{
    Vec3f position;
    Vec3f intensity;
    PointLight() : position(0, 0, 0), intensity(0, 0, 0) {}
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
};

struct Vertex 
{
    Vec3f pos;          // position
    Vec3f normal;       // per-vertex shading normal (optional)
};

struct Face
{
    int i0, i1, i2;     // vertex indices
    Vec3f n_unit;       // is a unit vector
    float plane_d;
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
    
    // Instancing support
    bool isInstance;          // true if this is an instance of another mesh
    int originalMeshId;       // Index of original mesh (-1 if this IS the original)
    
    // BVH
    int bvhIndex;             // Index into meshBVHs array (-1 if not built)
    AABB localBounds;         // Bounds in object space
    AABB worldBounds;         // Bounds in world space (after transformation)
    
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
    
    AABB localBounds;
    AABB worldBounds;
    
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
    
    AABB localBounds;
    AABB worldBounds;
    
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
    
    Plane() : material_id(-1), vertex_id(-1), n_unit(0, 0, 1), plane_d(0), hasTransform(false) {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};

struct Translation {
    int id;
    Vec3f delta;
};

struct Scaling {
    int id;
    Vec3f scale;
};

struct Rotation {
    int id;
    float angle;  // degrees
    Vec3f axis;
};

struct Composite {
    int id;
    Mat4f matrix;
};

struct Scene
{
    // Data
    Vec3f background_color;
    float shadow_ray_epsilon;
    int max_recursion_depth;
    vector<Camera> cameras;
    Vec3f ambient_light;
    vector<PointLight> point_lights;
    vector<Material> materials;
    vector<Vertex> vertex_data;
    vector<Mesh> meshes;
    vector<Triangle> triangles;
    vector<Sphere> spheres;
    vector<Plane> planes;
    
    // Transformations
    vector<Translation> translations;
    vector<Scaling> scalings;
    vector<Rotation> rotations;
    vector<Composite> composites;
};

struct PlyData {
    std::vector<Vec3f> verts;
    std::vector<Vec3f> normals;
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
    static std::vector<std::array<int,3>> load_ply_faces(const std::string& ply_path);
    static std::string flatten_faces_to_string(const std::vector<std::array<int,3>>& tris);
    static std::string join_with_json_dir(const std::string& scene_path, const std::string& rel_or_abs);
    static PlyData load_ply(const std::string& path);
};

#endif // PARSER_H
