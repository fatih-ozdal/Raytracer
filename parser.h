#ifndef PARSER_H
#define PARSER_H

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "Vec3f.h"
#include "Aabb.h"

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
    AABB localBounds;
};

struct Triangle
{
    int material_id;
    Face face;
    AABB localBounds;
};

struct Sphere
{
    int material_id;
    int center_vertex_id;
    float radius;
    AABB localBounds;
};

struct Plane 
{
    int material_id;
    int vertex_id;
    Vec3f n_unit;
    float plane_d;
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
    vector<int> loadFromPly(const string &filepath);
private:
    // Small parser helpers
    static Vec3f parseVec3f(const std::string& s);
    static float parseFloat(const std::string& s);
    static std::vector<std::array<int,3>> load_ply_faces(const std::string& ply_path);
    static std::string flatten_faces_to_string(const std::vector<std::array<int,3>>& tris);
    static std::string join_with_json_dir(const std::string& scene_path, const std::string& rel_or_abs);
    static PlyData load_ply(const std::string& path);
};

#endif // PARSER_H
