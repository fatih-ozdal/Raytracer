#ifndef PARSER_H
#define PARSER_H

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "Vec3f.h"

using json = nlohmann::json;
using std::vector;
using std::string;

struct LRBT_vec
{
    float l, r, b, t;
};

enum class CameraType : uint32_t
{
    None = 0,
    LookAt,
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
    Vec3f n_face;
    float plane_d;
};

struct Mesh
{
    bool is_smooth;
    int material_id;
    vector<Face> faces;
};

struct Triangle
{
    int material_id;
    Face face;
};

struct Sphere
{
    int material_id;
    int center_vertex_id;
    float radius;
};

struct Plane 
{
    int material_id;
    int vertex_id;
    Vec3f n_face;
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
};

#endif // PARSER_H
