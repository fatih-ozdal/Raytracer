#include <fstream>
#include <sstream>
#include "parser.h"
#include <cctype>
#include <algorithm>

#include <array>
#include <string>
#include <vector>
#include <cstdint>
#include <limits>

#include <iostream>
using std::cout;
using std::endl;

// Helper function to trim whitespace from string
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

// TODO: use .value() for optional fields, use at().get() for expected fields (not [].get(), it returns null)
// TODO: use .value() instead of parseFloat()

Scene parser::loadFromJson(const string &filepath)
{
    std::ifstream f(filepath);
    json j = json::parse(f);
    auto s = j["Scene"];

    Scene scene;

    // --- BackgroundColor ---
    if (s.contains("BackgroundColor")) {
        scene.background_color = parser::parseVec3f(
            s["BackgroundColor"].get<std::string>()
        );
    } else {
        scene.background_color = {0, 0, 0};
    }

    // --- ShadowRayEpsilon ---
    if (s.contains("ShadowRayEpsilon")) {
        scene.shadow_ray_epsilon = parser::parseFloat(
            s["ShadowRayEpsilon"].get<std::string>()
        );
    } else {
        scene.shadow_ray_epsilon = 1e-3f;
    }

    // --- MaxRecursionDepth ---
    if (s.contains("MaxRecursionDepth")) {
        scene.max_recursion_depth = std::stoi(s.at("MaxRecursionDepth").get<std::string>());
    } else {
        scene.max_recursion_depth = 0; // or whatever default you want
    }

    // --- Lights ---
    if (s.contains("Lights"))
    {
        json& lights = s["Lights"];

        // --- AmbientLight --- 
        if (lights.contains("AmbientLight")) {
            scene.ambient_light = parser::parseVec3f(
                lights["AmbientLight"].get<std::string>()
            );
        }
        else {
            scene.ambient_light = {0.f, 0.f, 0.f};
        }

        // --- PointLights ---
        if (lights.contains("PointLight")) {
            const auto& node = lights["PointLight"];

            auto parseOnePointLight = [&](const json& pl) {
                PointLight L;
                L.position  = parser::parseVec3f(pl["Position"].get<std::string>());
                L.intensity = parser::parseVec3f(pl["Intensity"].get<std::string>());
                scene.point_lights.push_back(L);
            };

            if (node.is_array()) {
                for (const auto& pl : node) parseOnePointLight(pl);
            } else {
                parseOnePointLight(node);
            }
        }
    }

    // --- Cameras ---
    if (s.contains("Cameras") && s["Cameras"].contains("Camera")) {
        const auto& node = s["Cameras"]["Camera"];

        auto parseOneCamera = [&](const json& cj) {
            Camera cam;
            bool is_lookAt_type = cj.value("_type", "") == "lookAt"; 

            cam.position = parser::parseVec3f(cj["Position"].get<std::string>());
            cam.up = parser::parseVec3f(cj["Up"].get<std::string>());

            if (is_lookAt_type)  {
                Vec3f gaze_point = parser::parseVec3f(cj["GazePoint"].get<std::string>());
                cam.gaze = gaze_point - cam.position;
            } else {
                cam.gaze = parser::parseVec3f(cj["Gaze"].get<std::string>());
            }

            cam.w = -1 * cam.gaze.normalize();
            Vec3f v_prime = cam.up.normalize();
            cam.u = v_prime.crossProduct(cam.w).normalize();
            cam.v = cam.w.crossProduct(cam.u);  // already normalized
            
            // ImageResolution
            std::stringstream ss(cj["ImageResolution"].get<std::string>());
            ss >> cam.width >> cam.height;
            
            // NearDistance
            cam.near_distance = parser::parseFloat(cj["NearDistance"].get<std::string>());

            // NearPlane
            if (is_lookAt_type) {
                float fovY_deg = parser::parseFloat(cj["FovY"].get<std::string>());
                float thetaY = deg2rad(fovY_deg);
                float t = cam.near_distance * std::tan(thetaY * 0.5f);
                float b = -t;

                float aspect = (cam.height > 0) ? (float(cam.width) / float(cam.height)) : 1.0f;
                float r = aspect * t;
                float l = -r;

                cam.near_plane = { l, r, b, t };
            } else {
                std::stringstream ss(cj["NearPlane"].get<std::string>());
                float l=0, r=0, b=0, t=0;
                ss >> l >> r >> b >> t;
                cam.near_plane = {l, r, b, t};
            }

            cam.image_name = cj["ImageName"].get<std::string>();
            scene.cameras.push_back(cam);
        };

        if (node.is_array()) {
            for (const auto& cj : node) parseOneCamera(cj);
        } else {
            parseOneCamera(node);
        }
    }

    // --- Materials ---
    if (s.contains("Materials") && s["Materials"].contains("Material")) {
        const auto& node = s["Materials"]["Material"];

        auto parseOneMaterial = [&](const json& mj) {
            Material mat{};

            std::string t = mj.value("_type", "");
            if (t == "mirror")          mat.type = MaterialType::Mirror;
            else if (t == "conductor")  mat.type = MaterialType::Conductor;
            else if (t == "dielectric") mat.type = MaterialType::Dielectric;
            else  mat.type = MaterialType::None;

            mat.ambient_refl  = parser::parseVec3f(mj.value("AmbientReflectance",  "0 0 0"));
            mat.diffuse_refl  = parser::parseVec3f(mj.value("DiffuseReflectance",  "0 0 0"));
            mat.specular_refl = parser::parseVec3f(mj.value("SpecularReflectance", "0 0 0"));
            mat.mirror_refl   = parser::parseVec3f(mj.value("MirrorReflectance",   "0 0 0"));

            mat.phong_exponent   = parser::parseFloat(mj.value("PhongExponent", "1"));
            mat.refraction_index = parser::parseFloat(mj.value("RefractionIndex", "0"));
            mat.absorption_index = parser::parseFloat(mj.value("AbsorptionIndex", "0"));
            mat.absorption_coef  = parser::parseVec3f(mj.value("AbsorptionCoefficient", "0 0 0"));

            scene.materials.push_back(mat);
        };

        if (node.is_array()) {
            for (const auto& mj : node) parseOneMaterial(mj);
        } else {
            parseOneMaterial(node);
        }
    }

    // --- VertexData ---
    if (s.contains("VertexData"))
    {
        std::string data;
        const auto& vd = s["VertexData"];

        if (vd.is_string()) {
            data = vd.get<std::string>();               
        } else if (vd.is_object()) {
            data = vd.value("_data", "");  // Note: assuming no need to check _type == xyz
        }

        std::stringstream ss(data);
        float x, y, z;

        // TODO: check this is correct
        auto count_tokens = [](const std::string& s) -> size_t {
            size_t tokens = 0;
            bool in_token = false;
            for (unsigned char ch : s) {
                if (std::isspace(ch)) {
                    in_token = false;
                } else {
                    if (!in_token) { ++tokens; in_token = true; }
                }
            }
            return tokens;
        };
        
        scene.vertex_data.reserve(scene.vertex_data.size() + count_tokens(data)/3);

        while (ss >> x >> y >> z)
        {
            Vertex v;
            v.pos = {x, y, z};
            v.normal = {0.f, 0.f, 0.f};
            scene.vertex_data.push_back(v);
        }
    }

    // --- Objects ---
    if (s.contains("Objects"))
    {
        const auto& objects = s["Objects"];

        // --- Meshes ---
        if (objects.contains("Mesh")) {
            const auto& meshNode = objects["Mesh"];

            auto parseOneMesh = [&](const json& mj) {
                Mesh mesh{};
                mesh.is_smooth = mj.value("_shadingMode", "flat") == "smooth";
                mesh.material_id = std::stoi(mj.at("Material").get<std::string>());

                std::string facesStr;
                if (mj.contains("Faces") && mj["Faces"].contains("_plyFile")) {
                    std::string ply_rel = mj["Faces"]["_plyFile"].get<std::string>();
                    std::string ply_path = join_with_json_dir(filepath, ply_rel);
                    auto ply = load_ply(ply_path);

                    // append vertices and remember base
                    size_t base = scene.vertex_data.size();
                    scene.vertex_data.reserve(base + ply.verts.size());
                    
                    bool has_normals = !ply.normals.empty();
                    
                    for (size_t i = 0; i < ply.verts.size(); ++i) {
                        Vertex v;
                        v.pos = ply.verts[i];
                        // Use normals from PLY if available, otherwise zero
                        v.normal = has_normals ? ply.normals[i] : Vec3f{0, 0, 0};
                        scene.vertex_data.push_back(v);
                    }

                    // convert faces to your 1-based global indexing with base offset
                    for (auto& f : ply.faces) {
                        f[0] += int(base);
                        f[1] += int(base);
                        f[2] += int(base);
                    }
                    facesStr = flatten_faces_to_string(ply.faces);

                }
                else if (mj.contains("Faces") && mj["Faces"].contains("_data")) 
                {
                    facesStr = mj.at("Faces").at("_data").get<std::string>();
                }

                std::stringstream ss(facesStr);

                // Track which vertex indices we touched so we normalize only those
                std::vector<int> touched;
                touched.reserve(256);

                auto mark_touched = [&](int idx) {
                    if (touched.empty() || touched.back() != idx) touched.push_back(idx);
                };

                int i0, i1, i2;
                while (ss >> i0 >> i1 >> i2) {
                    Face f{};
                    f.i0 = i0; f.i1 = i1; f.i2 = i2;

                    const Vec3f& va = scene.vertex_data[i0 - 1].pos;
                    const Vec3f& vb = scene.vertex_data[i1 - 1].pos;
                    const Vec3f& vc = scene.vertex_data[i2 - 1].pos;

                    Vec3f n_area = (vb - va).crossProduct(vc - va);
                    Vec3f n_unit = n_area.normalize();

                    // Degenerate triangle, early exit
                    if (n_area.x == 0.f && n_area.y == 0.f && n_area.z == 0.f)
                    {
                        continue;
                    }
                    
                    f.n_face = n_unit;
                    f.plane_d = -n_unit.dotProduct(va);

                    mesh.faces.push_back(f);

                    if (mesh.is_smooth)
                    {
                        // Area-weighted vertex normals: accumulate only from Mesh faces
                        scene.vertex_data[i0 - 1].normal += n_area;
                        scene.vertex_data[i1 - 1].normal += n_area;
                        scene.vertex_data[i2 - 1].normal += n_area;

                        mark_touched(i0);
                        mark_touched(i1);
                        mark_touched(i2);
                    }
                }

                if (mesh.is_smooth)
                {
                    // Normalize the affected vertex normals
                    std::sort(touched.begin(), touched.end());
                    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());

                    for (int vid : touched) {
                        Vec3f& n = scene.vertex_data[vid - 1].normal;
                        n = n.normalize();
                    }
                }

                scene.meshes.push_back(mesh);
            
            };

            if (meshNode.is_array()) {
                for (const auto& mj : meshNode) parseOneMesh(mj);
            } else {
                parseOneMesh(meshNode);
            }
        }

        // --- Triangles ---
        if (objects.contains("Triangle"))
        {
            const auto& triNode = objects["Triangle"];

            auto parseOneTriangle = [&](const json& tj) {
                Triangle tri{};
                tri.material_id = std::stoi(tj.at("Material").get<std::string>());

                std::stringstream ss(tj.at("Indices").get<std::string>());
                ss >> tri.face.i0 >> tri.face.i1 >> tri.face.i2;

                const Vec3f& tri_va = scene.vertex_data[tri.face.i0 - 1].pos;
                const Vec3f& tri_vb = scene.vertex_data[tri.face.i1 - 1].pos;
                const Vec3f& tri_vc = scene.vertex_data[tri.face.i2 - 1].pos;

                Vec3f normal = (tri_vc - tri_vb).crossProduct(tri_va - tri_vb);
                Vec3f n_unit = normal.normalize();

                // Degenerate triangle, skip pushing
                if (n_unit.x == 0.f && n_unit.y == 0.f && n_unit.z == 0.f) {
                    return;
                }

                tri.face.n_face  = n_unit;
                tri.face.plane_d = -n_unit.dotProduct(tri_va);

                scene.triangles.push_back(tri);
            };

            if (triNode.is_array()) {
                for (const auto& tj : triNode) parseOneTriangle(tj);
            } else {
                parseOneTriangle(triNode);
            }
        }

        // --- Spheres ---
        if (objects.contains("Sphere")) {
            const auto& sphNode = objects["Sphere"];

            auto parseOneSphere = [&](const json& sj) {
                Sphere sp{};
                sp.material_id       = std::stoi(sj.at("Material").get<std::string>());
                sp.center_vertex_id  = std::stoi(sj.at("Center").get<std::string>());
                sp.radius            = parser::parseFloat(sj.at("Radius").get<std::string>());

                scene.spheres.push_back(sp);
            };

            if (sphNode.is_array()) {
                for (const auto& sj : sphNode) parseOneSphere(sj);
            } else {
                parseOneSphere(sphNode);
            }
        }

        // --- Planes ---
        if (objects.contains("Plane")) {
            const auto& plNode = objects["Plane"];

            auto parseOnePlane = [&](const json& pj) {
                Plane p{};

                p.material_id = std::stoi(pj.at("Material").get<std::string>());
                p.vertex_id   = std::stoi(pj.at("Point").get<std::string>());
                p.n_face      = parser::parseVec3f(pj.at("Normal").get<std::string>()).normalize();

                // Degenerate plane
                if (p.n_face.x == 0.f && p.n_face.y == 0.f && p.n_face.z == 0.f) {
                    return;
                }

                const Vertex& plane_vertex = scene.vertex_data[p.vertex_id - 1];
                p.plane_d = -p.n_face.dotProduct(plane_vertex.pos);

                scene.planes.push_back(p);
            };

            if (plNode.is_array()) {
                for (const auto& pj : plNode) parseOnePlane(pj);
            } else {
                parseOnePlane(plNode);
            }
        }
    }

    return scene;
}

Vec3f parser::parseVec3f(const std::string& s)
{
    std::stringstream ss(s);
    float x, y, z;
    ss >> x >> y >> z;
    return { x, y, z };
}

float parser::parseFloat(const std::string& s)
{
    return std::strtof(s.c_str(), nullptr);
}

std::string parser::join_with_json_dir(const std::string& scene_path, const std::string& rel_or_abs)
{
    // absolute? just return
    if (!rel_or_abs.empty() && (rel_or_abs[0] == '/' || rel_or_abs[0] == '\\'
#ifdef _WIN32
        || (rel_or_abs.size() > 1 && rel_or_abs[1] == ':')
#endif
        )) return rel_or_abs;

    // dir of scene_path
    size_t slash = scene_path.find_last_of("/\\");
    if (slash == std::string::npos) return rel_or_abs;
    return scene_path.substr(0, slash + 1) + rel_or_abs;
}

// Property descriptor for PLY files
struct PlyProperty {
    std::string name;
    std::string type;  // float, double, uchar, int, etc.
    bool is_list = false;
    std::string count_type;  // for list properties
    std::string item_type;   // for list properties
    int byte_size = 0;
    
    PlyProperty(const std::string& n = "", const std::string& t = "") 
        : name(n), type(t) {
        // Set byte size based on type
        if (t == "char" || t == "uchar") byte_size = 1;
        else if (t == "short" || t == "ushort") byte_size = 2;
        else if (t == "int" || t == "uint" || t == "float") byte_size = 4;
        else if (t == "double") byte_size = 8;
    }
};

// read both vertices and faces (binary LE or ASCII), with proper property parsing
PlyData parser::load_ply(const std::string& path) {
    PlyData out;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::cerr << "Failed to open PLY file: " << path << std::endl;
        return out;
    }

    std::string line;
    bool in_header = true, is_ascii = false, is_bin_le = false;
    int64_t vcount = -1, fcount = -1;
    
    // Track which properties exist for vertices
    std::vector<PlyProperty> vertex_props;
    std::vector<PlyProperty> face_props;
    bool parsing_vertex_element = false;
    bool parsing_face_element = false;

    // ---- Parse header ----
    while (in_header && std::getline(in, line)) {
        line = trim(line);
        
        if (line.rfind("format ", 0) == 0) {
            is_ascii = line.find("ascii") != std::string::npos;
            is_bin_le = line.find("binary_little_endian") != std::string::npos;
        } 
        else if (line.rfind("element vertex", 0) == 0) {
            std::istringstream ls(line); 
            std::string a, b; 
            ls >> a >> b >> vcount;
            parsing_vertex_element = true;
            parsing_face_element = false;
        } 
        else if (line.rfind("element face", 0) == 0) {
            std::istringstream ls(line); 
            std::string a, b; 
            ls >> a >> b >> fcount;
            parsing_vertex_element = false;
            parsing_face_element = true;
        }
        else if (line.rfind("element ", 0) == 0) {
            // Some other element type - stop tracking properties
            parsing_vertex_element = false;
            parsing_face_element = false;
        }
        else if (line.rfind("property ", 0) == 0) {
            std::istringstream ls(line);
            std::string keyword, type_or_list;
            ls >> keyword >> type_or_list;
            
            if (type_or_list == "list") {
                // property list <count_type> <item_type> <name>
                PlyProperty prop;
                prop.is_list = true;
                ls >> prop.count_type >> prop.item_type >> prop.name;
                
                if (parsing_vertex_element) {
                    vertex_props.push_back(prop);
                } else if (parsing_face_element) {
                    face_props.push_back(prop);
                }
            } else {
                // property <type> <name>
                std::string name;
                ls >> name;
                PlyProperty prop(name, type_or_list);
                
                if (parsing_vertex_element) {
                    vertex_props.push_back(prop);
                } else if (parsing_face_element) {
                    face_props.push_back(prop);
                }
            }
        }
        else if (line == "end_header") {
            in_header = false;
        }
    }

    if (!is_ascii && !is_bin_le) {
        std::cerr << "Unsupported PLY format (must be ASCII or binary_little_endian)" << std::endl;
        return out;
    }

    // Find property indices for x, y, z, nx, ny, nz
    int x_idx = -1, y_idx = -1, z_idx = -1;
    int nx_idx = -1, ny_idx = -1, nz_idx = -1;
    
    for (size_t i = 0; i < vertex_props.size(); ++i) {
        const auto& prop = vertex_props[i];
        if (prop.name == "x") x_idx = i;
        else if (prop.name == "y") y_idx = i;
        else if (prop.name == "z") z_idx = i;
        else if (prop.name == "nx") nx_idx = i;
        else if (prop.name == "ny") ny_idx = i;
        else if (prop.name == "nz") nz_idx = i;
    }
    
    bool has_normals = (nx_idx >= 0 && ny_idx >= 0 && nz_idx >= 0);

    // ---- Read data ----
    if (is_ascii) {
        // ASCII format
        out.verts.reserve(vcount > 0 ? size_t(vcount) : 0);
        if (has_normals) {
            out.normals.reserve(vcount > 0 ? size_t(vcount) : 0);
        }
        
        for (int64_t i = 0; i < vcount; ++i) {
            std::getline(in, line);
            std::istringstream ls(line);
            
            std::vector<float> values;
            float val;
            while (ls >> val) {
                values.push_back(val);
            }
            
            if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0 && 
                x_idx < (int)values.size() && y_idx < (int)values.size() && z_idx < (int)values.size()) {
                out.verts.push_back({values[x_idx], values[y_idx], values[z_idx]});
                
                if (has_normals && nx_idx < (int)values.size() && ny_idx < (int)values.size() && nz_idx < (int)values.size()) {
                    out.normals.push_back({values[nx_idx], values[ny_idx], values[nz_idx]});
                }
            }
        }
        
        // Read faces (n i0 i1 i2 …)
        for (int64_t i = 0; i < fcount; ++i) {
            std::getline(in, line);
            if (line.empty()) continue;
            std::istringstream ls(line);
            int n; 
            if (!(ls >> n) || n < 3) continue;
            std::vector<int> idx(n);
            for (int k = 0; k < n; ++k) ls >> idx[k];
            // Triangulate polygon (fan triangulation)
            for (int k = 1; k + 1 < n; ++k)
                out.faces.push_back({ idx[0] + 1, idx[k] + 1, idx[k+1] + 1 });
        }
        
        return out;
    }

    // ---- Binary little-endian format ----
    out.verts.reserve(vcount > 0 ? size_t(vcount) : 0);
    if (has_normals) {
        out.normals.reserve(vcount > 0 ? size_t(vcount) : 0);
    }
    
    for (int64_t i = 0; i < vcount; ++i) {
        std::vector<float> values(vertex_props.size(), 0.0f);
        
        // Read each property
        for (size_t p = 0; p < vertex_props.size(); ++p) {
            const auto& prop = vertex_props[p];
            
            if (prop.type == "float") {
                float val;
                in.read(reinterpret_cast<char*>(&val), 4);
                values[p] = val;
            }
            else if (prop.type == "double") {
                double val;
                in.read(reinterpret_cast<char*>(&val), 8);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "uchar") {
                uint8_t val;
                in.read(reinterpret_cast<char*>(&val), 1);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "char") {
                int8_t val;
                in.read(reinterpret_cast<char*>(&val), 1);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "ushort") {
                uint16_t val;
                in.read(reinterpret_cast<char*>(&val), 2);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "short") {
                int16_t val;
                in.read(reinterpret_cast<char*>(&val), 2);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "uint") {
                uint32_t val;
                in.read(reinterpret_cast<char*>(&val), 4);
                values[p] = static_cast<float>(val);
            }
            else if (prop.type == "int") {
                int32_t val;
                in.read(reinterpret_cast<char*>(&val), 4);
                values[p] = static_cast<float>(val);
            }
            else {
                // Unknown type, skip
                if (prop.byte_size > 0) {
                    in.seekg(prop.byte_size, std::ios::cur);
                }
            }
        }
        
        // Extract position and normal
        if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0) {
            out.verts.push_back({values[x_idx], values[y_idx], values[z_idx]});
            
            if (has_normals) {
                out.normals.push_back({values[nx_idx], values[ny_idx], values[nz_idx]});
            }
        }
    }

    // Read faces: [uint8 n][int32 idx]*n  (most common format)
    out.faces.reserve(fcount > 0 ? size_t(fcount) * 2 : 0);
    for (int64_t f = 0; f < fcount; ++f) {
        uint8_t n = 0;
        if (!in.read(reinterpret_cast<char*>(&n), 1)) break;
        if (n < 3) { 
            in.seekg(int64_t(n) * 4, std::ios::cur); 
            continue; 
        }
        std::vector<int32_t> idx(n);
        if (!in.read(reinterpret_cast<char*>(idx.data()), int64_t(n) * 4)) break;
        // Triangulate polygon (fan triangulation)
        for (int k = 1; k + 1 < n; ++k)
            out.faces.push_back({ idx[0] + 1, idx[k] + 1, idx[k+1] + 1 });
    }
    
    return out;
}

std::string parser::flatten_faces_to_string(const std::vector<std::array<int,3>>& tris)
{
    std::string out;
    out.reserve(tris.size() * 12); // reserve some space to avoid reallocations

    for (const auto& t : tris) {
        out += std::to_string(t[0]) + " "
            + std::to_string(t[1]) + " "
            + std::to_string(t[2]) + " ";
    }
    if (!out.empty()) out.pop_back(); // remove last space
    return out;
}
