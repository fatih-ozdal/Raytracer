#include <fstream>
#include <sstream>
#include "parser.h"
#include <cctype>
#include <algorithm>

#include <iostream>
using std::cout;
using std::endl;

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

                // Note: assuming no need to check _type == triangle
                std::string facesStr = mj.at("Faces").at("_data").get<std::string>();
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
