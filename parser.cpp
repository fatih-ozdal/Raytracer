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
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to trim whitespace from string
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

// ============== TRANSFORMATION HELPER FUNCTIONS ==============

Mat4f parser::MakeTranslation(const Vec3f& t) {
    Mat4f m = Mat4f::identity();
    m.m[0][3] = t.x;
    m.m[1][3] = t.y;
    m.m[2][3] = t.z;
    return m;
}

Mat4f parser::MakeScaling(const Vec3f& s) {
    Mat4f m = Mat4f::identity();
    m.m[0][0] = s.x;
    m.m[1][1] = s.y;
    m.m[2][2] = s.z;
    return m;
}

Mat4f parser::MakeRotation(float angleDegrees, const Vec3f& axis) {
    float rad = angleDegrees * M_PI / 180.0f;
    float c = cosf(rad);
    float s = sinf(rad);
    float t = 1.0f - c;
    
    Vec3f a = axis.normalize();
    
    Mat4f m = Mat4f::identity();
    m.m[0][0] = t * a.x * a.x + c;
    m.m[0][1] = t * a.x * a.y - s * a.z;
    m.m[0][2] = t * a.x * a.z + s * a.y;
    
    m.m[1][0] = t * a.x * a.y + s * a.z;
    m.m[1][1] = t * a.y * a.y + c;
    m.m[1][2] = t * a.y * a.z - s * a.x;
    
    m.m[2][0] = t * a.x * a.z - s * a.y;
    m.m[2][1] = t * a.y * a.z + s * a.x;
    m.m[2][2] = t * a.z * a.z + c;
    
    return m;
}

Mat4f parser::ParseTransformations(const string& transformStr, const Scene& scene) {
    if (transformStr.empty()) {
        return Mat4f::identity();
    }
    
    // Parse all transformations into a vector
    vector<Mat4f> transforms;
    std::istringstream iss(transformStr);
    string token;
    
    while (iss >> token) {
        char type = token[0];
        int id = stoi(token.substr(1)) - 1;  // Convert to 0-indexed
        
        Mat4f transform;
        
        if (type == 't') {
            transform = scene.translations[id];
        }
        else if (type == 's') {
            transform = scene.scalings[id];
        }
        else if (type == 'r') {
            transform = scene.rotations[id];
        }
        else if (type == 'c') {
            transform = scene.composites[id];
        }
        
        transforms.push_back(transform);
    }
    
    // Multiply in REVERSE order: "t1 r2 s3" → M = S3 * R2 * T1
    Mat4f result = Mat4f::identity();
    for (int i = 0; i < transforms.size(); i++) {
        result = transforms[i] * result;
    }
    
    return result;
}

AABB parser::TransformAABB(const AABB& box, const Mat4f& transform) {
    // Get all 8 corners of the box
    Vec3f corners[8] = {
        Vec3f(box.min.x, box.min.y, box.min.z),
        Vec3f(box.max.x, box.min.y, box.min.z),
        Vec3f(box.min.x, box.max.y, box.min.z),
        Vec3f(box.max.x, box.max.y, box.min.z),
        Vec3f(box.min.x, box.min.y, box.max.z),
        Vec3f(box.max.x, box.min.y, box.max.z),
        Vec3f(box.min.x, box.max.y, box.max.z),
        Vec3f(box.max.x, box.max.y, box.max.z)
    };
    
    // Transform all corners and recompute AABB
    AABB result;
    result.reset();
    for (int i = 0; i < 8; i++) {
        Vec3f transformed = transform.transformPoint(corners[i]);
        result.expand(transformed);
    }
    
    return result;
}

// ============== MAIN SCENE LOADER ==============

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
        scene.background_color = Vec3f(0, 0, 0);
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
        scene.max_recursion_depth = 6;
    }

    // === PARSE TRANSFORMATIONS FIRST ===
    if (s.contains("Transformations")) {
        const auto& transNode = s["Transformations"];
        
        // Parse Translations
        if (transNode.contains("Translation")) {
            const auto& translationNode = transNode["Translation"];

            int translationCount = translationNode.is_array() ? translationNode.size() : 1;
            scene.translations.reserve(translationCount);

            auto parseOne = [&](const json& t) {
                Vec3f delta = parser::parseVec3f(t.at("_data").get<string>());
                Mat4f translation_matrix = MakeTranslation(delta);
                scene.translations.push_back(translation_matrix);
            };
            
            if (translationNode.is_array()) {
                for (const auto& t : translationNode) parseOne(t);
            } else {
                parseOne(translationNode);
            }
        }
        
        // Parse Scalings
        if (transNode.contains("Scaling")) {
            const auto& scaleNode = transNode["Scaling"];

            int scaleCount = scaleNode.is_array() ? scaleNode.size() : 1;
            scene.scalings.reserve(scaleCount);

            auto parseOne = [&](const json& s_json) {
                Vec3f scale_vec = parser::parseVec3f(s_json.at("_data").get<string>());
                Mat4f scale_matrix = MakeScaling(scale_vec);
                scene.scalings.push_back(scale_matrix);
            };
            
            if (scaleNode.is_array()) {
                for (const auto& s_json : scaleNode) parseOne(s_json);
            } else {
                parseOne(scaleNode);
            }
        }
        
        // Parse Rotations
        if (transNode.contains("Rotation")) {
            const auto& rotNode = transNode["Rotation"];

            int rotCount = rotNode.is_array() ? rotNode.size() : 1;
            scene.rotations.reserve(rotCount);

            auto parseOne = [&](const json& r) {
                string data = r.at("_data").get<string>();
                std::istringstream iss(data);
                float angle, axis_x, axis_y, axis_z;
                iss >> angle >> axis_x >> axis_y >> axis_z;
                Mat4f rot_matrix = MakeRotation(angle, Vec3f(axis_x, axis_y, axis_z));
                scene.rotations.push_back(rot_matrix);
            };
            
            if (rotNode.is_array()) {
                for (const auto& r : rotNode) parseOne(r);
            } else {
                parseOne(rotNode);
            }
        }
        
        // Parse Composites
        if (transNode.contains("Composite")) {
            const auto& compNode = transNode["Composite"];

            int compCount = compNode.is_array() ? compNode.size() : 1;
            scene.composites.reserve(compCount);

            auto parseOne = [&](const json& c) {
                Mat4f comp_matrix;
                string data = c.at("_data").get<string>();
                std::istringstream iss(data);
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        iss >> comp_matrix.m[i][j];
                    }
                }
                scene.composites.push_back(comp_matrix);
            };
            
            if (compNode.is_array()) {
                for (const auto& c : compNode) parseOne(c);
            } else {
                parseOne(compNode);
            }
        }
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
            scene.ambient_light = Vec3f(0.f, 0.f, 0.f);
        }

        // --- PointLights ---
        if (lights.contains("PointLight")) {
            const auto& pLightNode = lights["PointLight"];

            int pLightCount = pLightNode.is_array() ? pLightNode.size() : 1;
            scene.point_lights.reserve(pLightCount);

            auto parseOnePointLight = [&](const json& pl) {
                PointLight L;
                L.position  = parser::parseVec3f(pl["Position"].get<std::string>());
                L.intensity = parser::parseVec3f(pl["Intensity"].get<std::string>());
                
                // Apply transformations if present
                if (pl.contains("Transformations")) {
                    Mat4f transform = ParseTransformations(pl.at("Transformations").get<string>(), scene);
                    L.position = transform.transformPoint(L.position);
                }
                
                scene.point_lights.push_back(L);
            };

            if (pLightNode.is_array()) {
                for (const auto& pl : pLightNode) parseOnePointLight(pl);
            } else {
                parseOnePointLight(pLightNode);
            }
        }
    }

    // --- Cameras ---
    if (s.contains("Cameras") && s["Cameras"].contains("Camera")) {
        const auto& cameraNode = s["Cameras"]["Camera"];

        int cameraCount = cameraNode.is_array() ? cameraNode.size() : 1;
        scene.cameras.reserve(cameraCount);

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

            cam.w = cam.gaze * -1;
            cam.w = cam.w.normalize();
            Vec3f v_prime = cam.up.normalize();
            cam.u = v_prime.crossProduct(cam.w).normalize();
            cam.v = cam.w.crossProduct(cam.u);  // already normalized
            
            // Apply transformations if present (BEFORE computing near plane)
            if (cj.contains("Transformations")) {
                Mat4f camTransform = ParseTransformations(cj.at("Transformations").get<string>(), scene);
                
                cam.position = camTransform.transformPoint(cam.position);
                cam.gaze = camTransform.transformVector(cam.gaze).normalize();
                cam.up = camTransform.transformVector(cam.up).normalize();
                
                // Recompute u, v, w
                cam.w = cam.gaze * -1;
                cam.w = cam.w.normalize();
                Vec3f v_prime2 = cam.up.normalize();
                cam.u = v_prime2.crossProduct(cam.w).normalize();
                cam.v = cam.w.crossProduct(cam.u);
            }
            
            // ImageResolution
            std::stringstream ss(cj["ImageResolution"].get<std::string>());
            ss >> cam.image_width >> cam.image_height;
            
            // NearDistance
            cam.near_distance = parser::parseFloat(cj["NearDistance"].get<std::string>());

            // NearPlane
            if (is_lookAt_type) {
                float fovY_deg = parser::parseFloat(cj["FovY"].get<std::string>());
                float thetaY = deg2rad(fovY_deg);
                float t = cam.near_distance * std::tan(thetaY * 0.5f);
                float b = -t;

                float aspect = (cam.image_height > 0) ? (float(cam.image_width) / float(cam.image_height)) : 1.0f;
                float r = aspect * t;
                float l = -r;

                cam.near_plane = { l, r, b, t };
            } else {
                std::stringstream ss2(cj["NearPlane"].get<std::string>());
                float l=0, r=0, b=0, t=0;
                ss2 >> l >> r >> b >> t;
                cam.near_plane = {l, r, b, t};
            }

            // Pixel Width - Heigth
            cam.pixel_width  = (cam.near_plane.r - cam.near_plane.l) / cam.image_width;
            cam.pixel_height = (cam.near_plane.t - cam.near_plane.b) / cam.image_height;

            // Near Plane's origin
            cam.m = cam.position - cam.w * cam.near_distance;
            cam.q = cam.m + cam.u * cam.near_plane.l + cam.v * cam.near_plane.t;

            cam.image_name = cj["ImageName"].get<std::string>();
            scene.cameras.push_back(cam);
        };

        if (cameraNode.is_array()) {
            for (const auto& cj : cameraNode) parseOneCamera(cj);
        } else {
            parseOneCamera(cameraNode);
        }
    }

    // --- Materials ---
    if (s.contains("Materials") && s["Materials"].contains("Material")) {
        const auto& materialNode = s["Materials"]["Material"];

        int materialCount = materialNode.is_array() ? materialNode.size() : 1;
        scene.materials.reserve(materialCount);

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

        if (materialNode.is_array()) {
            for (const auto& mj : materialNode) parseOneMaterial(mj);
        } else {
            parseOneMaterial(materialNode);
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
            data = vd.value("_data", "");
        }

        std::stringstream ss(data);
        float x, y, z;

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
            v.pos = Vec3f(x, y, z);
            v.normal = Vec3f(0.f, 0.f, 0.f);
            scene.vertex_data.push_back(v);
        }
    }

    // --- Objects ---
    if (s.contains("Objects"))
    {
        const auto& objects = s["Objects"];

        // === MESHES ===
        if (objects.contains("Mesh")) {
            const auto& meshNode = objects["Mesh"];

            int meshCount = meshNode.is_array() ? meshNode.size() : 1;
            if (objects.contains("MeshInstance")) {
                const auto& instanceNode = objects["MeshInstance"];
                int meshInstanceCount = instanceNode.is_array() ? instanceNode.size() : 1;
                meshCount += meshInstanceCount;
            }
            scene.meshes.reserve(meshCount);

            auto parseOneMesh = [&](const json& mj) {
                Mesh mesh;
                mesh.is_smooth = mj.value("_shadingMode", "flat") == "smooth";
                mesh.material_id = std::stoi(mj.at("Material").get<std::string>());

                int jsonMeshId = std::stoi(mj.at("_id").get<std::string>());
                int meshIndex = scene.meshes.size();
                scene.meshIdToIndex[jsonMeshId] = meshIndex;

                std::string facesStr;
                bool ply_has_normals = false;

                // Get all of the Vertex ids for Faces
                if (mj.contains("Faces") && mj["Faces"].contains("_plyFile")) {
                    std::string ply_rel = mj["Faces"]["_plyFile"].get<std::string>();
                    std::string ply_path = join_with_json_dir(filepath, ply_rel);
                    auto ply = load_ply(ply_path);

                    size_t base = scene.vertex_data.size();
                    scene.vertex_data.reserve(base + ply.verts.size());
                    
                    ply_has_normals = !ply.normals.empty();
                    
                    for (size_t i = 0; i < ply.verts.size(); ++i) {
                        Vertex v;
                        v.pos = ply.verts[i];
                        v.normal = ply_has_normals ? ply.normals[i] : Vec3f(0, 0, 0);
                        scene.vertex_data.push_back(v);
                    }

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

                std::vector<int> touched;
                touched.reserve(256);

                auto mark_touched = [&](int idx) {
                    if (touched.empty() || touched.back() != idx) touched.push_back(idx);
                };

                mesh.localBounds.reset();

                int i0, i1, i2;
                while (ss >> i0 >> i1 >> i2) {
                    Face f{};
                    f.i0 = i0; f.i1 = i1; f.i2 = i2;

                    const Vec3f& va = scene.vertex_data[i0 - 1].pos;
                    const Vec3f& vb = scene.vertex_data[i1 - 1].pos;
                    const Vec3f& vc = scene.vertex_data[i2 - 1].pos;

                    mesh.localBounds.expand(va);
                    mesh.localBounds.expand(vb);
                    mesh.localBounds.expand(vc);

                    Vec3f faceNorm = (vb - va).crossProduct(vc - va);
                    f.n_unit = faceNorm.normalize();
                    f.plane_d = -(f.n_unit.dotProduct(va));

                    // Accumulate normal for smooth shading (if PLY didn't provide normals)
                    if (mesh.is_smooth && !ply_has_normals) {
                        scene.vertex_data[i0 - 1].normal = scene.vertex_data[i0 - 1].normal + faceNorm;
                        scene.vertex_data[i1 - 1].normal = scene.vertex_data[i1 - 1].normal + faceNorm;
                        scene.vertex_data[i2 - 1].normal = scene.vertex_data[i2 - 1].normal + faceNorm;

                        mark_touched(i0);
                        mark_touched(i1);
                        mark_touched(i2);
                    }

                    mesh.faces.push_back(f);
                }

                // Normalize accumulated normals (if we computed them)
                if (mesh.is_smooth && !ply_has_normals) {
                    for (int idx : touched) {
                        scene.vertex_data[idx - 1].normal = scene.vertex_data[idx - 1].normal.normalize();
                    }
                }

                // === PARSE TRANSFORMATIONS ===
                if (mj.contains("Transformations")) {
                    mesh.transformation = ParseTransformations(mj.at("Transformations").get<string>(), scene);
                    mesh.invTransformation = mesh.transformation.inverse();
                    mesh.hasTransform = true;
                    mesh.worldBounds = TransformAABB(mesh.localBounds, mesh.transformation);
                } else {
                    mesh.transformation = Mat4f::identity();
                    mesh.invTransformation = Mat4f::identity();
                    mesh.hasTransform = false;
                    mesh.worldBounds = mesh.localBounds;
                }

                scene.meshes.push_back(mesh);
            };

            if (meshNode.is_array()) {
                for (const auto& mj : meshNode) parseOneMesh(mj);
            } else {
                parseOneMesh(meshNode);
            }
        }

        // === MESH INSTANCES ===
        if (objects.contains("MeshInstance")) {
            const auto& instanceNode = objects["MeshInstance"];
            
            auto parseOneInstance = [&](const json& inst) {
                int baseMeshJsonId = stoi(inst.at("_baseMeshId").get<string>());
                int baseMeshId = scene.meshIdToIndex.at(baseMeshJsonId); 

                int jsonMeshId = std::stoi(inst.at("_id").get<std::string>());
                int meshIndex = scene.meshes.size();
                scene.meshIdToIndex[jsonMeshId] = meshIndex;
                
                bool resetTransform = false;
                if (inst.contains("_resetTransform")) {
                    resetTransform = (inst.at("_resetTransform").get<string>() == "true");
                }
                
                // Find the ORIGINAL mesh by following the chain
                int currentId = baseMeshId;
                while (currentId < (int)scene.meshes.size() && scene.meshes[currentId].isInstance) {
                    currentId = scene.meshes[currentId].originalMeshId;
                }
                int originalMeshId = currentId;
                
                const Mesh& originalMesh = scene.meshes[originalMeshId];
                const Mesh& baseMesh = scene.meshes[baseMeshId];

                int materialId;
                if (inst.contains("Material")) {
                    materialId = stoi(inst.at("Material").get<string>());
                } else {
                    materialId = baseMesh.material_id;
                }
                            
                // Compute transformation
                Mat4f instanceTransform = Mat4f::identity();
                if (inst.contains("Transformations")) {
                    instanceTransform = ParseTransformations(inst.at("Transformations").get<string>(), scene);
                }
                
                Mat4f finalTransform;
                if (resetTransform) {
                    finalTransform = instanceTransform;
                } else {
                    finalTransform = instanceTransform * baseMesh.transformation;
                }
                
                // Create new mesh as an instance
                Mesh newMesh;
                newMesh.faces = originalMesh.faces;  // Share geometry
                newMesh.is_smooth = originalMesh.is_smooth;
                newMesh.material_id = materialId;
                newMesh.transformation = finalTransform;
                newMesh.invTransformation = finalTransform.inverse();
                newMesh.hasTransform = true;
                newMesh.isInstance = true;
                newMesh.originalMeshId = originalMeshId;
                newMesh.bvhIndex = -1;
                newMesh.localBounds = originalMesh.localBounds;
                newMesh.worldBounds = TransformAABB(originalMesh.localBounds, finalTransform);
                
                scene.meshes.push_back(newMesh);
            };
            
            if (instanceNode.is_array()) {
                for (const auto& inst : instanceNode) {
                    parseOneInstance(inst);
                }
            } else {
                parseOneInstance(instanceNode);
            }
        }

        // === TRIANGLES ===
        if (objects.contains("Triangle")) {
            const auto& triNode = objects["Triangle"];

            int triCount = triNode.is_array() ? triNode.size() : 1;
            scene.triangles.reserve(triCount);

            auto parseOneTriangle = [&](const json& tj) {
                Triangle tri;
                tri.material_id = std::stoi(tj.at("Material").get<std::string>());

                std::string data = tj.at("Indices").get<std::string>();
                std::stringstream ss(data);
                int i0, i1, i2;
                ss >> i0 >> i1 >> i2;

                Face f{};
                f.i0 = i0; f.i1 = i1; f.i2 = i2;

                const Vec3f& va = scene.vertex_data[i0 - 1].pos;
                const Vec3f& vb = scene.vertex_data[i1 - 1].pos;
                const Vec3f& vc = scene.vertex_data[i2 - 1].pos;

                tri.localBounds.reset();
                tri.localBounds.expand(va);
                tri.localBounds.expand(vb);
                tri.localBounds.expand(vc);

                Vec3f faceNorm = (vb - va).crossProduct(vc - va);
                f.n_unit = faceNorm.normalize();
                f.plane_d = -(f.n_unit.dotProduct(va));
                tri.face = f;

                // === PARSE TRANSFORMATIONS ===
                if (tj.contains("Transformations")) {
                    tri.transformation = ParseTransformations(tj.at("Transformations").get<string>(), scene);
                    tri.invTransformation = tri.transformation.inverse();
                    tri.hasTransform = true;
                    tri.worldBounds = TransformAABB(tri.localBounds, tri.transformation);
                } else {
                    tri.transformation = Mat4f::identity();
                    tri.invTransformation = Mat4f::identity();
                    tri.hasTransform = false;
                    tri.worldBounds = tri.localBounds;
                }

                scene.triangles.push_back(tri);
            };

            if (triNode.is_array()) {
                for (const auto& tj : triNode) parseOneTriangle(tj);
            } else {
                parseOneTriangle(triNode);
            }
        }

        // === SPHERES ===
        if (objects.contains("Sphere")) {
            const auto& sphNode = objects["Sphere"];

            int sphCount = sphNode.is_array() ? sphNode.size() : 1;
            scene.spheres.reserve(sphCount);

            auto parseOneSphere = [&](const json& sj) {
                Sphere sp;
                sp.material_id = std::stoi(sj.at("Material").get<std::string>());
                sp.center_vertex_id = std::stoi(sj.at("Center").get<std::string>());
                sp.radius = parseFloat(sj.at("Radius").get<std::string>());

                Vec3f center = scene.vertex_data[sp.center_vertex_id - 1].pos;
                Vec3f rVec = Vec3f(sp.radius, sp.radius, sp.radius);
                sp.localBounds.min = center - rVec;
                sp.localBounds.max = center + rVec;

                // === PARSE TRANSFORMATIONS ===
                if (sj.contains("Transformations")) {
                    sp.transformation = ParseTransformations(sj.at("Transformations").get<string>(), scene);
                    sp.invTransformation = sp.transformation.inverse();
                    sp.hasTransform = true;
                    sp.worldBounds = TransformAABB(sp.localBounds, sp.transformation);
                } else {
                    sp.transformation = Mat4f::identity();
                    sp.invTransformation = Mat4f::identity();
                    sp.hasTransform = false;
                    sp.worldBounds = sp.localBounds;
                }

                scene.spheres.push_back(sp);
            };

            if (sphNode.is_array()) {
                for (const auto& sj : sphNode) parseOneSphere(sj);
            } else {
                parseOneSphere(sphNode);
            }
        }

        // === PLANES ===
        if (objects.contains("Plane")) {
            const auto& planeNode = objects["Plane"];

            int planeCount = planeNode.is_array() ? planeNode.size() : 1;
            scene.planes.reserve(planeCount);

            auto parseOnePlane = [&](const json& pj) {
                Plane plane;
                plane.material_id = std::stoi(pj.at("Material").get<std::string>());
                plane.vertex_id = std::stoi(pj.at("Point").get<std::string>());

                Vec3f normal = parseVec3f(pj.at("Normal").get<std::string>());
                plane.n_unit = normal.normalize();

                Vec3f point = scene.vertex_data[plane.vertex_id - 1].pos;
                plane.plane_d = -(plane.n_unit.dotProduct(point));

                // === PARSE TRANSFORMATIONS ===
                if (pj.contains("Transformations")) {
                    plane.transformation = ParseTransformations(pj.at("Transformations").get<string>(), scene);
                    plane.invTransformation = plane.transformation.inverse();
                    plane.hasTransform = true;
                    
                    // Transform plane equation
                    // Normal transforms with inverse transpose
                    plane.n_unit = plane.invTransformation.transpose().transformVector(plane.n_unit).normalize();
                    // Point on plane transforms normally
                    point = plane.transformation.transformPoint(point);
                    plane.plane_d = -(plane.n_unit.dotProduct(point));
                } else {
                    plane.transformation = Mat4f::identity();
                    plane.invTransformation = Mat4f::identity();
                    plane.hasTransform = false;
                }

                scene.planes.push_back(plane);
            };

            if (planeNode.is_array()) {
                for (const auto& pj : planeNode) parseOnePlane(pj);
            } else {
                parseOnePlane(planeNode);
            }
        }
    }

    return scene;
}

// ============== HELPER FUNCTIONS ==============

Vec3f parser::parseVec3f(const std::string& s) {
    std::stringstream ss(s);
    float x, y, z;
    ss >> x >> y >> z;
    return Vec3f(x, y, z);
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
