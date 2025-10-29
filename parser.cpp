#include <fstream>
#include <sstream>
#include "parser.h"

#include <iostream>

using std::cout;
using std::endl;

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
        scene.max_recursion_depth = static_cast<int>(
            parser::parseFloat(s["MaxRecursionDepth"].get<std::string>())
        );
    } else {
        scene.max_recursion_depth = 0; // or whatever default you want
    }
    
    // --- AmbientLight ---
    json& lights = s["Lights"];

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

    // --- Cameras ---
    if (s.contains("Cameras") && s["Cameras"].contains("Camera")) {
        const auto& node = s["Cameras"]["Camera"];

        auto parseOneCamera = [&](const json& cj) {
            Camera cam;

            cam.position = parser::parseVec3f(cj["Position"].get<std::string>());
            cam.gaze = parser::parseVec3f(cj["Gaze"].get<std::string>());
            cam.up = parser::parseVec3f(cj["Up"].get<std::string>());

            cam.w = -1 * cam.gaze.normalize();
            Vec3f v_prime = cam.up.normalize();
            cam.u = v_prime.crossProduct(cam.w);
            cam.v = cam.w.crossProduct(cam.u);

            // NearPlane
            {
                std::stringstream ss(cj["NearPlane"].get<std::string>());
                float l=0, r=0, b=0, t=0;
                ss >> l >> r >> b >> t; // read first four only
                cam.near_plane = {l, r, b, t};
            }

            cam.near_distance = parser::parseFloat(cj["NearDistance"].get<std::string>());

            // ImageResolution
            {
                std::stringstream ss(cj["ImageResolution"].get<std::string>());
                ss >> cam.width >> cam.height;
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
