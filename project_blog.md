# ray_tracer::devlog_volume - Volumetric Rendering with NanoVDB

After implementing path tracing, NEE, and MIS, we (Murat and Fatih) tackled one of the most visually striking features in rendering: **volumetric effects**. This meant rendering participating media—smoke, fog, clouds—using real-world volumetric data stored in OpenVDB format. We integrated **NanoVDB** for efficient CPU-based volume data access and implemented full physically-based volume rendering with scattering, absorption, and shadowing.

---

## Part 1: Data & Parsing - Integrating NanoVDB

### The Volume Struct: Treating VDB Grids as First-Class Scene Objects

We started by defining a `Volume` struct that would sit alongside meshes, spheres, and other primitives in our scene. The key challenge was integrating NanoVDB's data structures into our existing architecture.

Here's the Volume struct we defined in `parser.h`:

```cpp
struct Volume
{
    // NanoVDB components
    // Handle owns the memory buffer
    nanovdb::GridHandle<nanovdb::HostBuffer> handle;
   
    const nanovdb::FloatGrid* grid;
    
    nanovdb::ReadAccessor<float> accessor;

    // Spatial bounds
    // Used for BVH construction and ray intersection
    AABB bbox; 

    // Rendering parameters
    float density_scale; // Multiplier to control how "thick" the smoke is
    float step_size;     // Ray marching step size (lower = higher quality, slower)

    // Transformations
    // VDBs have an internal index->world transform. 
    // This matrix is for EXTRA scene placement (e.g. moving the VDB in the scene).
    Mat4f transformation;     
    Mat4f invTransformation;

    // Default constructor
    Volume() : 
        material_id(-1), 
        grid(nullptr), 
        accessor(nanovdb::ReadAccessor<float>(nullptr)), 
        density_scale(1.0f), 
        step_size(0.1f) 
    {
        transformation = Mat4f::identity();
        invTransformation = Mat4f::identity();
    }
};
```

## Struct Fields

1. **`GridHandle<nanovdb::HostBuffer> handle`**: This owns the memory buffer. NanoVDB requires the handle to stay alive for the grid pointer to remain valid.

2. **`const nanovdb::FloatGrid* grid`**: A pointer to the actual grid data. We use `FloatGrid` because our densities are stored as floating-point values.

3. **`ReadAccessor<float> accessor`**: NanoVDB's optimized data access structure. Though we ended up using `grid->getAccessor()` directly in our sampling code.

4. **`AABB bbox`**: We store world-space bounds for ray intersection testing. Without this, we'd have to march through empty space.

5. **Physical Parameters**: The struct also includes `sigma_a` (absorption), `sigma_s` (scattering), `g` (phase function asymmetry), and `scale` (density multiplier), though these weren't visible in the header snippet we examined—they're parsed from JSON.

### Adding Volumes to the Scene

We extended the `Scene` struct to hold a vector of volumes:

```cpp
struct Scene
{
    // ... existing fields ...
    vector<Volume> volumes;  // Added for volumetric rendering
};
```

### Robust JSON Parsing: Handling OpenVDB Paths and Nested Volume Definitions

The parsing logic in `parser.cpp` (lines 535-604) needed to handle various JSON structures and robustly load NanoVDB files:

```cpp
if (s.contains("Volumes")) {
    json volNode = s["Volumes"];

    // FIX: Handle the nested structure "Volumes": { "Volume": { ... } }
    // If "Volumes" is an object containing a key "Volume", drill down into it.
    if (volNode.is_object() && volNode.contains("Volume")) {
        volNode = volNode["Volume"];
    }

    // Reserve space if it's an array
    int volCount = volNode.is_array() ? volNode.size() : 1;
    scene.volumes.reserve(volCount);

    auto parseOneVolume = [&](const json& vj) {
        Volume vol;

        // 1. Load parameters (Physics)
        vol.sigma_a = parser::parseVec3f(vj.value("Absorption", "0.0 0.0 0.0"));
        vol.sigma_s = parser::parseVec3f(vj.value("Scattering", "1.0 1.0 1.0"));
        vol.g       = parser::parseFloat(vj.value("Asymmetry", "0.0"));
        vol.scale   = parser::parseFloat(vj.value("Scale", "2.0"));

        // 2. Load NanoVDB Grid Path
        // Robust check for different key names (path, Path, file, File)
        string nvdb_rel;
        if (vj.contains("path")) nvdb_rel = vj["path"];
        else if (vj.contains("Path")) nvdb_rel = vj["Path"];
        else if (vj.contains("File")) nvdb_rel = vj["File"];
        else if (vj.contains("file")) nvdb_rel = vj["file"];
        else {
            std::cerr << "Error: Volume definition missing 'path' or 'File' key." << std::endl;
            return;
        }

        string nvdb_path =  nvdb_rel;
        
        try {
            // Read the first grid from the file
            auto handle = nanovdb::io::readGrid<nanovdb::HostBuffer>(nvdb_path);
            vol.handle = std::move(handle);
            vol.grid = vol.handle.grid<float>();

            if (!vol.grid) {
                std::cerr << "Error: Loaded VDB is not a FloatGrid: " << nvdb_path << std::endl;
                return;
            }
            
            // 3. Extract World Space AABB
            auto bbox = vol.grid->worldBBox();
            vol.worldBounds.min = Vec3f(bbox.min()[0], bbox.min()[1], bbox.min()[2]);
            vol.worldBounds.max = Vec3f(bbox.max()[0], bbox.max()[1], bbox.max()[2]);

        } catch (const std::exception& e) {
            std::cerr << "Failed to load volume: " << nvdb_path << "\nReason: " << e.what() << std::endl;
            return;
        }

        scene.volumes.push_back(std::move(vol));
    };

    // Handle Array vs Single Object
    if (volNode.is_array()) {
        for (const auto& vj : volNode) parseOneVolume(vj);
    } else {
        parseOneVolume(volNode);
    }
}
```


---

## Part 2: Core Ray Tracing - World-to-Index Mapping and AABB Optimization

### Ray Marching Implementation: Fixed Step-Size Integration

The heart of volumetric rendering is ray marching—stepping along a ray through the volume and accumulating scattering and absorption. Here's our implementation in `raytracer.cpp` :

```cpp
Vec3f integrate_volume(const Ray& ray, const Scene& scene, float t_entry, float t_exit, const Vec3f& background_color)
{
    if (t_entry >= t_exit) return background_color;

    Vec3f L(0, 0, 0); 
    Vec3f T(1, 1, 1); 
    float step_size = 0.3f; 
    
    for (float t = t_entry; t < t_exit; t += step_size) {
        
        float dt = std::min(step_size, t_exit - t);
        Vec3f p = ray.origin + ray.direction * (t + 0.5f * dt);

        for (const auto& vol : scene.volumes) {
            float density = SampleDensity(vol, p);

            if (density > 0.0f) {
                Vec3f sigma_a = vol.sigma_a * density;
                Vec3f sigma_s = vol.sigma_s * density;
                Vec3f sigma_t = sigma_a + sigma_s; 

                // Transmittance over step dt
                Vec3f step_transmittance;
                step_transmittance.x = std::exp(-sigma_t.x * dt);
                step_transmittance.y = std::exp(-sigma_t.y * dt);
                step_transmittance.z = std::exp(-sigma_t.z * dt);

                // In-Scattering
                Vec3f in_scattered_light(0, 0, 0);

                for (const auto& light : scene.point_lights) {
                    Vec3f dir_to_light = light.position - p;
                    float dist_light = dir_to_light.length();
                    dir_to_light = dir_to_light.normalize();

                    float phase = phase_function(ray.direction, dir_to_light, vol.g);
                    
                    // Light visibility (Shadows + Volume Attenuation)
                    Vec3f Li = SampleLightRadiance(light, p, scene);

                    // Li is already attenuated by distance and transmittance
                    in_scattered_light = in_scattered_light + 
                                         Li.elwiseMult(sigma_s) * phase;
                }

                L = L + T.elwiseMult(in_scattered_light) * dt;
                T = T.elwiseMult(step_transmittance);
            }
        }
        
        if (T.x < 0.001f && T.y < 0.001f && T.z < 0.001f) break;
    }

    return L + T.elwiseMult(background_color);
}
```

**Step-by-Step Breakdown:**

1. **Sample point calculation**: `Vec3f p = ray.origin + ray.direction * (t + 0.5f * dt)` samples at the **midpoint** of each step, reducing discretization artifacts.

2. **Extinction coefficient**: `Vec3f sigma_t = sigma_a + sigma_s` combines absorption and out-scattering. This is the total attenuation.

3. **Beer's Law**: `step_transmittance.x = std::exp(-sigma_t.x * dt)` implements exponential attenuation per channel. We keep transmittance as `Vec3f` to support colored fog.

4. **In-scattering accumulation**: For each light, we compute how much light scatters toward the camera at this point. The phase function weights directional scattering.

5. **Radiance accumulation**: `L = L + T.elwiseMult(in_scattered_light) * dt` adds the in-scattered light, weighted by the current transmittance.

6. **Early termination**: If transmittance drops below 0.001 in all channels, the ray is fully occluded—no need to continue.

### World-to-Index Space: Mapping Ray Coordinates to Voxel Data

NanoVDB stores data in **index space** (integer voxel coordinates), but our rays are in **world space**. The transformation happens in `SampleDensity` (lines 419-434):

```cpp
float SampleDensity(const Volume& vol, const Vec3f& p)
{
    if (!vol.grid) return 0.0f;

    nanovdb::Vec3f objP(p.x, p.y, p.z);
    nanovdb::Vec3f indexP = vol.grid->worldToIndexF(objP);
    auto accessor = vol.grid->getAccessor();

    // Use nearest neighbor for simplicity/speed
    nanovdb::Coord ijk(std::floor(indexP[0] + 0.5f), 
                       std::floor(indexP[1] + 0.5f), 
                       std::floor(indexP[2] + 0.5f));
                       
    float density = accessor.getValue(ijk);
    return density * vol.scale;
}
```

##Details

1. **`worldToIndexF`**: This is NanoVDB's built-in transformation from world coordinates to continuous index space. It handles the grid's internal transformation matrix.

2. **Nearest-neighbor sampling**: We round to the nearest voxel with `std::floor(indexP[0] + 0.5f)`. Trilinear interpolation would be more accurate but significantly slower.

3. **`accessor.getValue(ijk)`**: NanoVDB's optimized voxel lookup. The accessor maintains a cache of recently accessed tree nodes.

4. **Density scaling**: Multiplying by `vol.scale` lets artists adjust volume "thickness" without re-exporting the VDB file.

### AABB Optimization: Clipping Ray Intervals to Volume Bounds

W We clip the ray interval to the volume's AABB before integrating :


---

## Part 3: Lighting & Physics - Phase Functions and Volumetric Shadows

### Henyey-Greenstein Phase Function: Anisotropic Scattering

Not all scattering is equal. Forward-scattering (like fog in headlights) and back-scattering (like clouds in sunlight) have different visual characteristics. We use the **Henyey-Greenstein phase function** (lines 407-417):

```cpp
float phase_function(const Vec3f& dir_in, const Vec3f& dir_out, float g) noexcept
{
    Vec3f wi = dir_in.normalize();
    Vec3f wo = dir_out.normalize();

    float cos_theta = wi.dotProduct(wo);
    cos_theta = clampF(cos_theta, -1.0f, 1.0f);

    float denominator = 1.0f + g * g - 2.0f * g * cos_theta;
    return (1.0f - g * g) / (4.0f * PI * denominator * std::sqrt(denominator));
}
```

**Parameter `g`:**

- `g = 0`: Isotropic scattering (equal in all directions)
- `g > 0`: Forward scattering (light continues in roughly the same direction)
- `g < 0`: Back scattering (light bounces back toward the source)

The denominator `1.0f + g * g - 2.0f * g * cos_theta` creates the characteristic angular distribution. We clamp `cos_theta` to prevent numerical issues when directions are nearly parallel.

### Volumetric Shadows: Calculating Transmittance Towards Light Sources

Volumes don't just scatter light—they also block it. We need to calculate **transmittance** (what fraction of light reaches a point) for proper shadowing. Here's our implementation (lines 437-474):

```cpp
Vec3f GetVolumeTransmittance(const Vec3f& p, const Vec3f& lightPos, const Scene& scene)
{
    Vec3f dir = lightPos - p;
    float dist = dir.length();
    dir = dir.normalize();

    Vec3f transmittance(1.0f, 1.0f, 1.0f); // Start white
    float step_size = 0.4f; 

    // TRICK: Multiplier to fake multiple scattering (0.5 lets light penetrate 2x deeper)
    float shadow_trick = 0.3f; 

    for (float t = 0; t < dist; t += step_size) {
        Vec3f current_p = p + dir * t;
        
        for (const auto& vol : scene.volumes) {
            float density = SampleDensity(vol, current_p);
            
            if (density > 0.0f) {
                // No averaging! Keep it Vec3f
                Vec3f sigma_t = (vol.sigma_a + vol.sigma_s) * density;
                
                // Apply the trick to sigma_t
                sigma_t = sigma_t * shadow_trick;

                transmittance.x *= std::exp(-sigma_t.x * step_size);
                transmittance.y *= std::exp(-sigma_t.y * step_size);
                transmittance.z *= std::exp(-sigma_t.z * step_size);
            }
        }

        // Optimization: If all channels are blocked
        if (transmittance.x < 0.01f && transmittance.y < 0.01f && transmittance.z < 0.01f) 
            return Vec3f(0,0,0);
    }

    return transmittance;
}
```

### Beer's Law Application: Modeling Exponential Extinction

The core physics is in this line:

```cpp
transmittance.x *= std::exp(-sigma_t.x * step_size);
```

This is **Beer's Law** (also called Beer-Lambert Law): light intensity decays exponentially with optical depth. For a step of length `dt` through a medium with extinction coefficient `sigma_t`, the transmittance is `exp(-sigma_t * dt)`.

We apply this **per-channel** to support colored volumes (e.g., red wine absorbs green/blue light, appearing red).

---

## Part 4: Refinement

### The "Shadow Density Multiplier" Trick: Faking Multiple Scattering

Real volumetric scattering involves light bouncing multiple times inside the volume. But computing this properly requires path tracing through the volume itself—exponentially expensive. Instead, we use an **artist-friendly hack** (line 447):

```cpp
float shadow_trick = 0.3f;
```

By reducing `sigma_t` for shadow rays, we let more light penetrate the volume. This **approximates** the effect of multiple scattering: even deep inside a cloud, some light reaches you via indirect paths.

## Results

### Density Scale Comparison (Bunny Cloud)

The `scale` parameter controls how "thick" the volume appears. Higher values mean denser smoke with more light absorption.

| Render | Scale | Render Time |
|--------|-------|-------------|
| <img width="400" alt="bunny_scale_02" src="https://github.com/user-attachments/assets/2fb6925b-54b9-4fda-9b62-3bf47f711150" /> | `0.2` | 16.738s |
| <img width="400" alt="bunny_scale_04" src="https://github.com/user-attachments/assets/0f4816fe-b660-4cb2-a193-3b9abb9b4b74" /> | `0.4` | 18.313s |

---

### Step Size Comparison (Light Inside Volume)

Step size controls quality vs. performance tradeoff. Smaller steps = higher quality but slower renders.


| Render | Step Size | Render Time |
|--------|-------------|-------------|
| <img src="project_results/bunny_volume_test_st5.png" width="400" height="400"> | 5.0 | 3.599 s |
| <img src="project_results/bunny_volume_test_st1.png" width="400" height="400"> | 1.0 | 9.765 s |

---

### Scattering & Absorption Coefficient Effects

Demonstrating different volumetric materials with absorption/scattering coefficients.

| Render | Description | Render Time |
|--------|-------------|-------------|
| <img width="400" alt="fire_bend" src="https://github.com/user-attachments/assets/d91e721e-e657-4f21-b3da-75024c5a8bac" /> | Fire VDB with red/orange emission | 53.341s |
| <img width="400" alt="red_green_smoke" src="https://github.com/user-attachments/assets/83d6d575-e915-4c4c-83c6-a39ee29cab2e" /> | Dual-color smoke (red + green scattering) | 55.977s |
| <img width="400" alt="bunny_gray" src="https://github.com/user-attachments/assets/2399c890-389b-4789-bb89-869ca79d1668" /> | Neutral gray bunny cloud | 17.836s |
| <img width="400" alt="green_radiance" src="https://github.com/user-attachments/assets/7bfa9470-800a-4e06-8e50-84ea555e2dd5" /> | Green scattering | 16.831s |

---

## Conclusion

Implementing volumetric rendering required integrating external data formats (NanoVDB), understanding participating media physics (Beer's Law, phase functions), and optimizing ray marching for performance (AABB culling, early termination). The result is a renderer capable of producing realistic fog, smoke, and clouds from industry-standard VDB files.

The code is available at [fatih-ozdal/Raytracer](https://github.com/fatih-ozdal/Raytracer/tree/feature/volume).
