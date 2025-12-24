#ifndef TEXTURE_H
#define TEXTURE_H

#include "Vec3f.h"
#include <string>
#include <vector>
#include <memory>

using std::string;
using std::vector;

enum class DecalMode {
    ReplaceKd,
    BlendKd,
    ReplaceKs,
    ReplaceAll,
    ReplaceBackground,
    ReplaceNormal,
    BumpNormal
};

enum class InterpolationMode {
    Nearest,
    Bilinear,
    Trilinear
};

enum class NoiseConversion {
    Linear,
    Absval
};

struct ImageData {
    string filepath;
    int width, height, channels;
    unsigned char* data;  // Raw pixel data
    
    ImageData();
    ~ImageData();

    ImageData(const ImageData&) = delete;
    ImageData& operator=(const ImageData&) = delete;

    ImageData(ImageData&& other) noexcept;
    ImageData& operator=(ImageData&& other) noexcept;

    // Load image from file using stb_image
    bool load(const string& path);

    // Sample with interpolation
    Vec3f sample(float u, float v, InterpolationMode mode) const;
    
private:
    Vec3f sampleNearest(float u, float v) const;
    Vec3f sampleBilinear(float u, float v) const;
    Vec3f sampleTrilinear(float u, float v) const;
    
    // Helper: get pixel at (x, y) with bounds checking
    Vec3f getPixel(int x, int y) const;
};


class TextureMap {
public:
    int id;
    DecalMode decal_mode;
    
    virtual ~TextureMap() = default;
    
    // Image textures için (UV-based)
    virtual Vec3f getValueUV(float u, float v, const std::vector<ImageData>& images) const {
        return Vec3f(255, 0, 255);  // Default: error 
    }
    
    // Procedural textures için (position-based)
    virtual Vec3f getValuePos(const Vec3f& pos) const {
        return Vec3f(255, 0, 255);  // Default: error 
    }
    
    virtual bool needsUV() const = 0;
    
    bool isBumpMap() const { return decal_mode == DecalMode::BumpNormal; }
    bool isNormalMap() const { return decal_mode == DecalMode::ReplaceNormal; }
    bool isBackgroundTexture() const { return decal_mode == DecalMode::ReplaceBackground; }
    
protected:
    TextureMap(int id_, DecalMode mode) : id(id_), decal_mode(mode) {}
};


class ImageTextureMap : public TextureMap {
public:
    int image_id;
    InterpolationMode interpolation;
    float bump_factor;  // for DecalMode::BumpNormal
    float normalizer;
    
    ImageTextureMap(int id, DecalMode mode);
    
    Vec3f getValueUV(float u, float v, const std::vector<ImageData>& images) const override;

    bool needsUV() const override { return true; }
};


class PerlinNoiseMap : public TextureMap {
public:
    NoiseConversion conversion;
    float bump_factor;  // for DecalMode::BumpNormal
    float noise_scale;
    int num_octaves;
    
    PerlinNoiseMap(int id, DecalMode mode);
    
    Vec3f getValuePos(const Vec3f& pos) const override;
    
    bool needsUV() const override { return false; }
    
private:
    float perlinNoise(const Vec3f& p) const;
    float turbulence(const Vec3f& p) const;
    float dot(const Vec3f& a, const Vec3f& b) const;
    float fadeWeight(float d) const;
};

class CheckerboardMap : public TextureMap {
public:
    float offset;
    float scale;
    Vec3f black_color;
    Vec3f white_color;
    
    CheckerboardMap(int id, DecalMode mode);
    
    Vec3f getValuePos(const Vec3f& pos) const override;
    
    bool needsUV() const override { return false; }
};

#endif // TEXTURE_H
