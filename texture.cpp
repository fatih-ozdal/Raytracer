#include "texture.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using std::floor;
using std::round;

ImageData::ImageData() 
    : width(0), height(0), channels(0), data(nullptr) {}

ImageData::~ImageData() {
    if (data) {
        stbi_image_free(data);
        data = nullptr;
    }
}

ImageData::ImageData(ImageData&& other) noexcept
    : filepath(std::move(other.filepath)),
      width(other.width), height(other.height), 
      channels(other.channels), data(other.data) {
    other.data = nullptr;
}

ImageData& ImageData::operator=(ImageData&& other) noexcept {
    if (this != &other) {
        // Clean up existing data
        if (data) {
            stbi_image_free(data);
        }
        
        // Transfer ownership
        filepath = std::move(other.filepath);
        width = other.width;
        height = other.height;
        channels = other.channels;
        data = other.data;
        
        other.data = nullptr;
    }
    return *this;
}

bool ImageData::load(const std::string& path) {
    filepath = path;
    
    // Load image using stb_image
    data = stbi_load(path.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        std::cerr << "Failed to load texture: " << path << std::endl;
        std::cerr << "STB Error: " << stbi_failure_reason() << std::endl;
        return false;
    }
    
    std::cout << "Loaded texture: " << path 
              << " (" << width << "x" << height 
              << ", " << channels << " channels)" << std::endl;
    
    return true;
}

Vec3f ImageData::sample(float u, float v, InterpolationMode mode) const {
    switch (mode) {
        case InterpolationMode::Nearest:
            return sampleNearest(u, v);
        case InterpolationMode::Bilinear:
            return sampleBilinear(u, v);
        case InterpolationMode::Trilinear:
            return sampleTrilinear(u, v);
        default:
            return sampleNearest(u, v);
    }
}

Vec3f ImageData::sampleNearest(float u, float v) const {
    // Wrap UV to [0, 1]
    u = u - floor(u);
    v = v - floor(v);

    float x = u * width;
    float y = v * height;

    int i = std::min((int) x, width - 1);
    int j = std::min((int) y, height - 1);
    
    return getPixel(i, j);
}

Vec3f ImageData::sampleBilinear(float u, float v) const {
    // Wrap UV to [0, 1]
    u = u - floor(u);
    v = v - floor(v);
    
    float x = u * width;
    float y = v * height;

    float x_center = x - 0.5f;
    float y_center = y - 0.5f;
    
    int p0 = static_cast<int>(floor(x_center));
    int q0 = static_cast<int>(floor(y_center));
    
    float dx = x_center - p0;
    float dy = y_center - q0;
    
    // Wrap coordinates for neighbors
    int p1 = (p0 + 1) % width;
    int q1 = (q0 + 1) % height;
    
    // Get four neighbor pixels
    Vec3f c_p0q0 = getPixel(p0, q0);  // (p, q)
    Vec3f c_p1q0 = getPixel(p1, q0);  // (p+1, q)
    Vec3f c_p0q1 = getPixel(p0, q1);  // (p, q+1)
    Vec3f c_p1q1 = getPixel(p1, q1);  // (p+1, q+1)
    
    // Bilinear interpolation
    Vec3f result = c_p0q0 * (1.0f - dx) * (1.0f - dy) +
                   c_p1q0 * dx * (1.0f - dy) +
                   c_p0q1 * (1.0f - dx) * dy +
                   c_p1q1 * dx * dy;
    
    return result;
}

Vec3f ImageData::sampleTrilinear(float u, float v) const {
    // Bonus: Mipmapping implementation
    // For now, just use bilinear
    return sampleBilinear(u, v);
}

Vec3f ImageData::getPixel(int x, int y) const {
    // Clamp coordinates (safety)
    x = std::max(0, std::min(x, width - 1));
    y = std::max(0, std::min(y, height - 1));
    
    int idx = (y * width + x) * channels;
    
    if (channels == 1) // Grayscale
    {
        float gray = data[idx];
        return Vec3f(gray, gray, gray);
    } 
    else if (channels == 3) // RGB
    {
        return Vec3f(
            data[idx + 0],
            data[idx + 1],
            data[idx + 2]
        );
    } 
    else if (channels == 4) // RGBA
    {
        return Vec3f(
            data[idx + 0],
            data[idx + 1],
            data[idx + 2]
        );
    }
    
    return Vec3f(255, 0, 255);  // Magenta for error
}


ImageTextureMap::ImageTextureMap(int id, DecalMode mode)
    : TextureMap(id, mode), image_id(-1), 
      interpolation(InterpolationMode::Nearest),
      bump_factor(1.0f), normalizer(255) {}

Vec3f ImageTextureMap::getValueUV(float u, float v, const std::vector<ImageData>& images) const 
{
    if (image_id < 0 || image_id >= static_cast<int>(images.size())) {
        std::cerr << "Invalid image_id: " << image_id << std::endl;
        return Vec3f(1, 0, 1);  // Magenta error
    }
    
    return images[image_id].sample(u, v, interpolation) / normalizer;
}

PerlinNoiseMap::PerlinNoiseMap(int id, DecalMode mode)
    : TextureMap(id, mode), conversion(NoiseConversion::Linear),
      bump_factor(1.0f), noise_scale(1.0f), num_octaves(1) {}

Vec3f PerlinNoiseMap::getValuePos(const Vec3f& pos) const {
    float noise = turbulence(pos * noise_scale);
    
    // Apply conversion mode
    if (conversion == NoiseConversion::Absval) {
        noise = std::abs(noise);
    } else {
        // Linear: map [-1, 1] to [0, 1]
        noise = (noise + 1.0f) * 0.5f;
    }
    
    // Clamp to [0, 1]
    noise = std::max(0.0f, std::min(1.0f, noise));
    
    return Vec3f(noise, noise, noise);
}

float PerlinNoiseMap::turbulence(const Vec3f& p) const {
    float sum = 0.0f;
    float weight = 1.0f;
    Vec3f pos = p;
    
    for (int i = 0; i < num_octaves; i++) {
        sum += weight * perlinNoise(pos);
        weight *= 0.5f;
        pos = pos * 2.0f;
    }
    
    return sum;
}

float PerlinNoiseMap::perlinNoise(const Vec3f& p) const {
    // Gradient vectors from slide (16 gradients)
    static const Vec3f gradients[16] = {
        Vec3f(1, 1, 0),   Vec3f(-1, 1, 0),   Vec3f(1, -1, 0),   Vec3f(-1, -1, 0),
        Vec3f(1, 0, 1),   Vec3f(-1, 0, 1),   Vec3f(1, 0, -1),   Vec3f(-1, 0, -1),
        Vec3f(0, 1, 1),   Vec3f(0, -1, 1),   Vec3f(0, 1, -1),   Vec3f(0, -1, -1),
        Vec3f(1, 1, 0),   Vec3f(-1, 1, 0),   Vec3f(0, -1, 1),   Vec3f(0, -1, -1)
    };
    
    // Hash table - shuffled
    static const int table[16] = {
        10, 2, 7, 14, 1, 13, 4, 9, 6, 0, 15, 8, 3, 11, 5, 12
    };
    
    // Step 1: Find lattice cell containing point
    int i = static_cast<int>(std::floor(p.x));
    int j = static_cast<int>(std::floor(p.y));
    int k = static_cast<int>(std::floor(p.z));
    
    // Step 2: Find relative position in cell (fractional part)
    float dx = p.x - i;
    float dy = p.y - j;
    float dz = p.z - k;
    
    // Step 3: Get gradient indices for 8 corners 
    int idx;
    idx = table[std::abs(k) % 16];
    idx = table[std::abs(j + idx) % 16];
    idx = table[std::abs(i + idx) % 16];
    Vec3f g000 = gradients[idx];
    
    idx = table[std::abs(k) % 16];
    idx = table[std::abs(j + idx) % 16];
    idx = table[std::abs(i + 1 + idx) % 16];
    Vec3f g100 = gradients[idx];
    
    idx = table[std::abs(k) % 16];
    idx = table[std::abs(j + 1 + idx) % 16];
    idx = table[std::abs(i + idx) % 16];
    Vec3f g010 = gradients[idx];
    
    idx = table[std::abs(k) % 16];
    idx = table[std::abs(j + 1 + idx) % 16];
    idx = table[std::abs(i + 1 + idx) % 16];
    Vec3f g110 = gradients[idx];
    
    idx = table[std::abs(k + 1) % 16];
    idx = table[std::abs(j + idx) % 16];
    idx = table[std::abs(i + idx) % 16];
    Vec3f g001 = gradients[idx];
    
    idx = table[std::abs(k + 1) % 16];
    idx = table[std::abs(j + idx) % 16];
    idx = table[std::abs(i + 1 + idx) % 16];
    Vec3f g101 = gradients[idx];
    
    idx = table[std::abs(k + 1) % 16];
    idx = table[std::abs(j + 1 + idx) % 16];
    idx = table[std::abs(i + idx) % 16];
    Vec3f g011 = gradients[idx];
    
    idx = table[std::abs(k + 1) % 16];
    idx = table[std::abs(j + 1 + idx) % 16];
    idx = table[std::abs(i + 1 + idx) % 16];
    Vec3f g111 = gradients[idx];
    
    // Step 4: Compute dot products (distance vectors vs gradients)
    float d000 = dot(Vec3f(dx,   dy,   dz),   g000);
    float d100 = dot(Vec3f(dx-1, dy,   dz),   g100);
    float d010 = dot(Vec3f(dx,   dy-1, dz),   g010);
    float d110 = dot(Vec3f(dx-1, dy-1, dz),   g110);
    float d001 = dot(Vec3f(dx,   dy,   dz-1), g001);
    float d101 = dot(Vec3f(dx-1, dy,   dz-1), g101);
    float d011 = dot(Vec3f(dx,   dy-1, dz-1), g011);
    float d111 = dot(Vec3f(dx-1, dy-1, dz-1), g111);
    
    // Step 5: Compute weights using fade function (slide formula) 
    float c000 = fadeWeight(dx)   * fadeWeight(dy)   * fadeWeight(dz)   * d000;
    float c100 = fadeWeight(dx-1) * fadeWeight(dy)   * fadeWeight(dz)   * d100;
    float c010 = fadeWeight(dx)   * fadeWeight(dy-1) * fadeWeight(dz)   * d010;
    float c110 = fadeWeight(dx-1) * fadeWeight(dy-1) * fadeWeight(dz)   * d110;
    float c001 = fadeWeight(dx)   * fadeWeight(dy)   * fadeWeight(dz-1) * d001;
    float c101 = fadeWeight(dx-1) * fadeWeight(dy)   * fadeWeight(dz-1) * d101;
    float c011 = fadeWeight(dx)   * fadeWeight(dy-1) * fadeWeight(dz-1) * d011;
    float c111 = fadeWeight(dx-1) * fadeWeight(dy-1) * fadeWeight(dz-1) * d111;
    
    float result = c000 + c100 + c010 + c110 + c001 + c101 + c011 + c111;
    
    return result;
}

float PerlinNoiseMap::dot(const Vec3f& a, const Vec3f& b) const {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float PerlinNoiseMap::fadeWeight(float d) const {
    float absD = std::abs(d);
    if (absD >= 1.0f) return 0.0f;
    
    return -6 * std::pow(absD, 5) + 15 * std::pow(absD, 4) - 10 * std::pow(absD, 3) + 1;
}

CheckerboardMap::CheckerboardMap(int id, DecalMode mode)
    : TextureMap(id, mode), offset(0.0f), scale(1.0f),
      black_color(0, 0, 0), white_color(1, 1, 1) {}

Vec3f CheckerboardMap::getValuePos(const Vec3f& pos) const {
    // Apply offset and scale
    bool x = (static_cast<int>(floor((pos.x + offset) * scale))) % 2;
    bool y = (static_cast<int>(floor((pos.y + offset) * scale))) % 2;
    bool z = (static_cast<int>(floor((pos.z + offset) * scale))) % 2;
    
    bool xorXY = (x != y);
    
    if (xorXY != z) {
        return black_color;
    } else {
        return white_color;
    }
}
