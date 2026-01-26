cmake_minimum_required(VERSION 3.15)
project(RayTracer)

# ==========================================
# 1. Dependencies
# ==========================================

# --- A. ZLIB (Compression for .zip VDBs) ---
find_package(ZLIB REQUIRED)

# --- B. Blosc (Compression for .nvdb) ---
# We fetch and build Blosc from source to ensure it exists.
include(FetchContent)
message(STATUS "Fetching Blosc...")

FetchContent_Declare(
  blosc
  GIT_REPOSITORY https://github.com/Blosc/c-blosc.git
  GIT_TAG        v1.21.1
  GIT_SHALLOW    TRUE
)

# Disable Blosc tests/benchmarks to speed up build
set(BLOSC_INSTALL OFF CACHE BOOL "" FORCE)
set(BLOSC_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_FUZZERS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(blosc)

# --- C. NanoVDB Headers ---
message(STATUS "Fetching NanoVDB headers...")
FetchContent_Declare(
  nanovdb_headers
  GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
  GIT_TAG        v11.0.0
  GIT_SHALLOW    TRUE
)
# We strictly populate, no build logic needed for NanoVDB itself
FetchContent_GetProperties(nanovdb_headers)
if(NOT nanovdb_headers_POPULATED)
  FetchContent_Populate(nanovdb_headers)
endif()
set(NANOVDB_INCLUDE_DIR "${nanovdb_headers_SOURCE_DIR}/nanovdb")


# ==========================================
# 2. Define Ray Tracer Executable
# ==========================================
add_executable(raytracer
    raytracer.cpp
    parser.cpp

    
    Aabb.h
    BitCast.h
    Bvh.h
    DebugBvh.h
    HitRecord.h
    Mat4f.h
    MathF.h
    parser.h
    raytracer.h
    Timer.h
    Vec3f.h
)

# ==========================================
# 3. Linking & Defines (The Fix)
# ==========================================

# Link ZLib and the Blosc static library we just built
target_link_libraries(raytracer PRIVATE ZLIB::ZLIB blosc_static)

# Includes
target_include_directories(raytracer PRIVATE ${NANOVDB_INCLUDE_DIR})
target_include_directories(raytracer PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Enable the features in NanoVDB
target_compile_definitions(raytracer PRIVATE 
    NANOVDB_USE_BLOSC=1 
    NANOVDB_USE_ZIP=1
    NANOVDB_USE_OPENVDB=0 
    NANOVDB_USE_CUDA=0
)

target_compile_features(raytracer PRIVATE cxx_std_17)