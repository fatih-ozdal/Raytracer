#ifndef BVH_H
#define BVH_H

#include "HitRecord.h"

struct TopPrim
{
    PrimKind kind;   // what type of object this refers to
    uint32_t index;  // index into the corresponding object array
    AABB bounds;
    Vec3f centroid;

    TopPrim() = default;
    TopPrim(uint32_t i, PrimKind k, AABB b, Vec3f c) : index(i), kind(k), bounds(b), centroid(c) {}
};

AABB getWorldAABB(const TopPrim& p, const Scene& scene) {
    switch (p.kind) {
        case PrimKind::Sphere:
            return scene.spheres[p.index].localBounds;
        case PrimKind::Triangle:
            return scene.triangles[p.index].localBounds;
        case PrimKind::Mesh:
            return scene.meshes[p.index].localBounds;
        //case PrimKind::MeshInstance:
            //return meshInstances[p.index].localBounds;
    }

    AABB dummy;
    dummy.reset();
    return dummy;
}

Vec3f getCentroid(const TopPrim& p, const Scene& scene) {
    const AABB box = getWorldAABB(p, scene);
    return 0.5f * (box.min + box.max);
}

struct BVHNode
{
    AABB bounds;
    int  leftNodeIdx;   // index into bvhNodes, right = left + 1 always
    int firstPrimIdx;   // index into topPrims, valid only if leaf
    int  primCount;  

    bool isLeaf() const noexcept { return primCount >= 0; }
};

#endif // BVH_H
