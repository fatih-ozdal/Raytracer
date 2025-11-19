#include "Bvh.h"
#include <iostream>

struct BvhStats {
    int nodeCount = 0;
    int leafCount = 0;
    int interiorCount = 0;
    int maxDepth = 0;
    int totalLeafPrims = 0;
    int maxLeafPrims = 0;
};

void GatherStats(uint32_t nodeIdx, int depth, BvhStats& stats, const vector<BVHNode>& topBvhNodes)
{
    if (nodeIdx == UINT32_MAX) return;
    const BVHNode& node = topBvhNodes[nodeIdx];

    stats.nodeCount++;
    stats.maxDepth = std::max(stats.maxDepth, depth);

    if (node.primCount > 0) { // leaf
        stats.leafCount++;
        stats.totalLeafPrims += node.primCount;
        stats.maxLeafPrims = std::max(stats.maxLeafPrims, (int)node.primCount);
    } else { // interior
        stats.interiorCount++;
        uint32_t left = node.leftNodeIdx;
        uint32_t right = node.leftNodeIdx + 1;
        GatherStats(left, depth + 1, stats, topBvhNodes);
        GatherStats(right, depth + 1, stats, topBvhNodes);
    }
}

struct MeshBvhStats {
    int nodeCount       = 0;
    int leafCount       = 0;
    int interiorCount   = 0;
    int maxDepth        = 0;
    int totalLeafPrims  = 0;
    int maxLeafPrims    = 0;
};

void GatherMeshBvhStats(const MeshBVH& bvh, uint32_t nodeIdx, int depth, MeshBvhStats& stats)
{
    if (nodeIdx >= bvh.nodes.size())
        return;

    const BVHNode& node = bvh.nodes[nodeIdx];

    if (node.primCount == 0 && node.leftNodeIdx == 0 && nodeIdx != bvh.rootNodeIdx) {
        return;
    }

    stats.nodeCount++;
    if (depth > stats.maxDepth) stats.maxDepth = depth;

    if (node.primCount > 0) {
        stats.leafCount++;
        stats.totalLeafPrims += node.primCount;
        if (node.primCount > stats.maxLeafPrims)
            stats.maxLeafPrims = node.primCount;
    } else {
        stats.interiorCount++;
        uint32_t leftChild  = node.leftNodeIdx;
        uint32_t rightChild = node.leftNodeIdx + 1;
        GatherMeshBvhStats(bvh, leftChild,  depth + 1, stats);
        GatherMeshBvhStats(bvh, rightChild, depth + 1, stats);
    }
}

void PrintBvhStats(const Scene& scene, const vector<BVHNode>& topBvhNodes, uint32_t rootNodeIdx) {
    BvhStats s;
    GatherStats(rootNodeIdx, 0, s, topBvhNodes);
    std::cout << "Top-level BVH: nodes=" << s.nodeCount
              << " leaves=" << s.leafCount
              << " interior=" << s.interiorCount
              << " maxDepth=" << s.maxDepth
              << " avgLeafPrim=" << (s.leafCount > 0 ? float(s.totalLeafPrims)/s.leafCount : 0)
              << " maxLeafPrim=" << s.maxLeafPrims << std::endl;
    
    for (size_t i = 0; i < meshBVHs.size(); i++) {
        if (scene.meshes[i].isInstance) continue; // Skip instances
        MeshBvhStats ms;
        GatherMeshBvhStats(meshBVHs[i], meshBVHs[i].rootNodeIdx, 0, ms);
        std::cout << "Mesh " << i << " BVH: nodes=" << ms.nodeCount
                  << " leaves=" << ms.leafCount
                  << " interior=" << ms.interiorCount
                  << " maxDepth=" << ms.maxDepth
                  << " avgLeafPrim=" << (ms.leafCount > 0 ? float(ms.totalLeafPrims)/ms.leafCount : 0)
                  << " maxLeafPrim=" << ms.maxLeafPrims << std::endl;
    }
}
