#pragma once
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "StreamWriter.hpp"
#include "foundation/PxPreprocessor.h"
#include "PxPhysicsAPI.h"
#include <iostream>
#include <vector>
#include <ctype.h>
#include <iomanip>
#include <sstream>

class Physics
{
public:
#pragma pack(push, 1)
    enum class DataType : uint32_t
    {
        NONE = 0,
        CONVEX_MESH = 1,
        TRIANGLE_MESH = 2,
        PRIMITIVE = 4,
        SHATTER = 144,
        KINEMATIC_LINKED = 192
    };

    enum class CollisionType : uint32_t
    {
        NONE = 0,
        STATIC = 1,
        RIGIDBODY = 2,
        SHATTER = 16,
        KINEMATIC_LINKED = 32,
        BACKWARD_COMPATIBLE = 2147483647
    };

    enum class CollisionLayer : uint64_t
    {
        COLLIDE_WITH_ALL = 0,
        STATIC_COLLIDABLES_ONLY = 1,
        DYNAMIC_COLLIDABLES_ONLY = 2,
        STAIRS = 3,
        SHOT_ONLY_COLLISION = 4,
        DYNAMIC_TRASH_COLLIDABLES = 5,
        KINEMATIC_COLLIDABLES_ONLY = 6,
        STATIC_COLLIDABLES_ONLY_TRANSPARENT = 7,
        DYNAMIC_COLLIDABLES_ONLY_TRANSPARENT = 8,
        KINEMATIC_COLLIDABLES_ONLY_TRANSPARENT = 9,
        STAIRS_STEPS = 10,
        STAIRS_SLOPE = 11,
        HERO_PROXY = 12,
        ACTOR_PROXY = 13,
        HERO_VR = 14,
        CLIP = 15,
        ACTOR_RAGDOLL = 16,
        CROWD_RAGDOLL = 17,
        LEDGE_ANCHOR = 18,
        ACTOR_DYN_BODY = 19,
        HERO_DYN_BODY = 20,
        ITEMS = 21,
        WEAPONS = 22,
        COLLISION_VOLUME_HITMAN_ON = 23,
        COLLISION_VOLUME_HITMAN_OFF = 24,
        DYNAMIC_COLLIDABLES_ONLY_NO_CHARACTER = 25,
        DYNAMIC_COLLIDABLES_ONLY_NO_CHARACTER_TRANSPARENT = 26,
        COLLIDE_WITH_STATIC_ONLY = 27,
        AI_VISION_BLOCKER = 28,
        AI_VISION_BLOCKER_AMBIENT_ONLY = 29,
        UNUSED_LAST = 30
    };

    enum class PrimitiveType : uint32_t
    {
        BOX = 0,
        CAPSULE = 1,
        SPHERE = 2
    };

    struct Header
    {
        DataType data_type = DataType::NONE;
        CollisionType collision_type = CollisionType::NONE;
        char id_string[2] = {'I', 'D'};
        uint32_t id = 5;
        char physx_string[5] = {'P', 'h', 'y', 's', 'X',};
    };

    struct ConvexHeader
    {
        char type_string[4] = {'C', 'V', 'X', '\0'};
        uint32_t count = 0;
    };

    struct TriangleHeader
    {
        char type_string[4] = {'T', 'R', 'I', '\0'};
        uint32_t count = 0;
    };

    struct PrimitiveHeader
    {
        char type_string[4] = {'I', 'C', 'P', '\0'};
        uint32_t count = 0;
    };

    struct ShatterHeader
    {

    };

    struct KinematicLinkedHeader
    {

    };

    struct ConvexMesh
    {
        CollisionLayer collision_layer = CollisionLayer::COLLIDE_WITH_ALL;
        glm::vec3 position = {0.0f, 0.0f, 0.0f};
        glm::vec4 rotation = {0.0f, 0.0f, 0.0f, 1.0f};
        std::vector<char> convex_mesh;
    };

    struct TriangleMesh
    {
        CollisionLayer collision_layer = CollisionLayer::COLLIDE_WITH_ALL;
        std::vector<char> triangle_mesh;
    };

    struct PrimitiveBox
    {
        char type_string[4] = {'B', 'O', 'X', '\0'};
        glm::vec3 half_extents = {0.0f, 0.0f, 0.0f};
        CollisionLayer collision_layer = CollisionLayer::COLLIDE_WITH_ALL;
        glm::vec3 position = {0.0f, 0.0f, 0.0f};
        glm::vec4 rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    };

    struct PrimitiveCapsule
    {
        char type_string[4] = {'C', 'A', 'P', '\0'};
        float radius = 0.0f;
        float length = 0.0f;
        CollisionLayer collision_layer = CollisionLayer::COLLIDE_WITH_ALL;
        glm::vec3 position = {0.0f, 0.0f, 0.0f};
        glm::vec4 rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    };

    struct PrimitiveSphere
    {
        char type_string[4] = {'S', 'P', 'H', '\0'};
        float radius = 0.0f;
        CollisionLayer collision_layer = CollisionLayer::COLLIDE_WITH_ALL;
        glm::vec3 position = {0.0f, 0.0f, 0.0f};
        glm::vec4 rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    };

    struct CollisionSettings
    {
        uint32_t data_type = (uint32_t)DataType::NONE;
        uint32_t collision_type = (uint32_t)CollisionType::NONE;
    };

    struct Aloc
    {
        Header header;
        std::vector<ConvexMesh> convex_meshes;
        std::vector<TriangleMesh> triangle_meshes;
        std::vector<PrimitiveBox> boxes;
        std::vector<PrimitiveCapsule> capsules;
        std::vector<PrimitiveSphere> spheres;
    };
#pragma pack(pop)

    Physics()
    {

    }

    void SetDataType(DataType data_type)
    {
        aloc.header.data_type = data_type;
    }

    void SetCollisionType(CollisionType collision_type)
    {
        aloc.header.collision_type = collision_type;
    }

    void AddConvexMesh(physx::PxU32 vertex_count, physx::PxVec3* vertices, physx::PxU32 triangle_count, physx::PxU32* indices, CollisionLayer collision_layer, physx::PxPhysics* gPhysics, physx::PxCooking* gCooking)
    {
        physx::PxCookingParams params = gCooking->getParams();

        // If inflation is used, the legacy incremental hull creation algorithm is picked.
        // Without inflation the new default quickhull algorithm is used.

        // Use the new (default) physx::PxConvexMeshCookingType::eQUICKHULL or the legacy physx::PxConvexMeshCookingType::eINFLATION_INCREMENTAL_HULL.
        params.convexMeshCookingType = physx::PxConvexMeshCookingType::eQUICKHULL;

        // If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
        // If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
        params.gaussMapLimit = 16;
        gCooking->setParams(params);

        // Setup the convex mesh descriptor
        physx::PxConvexMeshDesc desc;

        // We provide points only, therefore the physx::PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
        desc.points.data = vertices;
        desc.points.count = vertex_count;
        desc.points.stride = sizeof(physx::PxVec3);
        desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

        physx::PxU32 mesh_size = 0;

        // Serialize the cooked mesh into a stream.
        physx::PxDefaultMemoryOutputStream output_stream;
        bool res = gCooking->cookConvexMesh(desc, output_stream);
        PX_UNUSED(res);
        PX_ASSERT(res);
        mesh_size = output_stream.getSize();

        aloc.convex_meshes.emplace_back();
        aloc.convex_meshes.back().collision_layer = collision_layer;
        aloc.convex_meshes.back().convex_mesh = std::vector<char>((char*)output_stream.getData(), (char*)output_stream.getData() + output_stream.getSize());        
    }

    void AddTriangleMesh(physx::PxU32 vertex_count, physx::PxVec3* vertices, physx::PxU32 triangle_count, physx::PxU32* indices, CollisionLayer collision_layer, physx::PxPhysics* gPhysics, physx::PxCooking* gCooking)
    {
        bool skip_mesh_cleanup = true;
        bool skip_edge_data = false;
        bool inserted = false;
        const physx::PxU32 tris_per_leaf = 4;
        physx::PxTriangleMeshDesc mesh_description;
        mesh_description.points.count = vertex_count;
        mesh_description.points.data = vertices;
        mesh_description.points.stride = sizeof(physx::PxVec3);
        mesh_description.triangles.count = triangle_count;
        mesh_description.triangles.data = indices;
        mesh_description.triangles.stride = 3 * sizeof(physx::PxU32);

        physx::PxCookingParams params = gCooking->getParams();

        // Create BVH34 midphase
        params.midphaseDesc = physx::PxMeshMidPhase::eBVH34;

		// we suppress the triangle mesh remap table computation to gain some speed, as we will not need it 
        // in this snippet
        params.suppressTriangleMeshRemapTable = false;

        // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
        // The following conditions are true for a valid triangle mesh :
        //  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
        //  2. There are no large triangles(within specified PxTolerancesScale.)
        // It is recommended to run a separate validation check in debug/checked builds, see below.

        if (!skip_mesh_cleanup)
            params.meshPreprocessParams &= ~static_cast<physx::PxMeshPreprocessingFlags>(physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
        else
            params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

        // If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
        // marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
        // the collision behavior, as all edges of the triangle mesh will now be considered active.
        if (!skip_edge_data)
            params.meshPreprocessParams &= ~static_cast<physx::PxMeshPreprocessingFlags>(physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
        else
            params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;

        // Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
        // and worse cooking performance. Cooking time is better when more triangles per leaf are used.
        params.midphaseDesc.mBVH34Desc.numTrisPerLeaf = tris_per_leaf;

        gCooking->setParams(params);

        physx::PxDefaultMemoryOutputStream output_stream;
        gCooking->cookTriangleMesh(mesh_description, output_stream);
        
        aloc.triangle_meshes.emplace_back();
        aloc.triangle_meshes.back().collision_layer = collision_layer;
        aloc.triangle_meshes.back().triangle_mesh = std::vector<char>((char*)output_stream.getData(), (char*)output_stream.getData() + output_stream.getSize());
    }

    void AddPrimitiveBox(glm::vec3 half_extents, CollisionLayer collision_layer, glm::vec3 position, glm::vec4 rotation)
    {
        aloc.boxes.emplace_back();
        aloc.boxes.back().half_extents = half_extents;
        aloc.boxes.back().collision_layer = collision_layer;
        aloc.boxes.back().position = position;
        aloc.boxes.back().rotation = rotation;
    }

    void AddPrimitiveCapsule(float radius, float length, CollisionLayer collision_layer, glm::vec3 position, glm::vec4 rotation)
    {
        aloc.capsules.emplace_back();
        aloc.capsules.back().radius = radius;
        aloc.capsules.back().length = length;
        aloc.capsules.back().collision_layer = collision_layer;
        aloc.capsules.back().position = position;
        aloc.capsules.back().rotation = rotation;
    }

    void AddPrimitiveSphere(float radius, CollisionLayer collision_layer, glm::vec3 position, glm::vec4 rotation)
    {
        aloc.spheres.emplace_back();
        aloc.spheres.back().radius = radius;
        aloc.spheres.back().collision_layer = collision_layer;
        aloc.spheres.back().position = position;
        aloc.spheres.back().rotation = rotation;
    }

    void Write(char* output_path)
    {
        StreamWriter aloc_stream;
        aloc_stream.Write<Header>(&aloc.header);
        ConvexHeader convex_header;
        convex_header.count = (uint32_t)aloc.convex_meshes.size();
        if (convex_header.count > 0)
        {
            aloc_stream.Write<ConvexHeader>(&convex_header);
            for (auto& convex_mesh : aloc.convex_meshes)
            {
                aloc_stream.Write<CollisionLayer>(&convex_mesh.collision_layer);
                aloc_stream.Write<glm::vec3>(&convex_mesh.position);
                aloc_stream.Write<glm::vec4>(&convex_mesh.rotation);
                aloc_stream.Write<char>(convex_mesh.convex_mesh.data(), convex_mesh.convex_mesh.size());
            }
        }
        TriangleHeader triangle_header;
        triangle_header.count = (uint32_t)aloc.triangle_meshes.size();
        if (triangle_header.count > 0)
        {
            aloc_stream.Write<TriangleHeader>(&triangle_header);
            for (auto& triangle_mesh : aloc.triangle_meshes)
            {   
                aloc_stream.Write<CollisionLayer>(&triangle_mesh.collision_layer);
                aloc_stream.Write<char>(triangle_mesh.triangle_mesh.data(), triangle_mesh.triangle_mesh.size());
            }
        }
        PrimitiveHeader primitive_header;
        primitive_header.count = (uint32_t)aloc.boxes.size() + (uint32_t)aloc.capsules.size() + (uint32_t)aloc.spheres.size();
        if (primitive_header.count > 0)
        {
            aloc_stream.Write<PrimitiveHeader>(&primitive_header);
            for (auto& box : aloc.boxes)
            {
                aloc_stream.Write<PrimitiveBox>(&box);
            }
            for (auto& capsule : aloc.capsules)
            {
                aloc_stream.Write<PrimitiveCapsule>(&capsule);
            }
            for (auto& sphere : aloc.spheres)
            {
                aloc_stream.Write<PrimitiveSphere>(&sphere);
            }
        }
        aloc_stream.WriteToFile(std::string(output_path));
    }

private:
    Aloc aloc;
};