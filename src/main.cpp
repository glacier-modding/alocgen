#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include <iostream>
#include <ctype.h>
#include <vector>
#include <iomanip>
#include <sstream>
#include "Physics.hpp"

extern "C"
{
	__declspec(dllexport) void InitPhysX();
	__declspec(dllexport) void ReleasePhysX();
	__declspec(dllexport) int NewPhysics();
	__declspec(dllexport) int SetCollisionSettings(Physics::CollisionSettings* settings);
	__declspec(dllexport) int SetDataType(uint32_t data_type);
	__declspec(dllexport) int SetCollisionType(uint32_t collision_type);
	__declspec(dllexport) int AddConvexMesh(uint32_t vertex_count, float* input_vertices, uint32_t triangle_count, uint32_t* input_indices, uint64_t collision_layer);
	__declspec(dllexport) int AddTriangleMesh(uint32_t vertex_count, float* input_vertices, uint32_t triangle_count, uint32_t* input_indices, uint64_t collision_layer);
	__declspec(dllexport) int AddPrimitiveBox(float* half_extents, uint64_t collision_layer, float* position, float* rotation);
	__declspec(dllexport) int AddPrimitiveCapsule(float radius, float length, uint64_t collision_layer, float* position, float* rotation);
	__declspec(dllexport) int AddPrimitiveSphere(float radius, uint64_t collision_layer, float* position, float* rotation);
	__declspec(dllexport) int Write(char* output_path);
}

Physics physics;
physx::PxDefaultAllocator gAllocator;
physx::PxDefaultErrorCallback gErrorCallback;
physx::PxFoundation* gFoundation = NULL;
physx::PxPhysics* gPhysics = NULL;
physx::PxCooking* gCooking = NULL;
bool physx_loaded = false;

int NewPhysics()
{
	physics = Physics();
	return 0;
}

void InitPhysX()
{
	if (!physx_loaded)
	{
		gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);
		gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, physx::PxTolerancesScale(), true);
		gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, physx::PxCookingParams(physx::PxTolerancesScale()));
		physx_loaded = true;
	}
}
	
void ReleasePhysX()
{
	if (physx_loaded)
	{
		gPhysics->release();
		gCooking->release();
		gFoundation->release();
		physx_loaded = false;
	}
}

int SetCollisionSettings(Physics::CollisionSettings* settings)
{
	physics.SetDataType(static_cast<Physics::DataType>(settings->data_type));
	physics.SetCollisionType(static_cast<Physics::CollisionType>(settings->collision_type));
	return 0;
}

int SetDataType(uint32_t data_type)
{
	physics.SetDataType(static_cast<Physics::DataType>(data_type));
	return 0;
}

int SetCollisionType(uint32_t collision_type)
{
	physics.SetCollisionType(static_cast<Physics::CollisionType>(collision_type));
	return 0;
}

int AddConvexMesh(uint32_t vertex_count, float* input_vertices, uint32_t triangle_count, uint32_t* input_indices, uint64_t collision_layer)
{
	InitPhysX();
	std::vector<physx::PxVec3> vertices;
	std::vector<physx::PxU32> indices;
	for (uint32_t v = 0; v < vertex_count / 3; v++)
	{
		vertices.emplace_back();
		vertices.back().x = input_vertices[v * 3];
		vertices.back().y = input_vertices[v * 3 + 1];
		vertices.back().z = input_vertices[v * 3 + 2];
	}
	for (uint32_t i = 0; i < triangle_count * 3; i++)
	{
		indices.emplace_back();
		indices.back() = input_indices[i];
	}
	physics.AddConvexMesh((uint32_t)vertices.size(), vertices.data(), triangle_count, indices.data(), static_cast<Physics::CollisionLayer>(collision_layer), gPhysics, gCooking);
	return 0;
}

int AddTriangleMesh(uint32_t vertex_count, float* input_vertices, uint32_t triangle_count, uint32_t* input_indices, uint64_t collision_layer)
{
	InitPhysX();
	std::vector<physx::PxVec3> vertices;
	std::vector<physx::PxU32> indices;
	for (uint32_t v = 0; v < vertex_count / 3; v++)
	{
		vertices.emplace_back();
		vertices.back().x = input_vertices[v * 3];
		vertices.back().y = input_vertices[v * 3 + 1];
		vertices.back().z = input_vertices[v * 3 + 2];
	}
	for (uint32_t i = 0; i < triangle_count * 3; i++)
	{
		indices.emplace_back();
		indices.back() = input_indices[i];
	}
	physics.AddTriangleMesh((uint32_t)vertices.size(), vertices.data(), triangle_count, indices.data(), static_cast<Physics::CollisionLayer>(collision_layer), gPhysics, gCooking);
	return 0;
}

int AddPrimitiveBox(float* half_extents, uint64_t collision_layer, float* position, float* rotation)
{
	physics.AddPrimitiveBox(
		glm::vec3(half_extents[0], half_extents[1], half_extents[2]),
		static_cast<Physics::CollisionLayer>(collision_layer),
		glm::vec3(position[0], position[1], position[2]),
		glm::vec4(rotation[1], rotation[2], rotation[3], rotation[0])
	);
	return 0;
}

int AddPrimitiveCapsule(float radius, float length, uint64_t collision_layer, float* position, float* rotation)
{
	physics.AddPrimitiveCapsule(
		radius,
		length,
		static_cast<Physics::CollisionLayer>(collision_layer),
		glm::vec3(position[0], position[1], position[2]),
		glm::vec4(rotation[1], rotation[2], rotation[3], rotation[0])
	);
	return 0;
}

int AddPrimitiveSphere(float radius, uint64_t collision_layer, float* position, float* rotation)
{
	physics.AddPrimitiveSphere(
		radius,
		static_cast<Physics::CollisionLayer>(collision_layer),
		glm::vec3(position[0], position[1], position[2]),
		glm::vec4(rotation[1], rotation[2], rotation[3], rotation[0])
	);
	return 0;
}

int Write(char* output_path)
{
	physics.Write(output_path);
	return 0;
}