cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(BUILD_SHARED_LIBS True)

project(alocgen VERSION 1.0.0)

add_library(alocgen 
	${PROJECT_SOURCE_DIR}/src/main.cpp)

target_include_directories(alocgen PUBLIC
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/vendor
	${PROJECT_SOURCE_DIR}/vendor/Physx-3.4/PhysX_3.4/Include
	${PROJECT_SOURCE_DIR}/vendor/Physx-3.4/PxShared/include)
	
target_link_directories(alocgen PUBLIC
	${PROJECT_SOURCE_DIR}/vendor/Physx-3.4/PhysX_3.4/Lib
	${PROJECT_SOURCE_DIR}/vendor/Physx-3.4/PxShared/lib)

#target_link_libraries(alocgen PUBLIC LowLevelAABBDEBUG.lib)
#target_link_libraries(alocgen PUBLIC LowLevelClothDEBUG.lib)
#target_link_libraries(alocgen PUBLIC LowLevelDEBUG.lib)
#target_link_libraries(alocgen PUBLIC LowLevelDynamicsDEBUG.lib)
#target_link_libraries(alocgen PUBLIC LowLevelParticlesDEBUG.lib)
#target_link_libraries(alocgen PUBLIC PhysX3CharacterKinematicDEBUG_x64.lib)
#target_link_libraries(alocgen PUBLIC PhysX3CommonDEBUG_x64.lib)
target_link_libraries(alocgen PUBLIC PhysX3Cooking_x64.lib)
target_link_libraries(alocgen PUBLIC PhysX3_x64.lib)
target_link_libraries(alocgen PUBLIC PhysX3Extensions.lib)
#target_link_libraries(alocgen PUBLIC PhysX3VehicleDEBUG.lib)
#target_link_libraries(alocgen PUBLIC SceneQueryDEBUG.lib)
#target_link_libraries(alocgen PUBLIC SimulationControllerDEBUG.lib)
#target_link_libraries(alocgen PUBLIC PsFastXmlDEBUG_x64.lib)
target_link_libraries(alocgen PUBLIC PxFoundation_x64.lib)
#target_link_libraries(alocgen PUBLIC PxPvdSDKDEBUG_x64.lib)
#target_link_libraries(alocgen PUBLIC PxTaskDEBUG_x64.lib)