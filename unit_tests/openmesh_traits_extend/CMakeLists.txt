# add a binary
add_executable(meshtraits_extend meshtraits_extend.cpp)

# add dependent libraries
target_link_libraries(meshtraits_extend OpenMeshCore OpenMeshTools Utility Math inshape3dcore)

# add include directories
target_include_directories(meshtraits_extend PUBLIC ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libs/OpenMesh/src)



