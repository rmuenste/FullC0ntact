# add a binary
add_executable(omesh_tests omesh_tests.cpp)

# add dependent libraries
target_link_libraries(omesh_tests OpenMeshCore OpenMeshTools Utility Math inshape3dcore)

# add include directories
target_include_directories(omesh_tests PUBLIC ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libs/OpenMesh/src)



