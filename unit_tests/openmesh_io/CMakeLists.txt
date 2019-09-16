# add a binary
add_executable(io_test io_test.cpp)

# add dependent libraries
target_link_libraries(io_test OpenMeshCore OpenMeshTools Utility Math inshape3dcore)

# add include directories
target_include_directories(io_test PUBLIC ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libs/OpenMesh/src)



