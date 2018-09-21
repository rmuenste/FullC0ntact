
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(meshtest_cgal meshtest_cgal.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(meshtest_cgal meshtest_cgal.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(meshtest_cgal ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(meshtest_cgal PUBLIC ${FC_APP_INCLUDE_DIRS})



