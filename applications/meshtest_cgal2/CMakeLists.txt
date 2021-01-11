
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(meshtest_cgal2 meshtest_cgal2.cpp)
else(NOT FC_CUDA_SUPPORT)

# add a binary
cuda_add_executable(meshtest_cgal2 meshtest_cgal2.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(meshtest_cgal2 ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(meshtest_cgal2 PUBLIC ${FC_APP_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})



