
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(cgal_tests cgal_tests.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(cgal_tests cgal_tests.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(cgal_tests ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(cgal_tests PUBLIC ${FC_APP_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})
