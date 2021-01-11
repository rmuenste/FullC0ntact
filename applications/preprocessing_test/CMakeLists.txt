
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(preprocessing_test preprocessing_test.cpp)
else(NOT FC_CUDA_SUPPORT)

# add a binary
cuda_add_executable(preprocessing_test preprocessing_test.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(preprocessing_test ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(preprocessing_test PRIVATE ${FC_APP_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})
