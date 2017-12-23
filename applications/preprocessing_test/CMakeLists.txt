
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(preprocessing_test preprocessing_test.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(preprocessing_test preprocessing_test.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(preprocessing_test Utility Math inshape3dcore )
else(WIN32)
  # add dependent libraries
  target_link_libraries(preprocessing_test Utility Math inshape3dcore )
endif(WIN32)

# add include directories
target_include_directories(preprocessing_test PRIVATE ${FC_APP_INCLUDE_DIRS})
