
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(voxelizationtest voxelizationtest.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(voxelizationtest voxelizationtest.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(voxelizationtest Utility Math inshape3dcore )
else(WIN32)
  # add dependent libraries
  target_link_libraries(voxelizationtest Utility Math inshape3dcore )
endif(WIN32)


# add include directories
target_include_directories(voxelizationtest PUBLIC ${FC_APP_INCLUDE_DIRS})



