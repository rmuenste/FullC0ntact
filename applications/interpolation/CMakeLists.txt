
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(interpolation interpolation.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(interpolation interpolation.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(interpolation Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(interpolation Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(interpolation PRIVATE ${FC_APP_INCLUDE_DIRS})



