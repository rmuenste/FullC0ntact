
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
cuda_add_executable(particles_gpu particles_gpu.cpp)

# add dependent libraries
if(WIN32)
  target_link_libraries(particles_gpu Utility Math inshape3dcore difi)
else(WIN32)
  target_link_libraries(particles_gpu Utility Math inshape3dcore difi)
endif(WIN32)

# add include directories
target_include_directories(particles_gpu PUBLIC ${FC_APP_INCLUDE_DIRS})
