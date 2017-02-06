
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(distmap_test2 distmap_test2.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(distmap_test2 distmap_test2.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(distmap_test2 Utility Math inshape3dcore difi)
else(WIN32)
  # add dependent libraries
  target_link_libraries(distmap_test2 Utility Math inshape3dcore difi)
endif(WIN32)

# add include directories
target_include_directories(distmap_test2 PUBLIC ${FC_APP_INCLUDE_DIRS})




