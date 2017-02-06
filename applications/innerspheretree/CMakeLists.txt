
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(innerspheretree innerspheretree.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(innerspheretree innerspheretree.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(innerspheretree Utility Math inshape3dcore difi)
else(WIN32)
  # add dependent libraries
  target_link_libraries(innerspheretree Utility Math inshape3dcore difi)
endif(WIN32)


# add include directories
target_include_directories(innerspheretree PUBLIC ${FC_APP_INCLUDE_DIRS})



