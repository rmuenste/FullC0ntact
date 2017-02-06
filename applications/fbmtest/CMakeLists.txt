
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(meshtest meshtest.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(meshtest meshtest.cpp)
endif(FC_CUDA_SUPPORT)

# add a binary

if(WIN32)
  # add dependent libraries
  target_link_libraries(meshtest Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(meshtest Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(meshtest PRIVATE ${FC_APP_INCLUDE_DIRS})




