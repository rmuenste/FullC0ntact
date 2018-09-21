
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
target_link_libraries(meshtest ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(meshtest PRIVATE ${FC_APP_INCLUDE_DIRS})




