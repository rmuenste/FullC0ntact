
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(velocitybased velocitybased.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(velocitybased velocitybased.cpp)
endif(FC_CUDA_SUPPORT)

target_link_libraries(velocitybased ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(velocitybased PRIVATE ${FC_APP_INCLUDE_DIRS})




