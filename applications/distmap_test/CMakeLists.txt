
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
cuda_add_executable(distmap_test distmap_test.cpp)

# add dependent libraries
if(WIN32)
  target_link_libraries(distmap_test Utility Math inshape3dcore difi)
else(WIN32)
  target_link_libraries(distmap_test Utility Math inshape3dcore difi)
endif(WIN32)

# add include directories
target_include_directories(distmap_test PUBLIC ${FC_APP_INCLUDE_DIRS})
