
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(boxes_sticks boxes_sticks.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(boxes_sticks Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(boxes_sticks Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(boxes_sticks PRIVATE ${FC_APP_INCLUDE_DIRS})




