#

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(soft_body2 soft_body2.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(soft_body2 Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(soft_body2 Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(soft_body2 PRIVATE ${FC_APP_INCLUDE_DIRS})





