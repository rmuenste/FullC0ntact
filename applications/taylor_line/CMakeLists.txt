
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(taylor_line taylor_line.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(taylor_line Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(taylor_line Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(taylor_line PRIVATE ${FC_APP_INCLUDE_DIRS})





