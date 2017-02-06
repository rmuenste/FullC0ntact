
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(test0 test0.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(test0 Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(test0 Utility Math inshape3dcore)
endif(WIN32)


# add include directories
target_include_directories(test0 PUBLIC ${FC_APP_INCLUDE_DIRS})


