
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(test1 test1.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(test1 Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(test1 Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories(test1 PUBLIC ${FC_APP_INCLUDE_DIRS})



