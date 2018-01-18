
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(duckpond duckpond.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(duckpond ${FC_APP_LIB_DIRS})
else(WIN32)
  # add dependent libraries
  target_link_libraries(duckpond ${FC_APP_LIB_DIRS})
endif(WIN32)

# add include directories
target_include_directories(duckpond PRIVATE ${FC_APP_INCLUDE_DIRS})

