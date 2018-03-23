# add a binary
add_executable(interpolation interpolation.cpp)

# add dependent libraries
target_link_libraries(interpolation Utility Math inshape3dcore)


# add include directories
target_include_directories(interpolation PUBLIC ${FC_APP_INCLUDE_DIRS})



