# add a binary
add_executable(interpolation2 interpolation.cpp)

# add dependent libraries
target_link_libraries(interpolation2 Utility Math inshape3dcore)


# add include directories
target_include_directories(interpolation2 PUBLIC ${FC_APP_INCLUDE_DIRS})



