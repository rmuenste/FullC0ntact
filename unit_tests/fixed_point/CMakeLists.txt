# add a binary
add_executable(fixed_point fixed_point.cpp)

# add dependent libraries
target_link_libraries(fixed_point Utility Math inshape3dcore)


# add include directories
target_include_directories(fixed_point PUBLIC ${FC_APP_INCLUDE_DIRS})

