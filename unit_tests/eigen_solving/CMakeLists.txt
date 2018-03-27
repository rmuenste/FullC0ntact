# add a binary
add_executable(eigen_solving eigen_solving.cpp)

# add dependent libraries
target_link_libraries(eigen_solving Utility Math inshape3dcore)


# add include directories
target_include_directories(eigen_solving PUBLIC ${FC_APP_INCLUDE_DIRS})



