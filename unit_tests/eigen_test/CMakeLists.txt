# add a binary
add_executable(eigen_test eigen_test.cpp)

# add dependent libraries
target_link_libraries(eigen_test Utility Math inshape3dcore)


# add include directories
target_include_directories(eigen_test PUBLIC ${FC_APP_INCLUDE_DIRS})



