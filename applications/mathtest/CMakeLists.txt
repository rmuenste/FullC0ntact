
# add a binary
add_executable(mathtest mathtest.cpp)

# add dependent libraries
target_link_libraries(mathtest Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})


# add include directories
target_include_directories(mathtest PUBLIC ${FC_APP_INCLUDE_DIRS})



