
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(simple_cgal simple_cgal.cpp)

target_link_libraries(simple_cgal Utility Math inshape3dcore ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

if(WIN32)
  target_compile_definitions(simple_cgal PUBLIC _USE_MATH_DEFINES)
endif(WIN32)

target_include_directories(simple_cgal PUBLIC ${FC_APP_INCLUDE_DIRS})
