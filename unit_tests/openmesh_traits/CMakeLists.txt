# add a binary
add_executable(meshtraits meshtraits.cpp)

# add dependent libraries
target_link_libraries(meshtraits OpenMeshCore OpenMeshTools Utility Math inshape3dcore)

if(USE_CGAL)
  target_link_libraries(meshtraits ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES}) 
endif(USE_CGAL)

# add include directories
target_include_directories(meshtraits PUBLIC ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libs/OpenMesh/src)



