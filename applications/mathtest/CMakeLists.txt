
# add a binary
add_executable(mathtest mathtest.cpp)

set(APP_LIBS_RELEASE Utility Math inshape3dcore ${CGAL_LIBRARIES_RELEASE} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
set(APP_LIBS_DEBUG Utility Math inshape3dcore ${CGAL_LIBRARIES_DEBUG} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

# add dependent libraries
target_link_libraries(mathtest 
  optimized ${APP_LIBS_RELEASE}
  debug ${APP_LIBS_DEBUG}
  )

#set(FC_APP_LIB_DIRS ${FC_APP_LIB_DIRS} ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES}) 

# add include directories
target_include_directories(mathtest PUBLIC ${FC_APP_INCLUDE_DIRS})



