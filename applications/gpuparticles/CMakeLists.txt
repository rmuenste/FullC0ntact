
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
cuda_add_executable(gpuparticles gpuparticles.cpp)

# add dependent libraries
if(WIN32)
  target_link_libraries(gpuparticles Utility Math inshape3dcore ${OPENGL_LIBRARIES} ${FREEGLUT_LIBRARY} particles_kernel)
else(WIN32)
  target_link_libraries(gpuparticles Utility Math inshape3dcore ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} particles_kernel)
endif(WIN32)

# add include directories
target_include_directories(gpuparticles PUBLIC ${FC_APP_INCLUDE_DIRS})


