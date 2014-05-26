#________________________________________________________________
#
# Inshape3d Library : velocitybased folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes  ${CUDA_SDK_ROOT_DIR}/common/inc ${CUDA_SDK_ROOT_DIR}/shared/inc)

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




