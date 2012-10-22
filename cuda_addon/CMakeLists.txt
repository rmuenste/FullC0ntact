
set(particles_src
particleSystem.cpp
particles_kernel.cu
particleSystem.cu
particleSystem.h
particles_kernel.cuh
particleSystem.cuh
)

set(main_src
src/particles/particles.cpp
src/particles/render_particles.cpp
src/particles/shaders.cpp
src/particles/paramgl.cpp
src/particles/rendercheck_gl.cpp
src/particles/render_particles.h
src/particles/shaders.h
src/particles/paramgl.h
src/particles/rendercheck_gl.h
)

#${SHRUTIL_LIBRARY}

link_libraries(${FREEGLUT_LIBRARY} ${GLEW_LIBRARY} ${CUTIL_LIBRARY})

include_directories (. ${CUDA_SDK_ROOT_DIR}/common/inc ${SHRUTIL_INCLUDE_DIR})

# add a library
CUDA_ADD_LIBRARY(particles_kernel ${particles_src})

# link libraries
#target_link_libraries(particles_main particles_kernel)

