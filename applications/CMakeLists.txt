#________________________________________________________________
#
# Inshape3d Library : applications folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#


if(FC_CUDA_SUPPORT)
  ADD_SUBDIRECTORY(gpuparticles)
else(FC_CUDA_SUPPORT)
  ADD_SUBDIRECTORY(distancegridtest)
  ADD_SUBDIRECTORY(duckpond)  
  ADD_SUBDIRECTORY(gjktest)
  ADD_SUBDIRECTORY(inclinedplane)
  ADD_SUBDIRECTORY(mathtest)
  ADD_SUBDIRECTORY(friction)
  ADD_SUBDIRECTORY(fbmtest)
  ADD_SUBDIRECTORY(meshmeshtest)
  ADD_SUBDIRECTORY(gridgeneration)
  ADD_SUBDIRECTORY(shapestest)
  ADD_SUBDIRECTORY(velocitybased)
  ADD_SUBDIRECTORY(penetrationcorrection)
  ADD_SUBDIRECTORY(pardynamics)
  ADD_SUBDIRECTORY(sequentialimpulses)
  ADD_SUBDIRECTORY(test0)
endif(FC_CUDA_SUPPORT)





