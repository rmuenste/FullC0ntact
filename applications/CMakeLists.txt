#________________________________________________________________
#
# Inshape3d Library : applications folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#


if(FC_CUDA_SUPPORT)
  ADD_SUBDIRECTORY(gpu_test)
  ADD_SUBDIRECTORY(meshmeshtest_gpu)
  ADD_SUBDIRECTORY(particles_gpu)
  ADD_SUBDIRECTORY(distmap_test)
  ADD_SUBDIRECTORY(distmap_test2)
  ADD_SUBDIRECTORY(innersphere)
  ADD_SUBDIRECTORY(innerspheretree)
  ADD_SUBDIRECTORY(hashgridtest)
  ADD_SUBDIRECTORY(gridgeneration)
  ADD_SUBDIRECTORY(velocitybased)
  ADD_SUBDIRECTORY(openmeshtest)
  ADD_SUBDIRECTORY(openmeshtest2)
  ADD_SUBDIRECTORY(fbmtest)
else(FC_CUDA_SUPPORT)
  ADD_SUBDIRECTORY(boxes_sticks)
  ADD_SUBDIRECTORY(distancegridtest)
  ADD_SUBDIRECTORY(duckpond)  
  ADD_SUBDIRECTORY(dem_sim)  
  ADD_SUBDIRECTORY(dem_sim_test0)  
  ADD_SUBDIRECTORY(dem_sim_test1)    
  ADD_SUBDIRECTORY(dem_sim_test2)      
  ADD_SUBDIRECTORY(dem_sim_test3)      
  ADD_SUBDIRECTORY(gjktest)
  ADD_SUBDIRECTORY(inclinedplane)
  ADD_SUBDIRECTORY(mathtest)
  ADD_SUBDIRECTORY(meshmeshtest)
  ADD_SUBDIRECTORY(friction)
  ADD_SUBDIRECTORY(fbmtest)
  ADD_SUBDIRECTORY(shapestest)
  ADD_SUBDIRECTORY(velocitybased)
  ADD_SUBDIRECTORY(penetrationcorrection)
  ADD_SUBDIRECTORY(pardynamics)
  ADD_SUBDIRECTORY(sequentialimpulses)
  ADD_SUBDIRECTORY(test0)
  ADD_SUBDIRECTORY(test1)  
endif(FC_CUDA_SUPPORT)





