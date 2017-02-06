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
  ADD_SUBDIRECTORY(openmeshtest3)
  ADD_SUBDIRECTORY(openmeshtest4)
  ADD_SUBDIRECTORY(interpolation)
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
  ADD_SUBDIRECTORY(ecoli)  
  ADD_SUBDIRECTORY(gjktest)
  ADD_SUBDIRECTORY(inclinedplane)
  ADD_SUBDIRECTORY(mathtest)
  ADD_SUBDIRECTORY(meshmeshtest)
  ADD_SUBDIRECTORY(multiblockgrid)
  ADD_SUBDIRECTORY(friction)
  ADD_SUBDIRECTORY(flagella)
  ADD_SUBDIRECTORY(fbmtest)
  ADD_SUBDIRECTORY(shapestest)
  ADD_SUBDIRECTORY(velocitybased)
  ADD_SUBDIRECTORY(penetrationcorrection)
  ADD_SUBDIRECTORY(pardynamics)
  ADD_SUBDIRECTORY(sequentialimpulses)
  ADD_SUBDIRECTORY(test0)
  ADD_SUBDIRECTORY(test1)  
  ADD_SUBDIRECTORY(trypanosome)  
  ADD_SUBDIRECTORY(soft_body2)  
  ADD_SUBDIRECTORY(openmeshtest2)
  ADD_SUBDIRECTORY(openmeshtest3)
  ADD_SUBDIRECTORY(openmeshtest4)
  ADD_SUBDIRECTORY(interpolation)

  find_package(Qt5Widgets 5.7 HINTS "/opt/Qt/5.7/gcc_64/lib")

  if(${Qt5Widgets_FOUND})
    message(STATUS "Qt5 found status : ${Qt5Widgets_FOUND}")
    find_package(Qt5OpenGL 5.7 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    ADD_SUBDIRECTORY(qt_opengl_test)
  endif(${Qt5Widgets_FOUND})


endif(FC_CUDA_SUPPORT)




# add include directories
set(FC_APP_INCLUDE_DIRS . 
  ${CMAKE_SOURCE_DIR}/math 
  ${CMAKE_SOURCE_DIR}/cuda_addon
  ${CMAKE_SOURCE_DIR}/util 
  ${CMAKE_SOURCE_DIR}/inshape3dcore 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/collision 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/distance 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/grid 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/fortrancppinterface 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/intersection 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/physics 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/postprocessing 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/preprocessing 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/shapes 
  ${CMAKE_SOURCE_DIR}/inshape3dcore/shapes/modelsandmeshes 
  ${CMAKE_SOURCE_DIR}/libs/rapidxml-1.13)

