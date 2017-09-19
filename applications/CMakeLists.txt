set(ODE_DIR ${CMAKE_SOURCE_DIR}/libs/ode-0.13-cmake)
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
  ${CMAKE_SOURCE_DIR}/libs/rapidxml-1.13
  ${ODE_DIR}
  ${ODE_DIR}/include)

if(USE_ODE)
  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} ${ODE_DIR} ${ODE_DIR}/include)
endif(USE_ODE)


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
  ADD_SUBDIRECTORY(gridgeneration)
  ADD_SUBDIRECTORY(inclinedplane)

  if(USE_ODE)
    ADD_SUBDIRECTORY(ode-test)
  endif(USE_ODE)

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
  ADD_SUBDIRECTORY(taylor_line)
   
if(USE_CGAL)
  find_package(CGAL)
  if(${CGAL_FOUND})
    find_package(GMP)
    find_package(MPFR)
    if(${GMP_FOUND} AND ${MPFR_FOUND})
      ADD_SUBDIRECTORY(meshtest_cgal)    
      ADD_SUBDIRECTORY(cgal_tests)    
    else()
      
      if(NOT ${GMP_FOUND})
        message(WARNING "GMP library not found. Cannot build meshtest_cgal application.")
      endif()

      if(NOT ${MPFR_FOUND})
        message(WARNING "MPFR library not found. Cannot build meshtest_cgal application.")
      endif()

    endif()
  endif()	
endif(USE_CGAL)

  # Applications that require OpenMesh
  if(USE_OPENMESH)

    find_package(Qt5 COMPONENTS Core Widgets OpenGL HINTS "C:/Qt/5.8/msvc2015_64/lib" REQUIRED)
    message(STATUS "Qt5: ${Qt5_FOUND}")
    
    if(${Qt5_FOUND})
      message(STATUS "Qt5 found status : ${Qt5Widgets_FOUND}")
      #find_package(Qt5OpenGL 5.7 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")
      find_package(OpenGL REQUIRED)
      find_package(GLUT REQUIRED)
      ADD_SUBDIRECTORY(qt_opengl_test)
      ADD_SUBDIRECTORY(peristaltic_ogl)	
    endif(${Qt5_FOUND})

    ADD_SUBDIRECTORY(interpolation)
    ADD_SUBDIRECTORY(openmeshtest2)
    ADD_SUBDIRECTORY(openmeshtest3)
    ADD_SUBDIRECTORY(openmeshtest4)
    ADD_SUBDIRECTORY(peristaltic)

  endif(USE_OPENMESH)


endif(FC_CUDA_SUPPORT)

