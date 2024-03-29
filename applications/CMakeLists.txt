
include(../cmake_scripts/ConfigureLibraries.cmake)

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
    ADD_SUBDIRECTORY(ode-app)
  endif(USE_ODE)

  ADD_SUBDIRECTORY(mathtest)
  ADD_SUBDIRECTORY(meshmeshtest)
  ADD_SUBDIRECTORY(multiblockgrid)
  ADD_SUBDIRECTORY(friction)
  ADD_SUBDIRECTORY(flagella)
  ADD_SUBDIRECTORY(fbmtest)
  ADD_SUBDIRECTORY(shapestest)
  ADD_SUBDIRECTORY(velocitybased)

  ADD_SUBDIRECTORY(pardynamics)
  ADD_SUBDIRECTORY(sequentialimpulses)
  ADD_SUBDIRECTORY(test0)
  ADD_SUBDIRECTORY(test1)  
  ADD_SUBDIRECTORY(trypanosome)  
  ADD_SUBDIRECTORY(soft_body2)  
  ADD_SUBDIRECTORY(taylor_line)
  ADD_SUBDIRECTORY(taylor_line2)
  #ADD_SUBDIRECTORY(ogre-app)

  if(USE_CGAL)
    if(${CGAL_FOUND} AND ${GMP_FOUND} AND ${MPFR_FOUND})

      # Applications that require OpenMesh
      if(USE_OPENMESH)

        #    find_package(Qt5 COMPONENTS Core Widgets OpenGL HINTS "C:/Qt/5.8/msvc2015_64/lib" REQUIRED)
        #    message(STATUS "Qt5: ${Qt5_FOUND}")
        #
        #    if(${Qt5_FOUND})
        #      message(STATUS "Qt5 found status : ${Qt5Widgets_FOUND}")
        #      #find_package(Qt5OpenGL 5.7 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")
        #      find_package(OpenGL REQUIRED)
        #      find_package(GLUT REQUIRED)
        #      ADD_SUBDIRECTORY(qt_opengl_test)
        #      ADD_SUBDIRECTORY(peristaltic_ogl)	
        #    endif(${Qt5_FOUND})

        #ADD_SUBDIRECTORY(position_based_dynamics)
        ADD_SUBDIRECTORY(interpolation)
        ADD_SUBDIRECTORY(openmeshtest2)
        ADD_SUBDIRECTORY(openmeshtest3)
        ADD_SUBDIRECTORY(openmeshtest4)
        ADD_SUBDIRECTORY(peristaltic)

      endif(USE_OPENMESH)
      ADD_SUBDIRECTORY(meshtest_cgal)    
      ADD_SUBDIRECTORY(meshtest_cgal2)    
      ADD_SUBDIRECTORY(cgal_tests)    
      ADD_SUBDIRECTORY(preprocessing_test)
      ADD_SUBDIRECTORY(boxmesher)
      ADD_SUBDIRECTORY(fbm_test2)
      ADD_SUBDIRECTORY(fbm_test3)
    endif(${CGAL_FOUND} AND ${GMP_FOUND} AND ${MPFR_FOUND})
  endif(USE_CGAL)


endif(FC_CUDA_SUPPORT)
