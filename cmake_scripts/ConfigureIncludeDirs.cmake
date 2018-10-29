# set the include directories
set(FC_INCLUDE_DIRS . 
  ${PROJECT_SOURCE_DIR}/math 
  ${PROJECT_SOURCE_DIR}/cuda_addon
  ${PROJECT_SOURCE_DIR}/util 
  ${PROJECT_SOURCE_DIR}/inshape3dcore 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/collision 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/distance 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/grid 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/fortrancppinterface 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/intersection 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/physics 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/postprocessing 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/preprocessing 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/shapes 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/shapes/modelsandmeshes 
  ${PROJECT_SOURCE_DIR}/libs/rapidxml-1.13
  ${PROJECT_SOURCE_DIR}/libs/eigen
  ${ODE_DIR}
  ${ODE_DIR}/include

  ${CUDA_SDK_ROOT_DIR}/common/inc 
  ${CUDA_SDK_ROOT_DIR}/shared/inc 
  ${CUDA_PATH}/include 
  ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install-dir/include
)

# add include directories
set(FC_APP_INCLUDE_DIRS . 
  ${PROJECT_SOURCE_DIR}/math 
  ${PROJECT_SOURCE_DIR}/cuda_addon
  ${PROJECT_SOURCE_DIR}/util 
  ${PROJECT_SOURCE_DIR}/inshape3dcore 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/collision 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/distance 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/grid 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/fortrancppinterface 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/intersection 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/physics 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/postprocessing 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/preprocessing 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/shapes 
  ${PROJECT_SOURCE_DIR}/inshape3dcore/shapes/modelsandmeshes 
  ${PROJECT_SOURCE_DIR}/libs/rapidxml-1.13
  ${PROJECT_SOURCE_DIR}/libs/eigen
  ${ODE_DIR}
  ${ODE_DIR}/include
  ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install-dir/include
  )

if(USE_EIGEN)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/libs/eigen
  )

  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/libs/eigen
  )
endif(USE_EIGEN)

if(USE_OPTICALTWEEZERS)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/vector
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/strahl	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/ot	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/tinyxml	
  )

  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/vector
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/strahl	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/ot	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/tinyxml	
  )
endif()

if(USE_ODE)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} 
      ${ODE_DIR}
      ${ODE_DIR}/include
     )

  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} 
      ${ODE_DIR}
      ${ODE_DIR}/include
     )
endif(USE_ODE)

if(USE_CGAL)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install/include)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})


  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install/include)
  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  set(FC_APP_INCLUDE_DIRS ${FC_APP_INCLUDE_DIRS} ${CGAL_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})
endif(USE_CGAL)

