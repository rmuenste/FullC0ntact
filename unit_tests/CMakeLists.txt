set(ODE_DIR ${CMAKE_SOURCE_DIR}/libs/ode-cmake)
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
  ${CMAKE_SOURCE_DIR}/libs/eigen
  ${ODE_DIR}
  ${ODE_DIR}/include)
add_subdirectory(integration)
add_subdirectory(interpolation)
add_subdirectory(eigen_test)
add_subdirectory(eigen_solving)
