#________________________________________________________________
#
# Inshape3d Library : velocitybased folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(soft_body2 soft_body2.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(soft_body2 Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(soft_body2 Utility Math inshape3dcore)
endif(WIN32)





