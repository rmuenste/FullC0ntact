#________________________________________________________________
#
# Inshape3d Library : test1 folder CMakeLists.txt file
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
add_executable(test1 test1.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(test1 Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(test1 Utility Math inshape3dcore)
endif(WIN32)





