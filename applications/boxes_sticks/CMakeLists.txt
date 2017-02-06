
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(boxes_sticks boxes_sticks.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(boxes_sticks Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(boxes_sticks Utility Math inshape3dcore)
endif(WIN32)

# add include directories
target_include_directories (boxes_sticks PUBLIC . ../../math ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)





