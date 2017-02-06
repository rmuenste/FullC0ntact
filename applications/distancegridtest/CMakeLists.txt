
# add a binary
add_executable(distancegridtest distancegridtest.cpp)

# add dependent libraries
target_link_libraries(distancegridtest Utility Math inshape3dcore)

# add include directories
target_include_directories (distancegridtest PUBLIC . ../../math ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)



