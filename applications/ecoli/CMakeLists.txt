
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(ecoli ecoli.cpp)

target_link_libraries(ecoli ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(ecoli PUBLIC ${FC_APP_INCLUDE_DIRS})

#message(STATUS "includes: ${FC_APP_INCLUDE_DIRS}")

# add include directories
#target_include_directories (ecoli PUBLIC . ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
#../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
#../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

