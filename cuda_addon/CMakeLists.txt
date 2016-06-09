# add include directories
include_directories (. ../math ../util ../inshape3dcore ../inshape3dcore/collision ../inshape3dcore/distance ../inshape3dcore/grid 
../inshape3dcore/intersection ../inshape3dcore/physics ../inshape3dcore/postprocessing ../inshape3dcore/preprocessing ../inshape3dcore/shapes ../inshape3dcore/shapes/modelsandmeshes)

set(difi_src
auxiliary_functions.cuh
common.h
difi.cu
particledem.cu
particledem_um.cu
particledem_um.cuh
ext_unit_tests.cu
difi.cuh
distance.cuh
intersection.cuh
unit_tests.cuh
uniformgrid.cuh
bvh.cuh
particleworld.hpp
hashgrid.hpp
managed.hpp
)

source_group(main FILES ${difi_src})

# add a library
CUDA_ADD_LIBRARY(difi ${difi_src})

if(WIN32)
  target_link_libraries(difi Utility Math inshape3dcore)
else(WIN32)
  target_link_libraries(difi Utility Math inshape3dcore)
endif(WIN32)
