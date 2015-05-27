set(difi_src
common.h
difi.cu
difi.cuh
)

source_group(main FILES ${difi_src})

# add a library
CUDA_ADD_LIBRARY(difi ${difi_src})


