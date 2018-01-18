
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(q2p1prj2vtm multiblockgrid.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(q2p1prj2vtm multiblockgrid.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(q2p1prj2vtm ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(q2p1prj2vtm PRIVATE ${FC_APP_INCLUDE_DIRS})
