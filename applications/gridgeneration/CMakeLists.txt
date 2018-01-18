
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(gridgeneration gridgeneration.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(gridgeneration gridgeneration.cpp)
endif(NOT FC_CUDA_SUPPORT)

target_link_libraries(gridgeneration ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(gridgeneration PRIVATE ${FC_APP_INCLUDE_DIRS})
