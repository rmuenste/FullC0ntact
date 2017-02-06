
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(meshmeshtest_gpu meshmeshtest_gpu.cpp)

  if(WIN32)
    # add dependent libraries
    target_link_libraries(meshmeshtest_gpu Utility Math inshape3dcore difi)
  else(WIN32)
    # add dependent libraries
    target_link_libraries(meshmeshtest_gpu Utility Math inshape3dcore difi)
  endif(WIN32)

endif(FC_CUDA_SUPPORT)


# add include directories
target_include_directories(meshmeshtest_gpu PUBLIC ${FC_APP_INCLUDE_DIRS})


