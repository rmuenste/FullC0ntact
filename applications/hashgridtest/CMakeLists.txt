
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(hashgridtest hashgridtest.cpp)

  if(WIN32)
    # add dependent libraries
    target_link_libraries(hashgridtest Utility Math inshape3dcore difi)
  else(WIN32)
    # add dependent libraries
    target_link_libraries(hashgridtest Utility Math inshape3dcore difi)
  endif(WIN32)

endif(FC_CUDA_SUPPORT)

# add include directories
target_include_directories(hashgridtest PUBLIC ${FC_APP_INCLUDE_DIRS})

