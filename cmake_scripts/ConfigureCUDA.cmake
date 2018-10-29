
  add_definitions(-DFC_CUDA_SUPPORT)

  message(STATUS "Before CUDA CMake CMake_C_COMPILER=${CMAKE_C_COMPILER}")

  FIND_PACKAGE( CUDA )
  IF(CUDA_FOUND)
    message(STATUS "Found CUDA")
    SET(CUDA_HOST_COMPILER "${CMAKE_C_COMPILER}")

    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
      ${CMAKE_BINARY_DIR} 
      ${CMAKE_SOURCE_DIR}/cmake_scripts/test_programs/has_cuda_gpu.cpp
      CMAKE_FLAGS 
      -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
      -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
      COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
      RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    message("${RUN_OUTPUT_VAR}")
    # Display number of GPUs found
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
      set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
    else()
      set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()  

    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
      ${CMAKE_BINARY_DIR} 
      ${CMAKE_SOURCE_DIR}/cmake_scripts/test_programs/cuda_compute_capability.cpp
      CMAKE_FLAGS 
      -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
      -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
      COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
      RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
    # message("${RUN_OUTPUT_VAR}") # Display compute capability   
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
      set(CUDA_COMPUTE_CAPABILITY ${RUN_OUTPUT_VAR} CACHE STRING "Compute capability of CUDA-capable GPU present")
      message(STATUS "CMake_C_COMPILER=${CMAKE_C_COMPILER}")
      message(STATUS "CUDA_HOST_COMPILER=${CUDA_HOST_COMPILER}")
      message("${RUN_OUTPUT_VAR}")
      if(CMAKE_BUILD_TYPE STREQUAL "Release")
        IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
          set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY} ")
        ELSE("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
          set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY}")
        ENDIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
      elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")  
        IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
          set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-G -gencode arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY}")  
        ELSE("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
          set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-G -gencode arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY}")  
        ENDIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
      endif(CMAKE_BUILD_TYPE STREQUAL "Release")
    else()
      set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()	

  ELSEIF(CUDA_FOUND)
    message(FATAL_ERROR "The CUDA library was not found on the system.")
  ENDIF(CUDA_FOUND)

  # search for cuda libraries
  # FIND_PACKAGE(CUDASDK)