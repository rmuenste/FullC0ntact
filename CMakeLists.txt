# cmake version
cmake_minimum_required(VERSION 3.2)
CMAKE_POLICY(SET CMP0054 NEW)

# name of the project
PROJECT(FullC0ntact)

# set module directory
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake_scripts)

# allow easier if-else syntax
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

# enable testing
ENABLE_TESTING()

# output the system name
MESSAGE(STATUS "Configuring FullC0ntact for a ${CMAKE_SYSTEM} system")

SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

MESSAGE(STATUS "module path ${CMAKE_MODULE_PATH}")

#------------------------------------------------------------------------------------------------------------------
#                                               compiler settings
#------------------------------------------------------------------------------------------------------------------

option(FC_SHARED_LIBS
  "Build shared libraries"
  OFF
  )

IF(FC_SHARED_LIBS)
  SET(BUILD_SHARED_LIBS ON)
ENDIF(FC_SHARED_LIBS)

if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
  message(STATUS "Checking build type...")
  include(./cmake_scripts/GenerateBuildIds.cmake)
endif()

#------------------------------------------------------------------------------------------------------------------
#                                               CMake and Preprocessor options
#------------------------------------------------------------------------------------------------------------------

# set the default build type to featflowlib ()
option(FC_FEATFLOW
  "Build the FullC0ntact library for use with the FEATFLOW simulator"
  OFF
  )

option(USE_OPENMESH
  "Use the OpenMesh library"
  OFF
  )

option(USE_CGAL
  "Use the cgal library"
  OFF
  )

option(USE_OPTICALTWEEZERS
  "Use the opticaltweezers library"
  OFF
  )

option(USE_ODE
  "Use the ODE library"
  OFF
  )

option(BUILD_FC_UNIT_TESTS
  "Build the unit tests of the FC library"
  OFF
  )

IF(FC_FEATFLOW)
  add_definitions(-DFEATFLOWLIB)
ENDIF(FC_FEATFLOW)

option(FC_SILENT
  "Suppress output for some applications"
  OFF
  )

IF(FC_SILENT)
  add_definitions(-DFC_SILENT)
ENDIF(FC_SILENT)

option(FC_CUDA_SUPPORT
  "Enables use of CUDA extensions"
  OFF
  )

IF(FC_CUDA_SUPPORT)

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
ENDIF(FC_CUDA_SUPPORT)

#-----------------------------------------------------------------------------------------------------------------
#                                               project directories
#-----------------------------------------------------------------------------------------------------------------

if(USE_OPENMESH)
  if(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenMesh")
    ADD_SUBDIRECTORY(libs/OpenMesh)
  endif(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenMesh")
endif()

if(USE_ODE)
  if(EXISTS "${CMAKE_SOURCE_DIR}/libs/ode-cmake")
    ADD_SUBDIRECTORY(libs/ode-cmake)
  endif(EXISTS "${CMAKE_SOURCE_DIR}/libs/ode-cmake")
endif()

#-------------------------------------------------------------------------------------------------
#                               Configure CGAL 
#-------------------------------------------------------------------------------------------------

if(USE_CGAL)
  find_package(CGAL)
  if(${CGAL_FOUND})
    find_package(GMP)
    find_package(MPFR)

    if(NOT ${GMP_FOUND})
      message(WARNING "GMP library not found. Cannot build cgal applications.")
    endif()

    if(NOT ${MPFR_FOUND})
      message(WARNING "MPFR library not found. Cannot build cgal applications.")
    endif()

  endif()	
endif(USE_CGAL)

#-------------------------------------------------------------------------------------------------
#                               Configure OT Library 
#-------------------------------------------------------------------------------------------------

if(USE_OPTICALTWEEZERS)
  add_subdirectory(extern/libraries/opticaltweezers)
endif(USE_OPTICALTWEEZERS)

#-------------------------------------------------------------------------------------------------
#                               Configure BoostC++ 
#-------------------------------------------------------------------------------------------------
# find_package(Boost 1.56 REQUIRED COMPONENTS thread): 
# find_package(Boost [version] [EXACT] [REQUIRED] [COMPONENTS <libs> ...]) 
# Here [version] denotes the minimum version of the library
# In case the libraries are found the following variables are set:
# 
# Boost_INCLUDE_DIRS : - Boost include directories
# Boost_LIBRARY_DIRS : - Boost library directories
# Boost_LIBRARIES    : - Boost libraries to be linked
# Boost version vars : - Boost detailed library version: ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}
if(USE_CGAL)
  find_package(Boost 1.56 REQUIRED COMPONENTS thread)
endif(USE_CGAL)

#if(Boost_FOUND)
#  message("Boost library version: ${Boost_VERSION}")
#  message("Boost detailed library version: ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
#  message("Boost include directory: ${Boost_INCLUDE_DIRS}")
#  message("Boost library directory: ${Boost_LIBRARY_DIRS}")
#  message("Boost library directory: ${Boost_LIBRARIES}")
#  message(FATAL_ERROR "Found Boost library")
#else()
#  message(FATAL_ERROR "Did not find Boost library")
#endif()

# add the core subdirectories
if(FC_CUDA_SUPPORT)
  ADD_SUBDIRECTORY(cuda_addon)
endif(FC_CUDA_SUPPORT)
ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(util)
ADD_SUBDIRECTORY(inshape3dcore)

# check whether the applications should be build
if(NOT FC_FEATFLOW)
  ADD_SUBDIRECTORY(applications)

  MESSAGE(STATUS "Copying data directories...")

  if(FC_CUDA_SUPPORT)
    SET(APP_DIRS gpu_test distmap_test2 meshmeshtest openmeshtest openmeshtest2 openmeshtest3 openmeshtest4 interpolation meshmeshtest_gpu innersphere hashgridtest innerspheretree particles_gpu gridgeneration peristaltic velocitybased )
  else(FC_CUDA_SUPPORT)
    SET(APP_DIRS duckpond dem_sim dem_sim_test0 dem_sim_test1 dem_sim_test2 dem_sim_test3 flagella gjktest meshtest meshmeshtest gridgeneration velocitybased friction dynamics 
      distancegridtest inclinedplane mathtest shapestest sequentialimpulses penetrationcorrection test0 test1 openmeshtest openmeshtest2 openmeshtest3 openmeshtest4 interpolation 
      qt_opengl_test trypanosome soft_body2 ecoli multiblockgrid taylor_line peristaltic peristaltic_ogl ode-test ode-app meshtest_cgal meshtest_cgal2 preprocessing_test) 
  endif(FC_CUDA_SUPPORT)

  #--------------------------------------------------------------------------------------------------------------------
  #                                               prepare application
  #--------------------------------------------------------------------------------------------------------------------

  foreach(app ${APP_DIRS})
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/applications/${app}/meshes ${CMAKE_BINARY_DIR}/applications/${app}/meshes)

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/solution)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/solution)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/output)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/output)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/start)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start/data.TXT AND EXISTS ${CMAKE_SOURCE_DIR}/applications/${app}/start/data.default)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/${app}/start/data.default
        ${CMAKE_BINARY_DIR}/applications/${app}/start/data.TXT)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start/sampleRigidBody.xml AND EXISTS ${CMAKE_SOURCE_DIR}/applications/${app}/start/sampleRigidBody.xml)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/${app}/start/sampleRigidBody.xml
        ${CMAKE_BINARY_DIR}/applications/${app}/start/sampleRigidBody.xml)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start/rigidbody.xml AND EXISTS ${CMAKE_SOURCE_DIR}/applications/${app}/start/rigidbody.xml)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/${app}/start/rigidbody.xml
        ${CMAKE_BINARY_DIR}/applications/${app}/start/rigidbody.xml)
    endif ()

  endforeach(app)

  execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/dem_sim/meshes/particles.i3d.dem_sim
    ${CMAKE_BINARY_DIR}/applications/dem_sim/solution/particles.i3d.dem_sim)

  execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/dem_sim_test0/meshes/particles.i3d.dem_t0
    ${CMAKE_BINARY_DIR}/applications/dem_sim/solution/particles.i3d.dem_t0)  

endif(NOT FC_FEATFLOW)
#--------------------------------------------------------------------------------------------------------------------
#                                               Add the unit tests
#--------------------------------------------------------------------------------------------------------------------

if(BUILD_FC_UNIT_TESTS)
  add_subdirectory(unit_tests)
endif(BUILD_FC_UNIT_TESTS)

#--------------------------------------------------------------------------------------------------------------------
#                                               generate documentation
#--------------------------------------------------------------------------------------------------------------------
# add a target to generate API documentation with Doxygen
#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#  add_custom_target(doc
#    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#    COMMENT "Generating API documentation with Doxygen" VERBATIM
#    )
#endif(DOXYGEN_FOUND)

#--------------------------------------------------------------------------------------------------------------------
#                                                    add tests
#--------------------------------------------------------------------------------------------------------------------
ADD_TEST(NAME VelocityBased WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/velocitybased COMMAND velocitybased)
ADD_TEST(NAME ShapesTest WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/shapestest COMMAND shapestest)
ADD_TEST(NAME SequentialImpulses WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/sequentialimpulses COMMAND sequentialimpulses)
ADD_TEST(NAME MeshMesh WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/meshmeshtest COMMAND meshmeshtest)

