#________________________________________________________________
#
# Inshape3d Library : Top-Level CMakeLists file
# Author            : Raphael Muenster
#________________________________________________________________
#
# cmake version
cmake_minimum_required(VERSION 2.8)

# name of the project
PROJECT(FullC0ntact)

# set module directory
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_scripts/modules")

# allow easier if-else syntax
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

# enable testing
ENABLE_TESTING()

# output the system name
MESSAGE(STATUS "Configuring Inshape3d for a ${CMAKE_SYSTEM} system")

#--------------------------------------------------------------------------------------------------------------------
#                                               compiler settings
#--------------------------------------------------------------------------------------------------------------------

message(STATUS "Checking build type...")

option(FC_SHARED_LIBS
    "Build shared libraries"
    OFF
)

IF(FC_SHARED_LIBS)
 SET(BUILD_SHARED_LIBS ON)
ENDIF(FC_SHARED_LIBS)

IF(NOT FC_BUILD_ID)
  # A build id is composed of:
  # ${cpu_type}-${os}-${compiler}-${build_type}
  # Example: nehalem-linux-intel-release

  # We do not have a user defined build id
  # so we select a default build

  message(STATUS "No build id selected... trying to determine machine type and compiler settings")

  # which compiler is found
  message(STATUS "C++ Compiler ${CMAKE_CXX_COMPILER_ID}")

  message(STATUS "Processor ${CMAKE_SYSTEM_PROCESSOR}")

  set(_vendor_id)
  set(_cpu_family)
  set(_cpu_model)
  set(_cpu_flags)

  set(FC_CPU_TYPE)
  set(FC_OS)
  set(FC_DEFAULT_BUILD)
  set(FC_COMPILER_ID)

  # set the compiler
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(FC_COMPILER_ID "gcc")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    set(FC_COMPILER_ID "intel")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(FC_COMPILER_ID "clang")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  ELSE()
    message(FATAL_ERROR "Compiler :<${CMAKE_CXX_COMPILER_ID}> is not tested with this library and hence not supported.")  
  ENDIF()

  IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(FC_DEFAULT_BUILD "release")
  ELSE(NOT CMAKE_BUILD_TYPE)
    string(TOLOWER "${CMAKE_BUILD_TYPE}" FC_DEFAULT_BUILD)
  ENDIF(NOT CMAKE_BUILD_TYPE)

  # For a windows system cmake will generate visual studio files
  # with all possible build types and we are done. For a linux system
  # we want to select compiler settings that are useful for the hardware
  # we are using  
  IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(FC_OS "linux")
    file(READ "/proc/cpuinfo" _cpuinfo)
    string(REGEX REPLACE ".*vendor_id[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _vendor_id "${_cpuinfo}")
    string(REGEX REPLACE ".*cpu family[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_family "${_cpuinfo}")
    string(REGEX REPLACE ".*model[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_model "${_cpuinfo}")
    string(REGEX REPLACE ".*flags[ \t]*:[ \t]+([ \ta-zA-Z0-9_-]+)(.*)" "\\1" _cpu_flags "${_cpuinfo}")

    message(STATUS "vendor_id ${_vendor_id}")
    message(STATUS "family ${_cpu_family}")
    message(STATUS "model ${_cpu_model}")
    message(STATUS "flags ${_cpu_flags}")
  
    IF(${_vendor_id} MATCHES "AuthenticAMD")
      IF(${_cpu_family} EQUAL 16)
        IF(${_cpu_model} EQUAL 4)
          SET(FC_CPU_TYPE "phenomIIx4")
        ENDIF()
      ELSEIF(${_cpu_family} EQUAL 15)
        IF(${_cpu_model} EQUAL 33)
          SET(FC_CPU_TYPE "opteron")
        ELSEIF(${_cpu_model} EQUAL 65 OR ${_cpu_model} EQUAL 66 )
          SET(FC_CPU_TYPE "opteronx2")
        ENDIF()
      ELSE()
        message(FATAL_ERROR "Unknown CPU, cannot set up default configuration")
      ENDIF()
    ELSEIF(${_vendor_id} MATCHES "GenuineIntel")
      IF(${_cpu_family} EQUAL 6)
        IF("15 21 22 23 29" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "core2duo")
        ELSEIF("26 30 37 44 42 46" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "nehalem")
        ENDIF()
      ENDIF()
    ELSE()
      message(FATAL_ERROR "CPU could not be identified.")
    ENDIF()
  
    SET(FC_BUILD_ID "${FC_CPU_TYPE}-${FC_OS}-${FC_COMPILER_ID}-${FC_DEFAULT_BUILD}")
  
    include(${CMAKE_SOURCE_DIR}/cmake_scripts/buildids.cmake)
    IF(NOT FC_BUILD_ID_FOUND)
      message(FATAL_ERROR "Build id:<${FC_BUILD_ID}> was not found.")
    ENDIF(NOT FC_BUILD_ID_FOUND)
    message(STATUS "Configuring for build id:<${FC_BUILD_ID}>")
  
  endif(CMAKE_SYSTEM_NAME MATCHES "Linux")  
ELSE(NOT FC_BUILD_ID)

  include(${CMAKE_SOURCE_DIR}/cmake_scripts/buildids.cmake)
  IF(NOT FC_BUILD_ID_FOUND)
    message(FATAL_ERROR "Build id:<${FC_BUILD_ID}> was not found.")
  ENDIF(NOT FC_BUILD_ID_FOUND)
  message(STATUS "Configuring for build id:<${FC_BUILD_ID}>")

ENDIF(NOT FC_BUILD_ID)


#--------------------------------------------------------------------------------------------------------------------
#                                               CMake and Preprocessor options
#--------------------------------------------------------------------------------------------------------------------

add_definitions(-std=c++0x)

# set the default build type to featflowlib ()
option(FC_FEATFLOW
    "Build the FullC0ntact library for use with the FEATFLOW simulator"
    OFF
)

IF(FC_FEATFLOW)
  add_definitions(-DFEATFLOWLIB)
ENDIF(FC_FEATFLOW)

option(FC_MPI_SUPPORT
    "Enables MPI extensions"
    OFF
)

IF(FC_MPI_SUPPORT)
  add_definitions(-DFC_MPI_SUPPORT)
ENDIF(FC_MPI_SUPPORT)

option(FC_OPENMP_SUPPORT
    "Enables OpenMP extensions"
    OFF
)

option(FC_SILENT
    "Suppress output for some applications"
    OFF
)

IF(FC_SILENT)
  add_definitions(-DFC_SILENT)
ENDIF(FC_SILENT)

IF(FC_OPENMP_SUPPORT)
  FIND_PACKAGE( OpenMP )
  IF(OPENMP_FOUND)
    add_definitions(-DFC_OPENMP)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  ELSEIF(OPENMP_FOUND)
    message(FATAL_ERROR "OpenMP was requested, but the OpenMP library was not found on the system.")
  ENDIF(OPENMP_FOUND)
ENDIF(FC_OPENMP_SUPPORT)

option(FC_CUDA_SUPPORT
    "Enables use of CUDA extensions"
    OFF
)

IF(FC_CUDA_SUPPORT)

  add_definitions(-DFC_CUDA_SUPPORT)

  FIND_PACKAGE(OpenGL REQUIRED)

  FIND_PACKAGE( CUDA )
  IF(CUDA_FOUND)
    message(STATUS "Found CUDA")
  ELSEIF(CUDA_FOUND)
    message(FATAL_ERROR "The CUDA library was not found on the system.")
  ENDIF(CUDA_FOUND)

  # search for cuda libraries
  FIND_PACKAGE(CUDASDK)
ENDIF(FC_CUDA_SUPPORT)


#--------------------------------------------------------------------------------------------------------------------
#                                               project directories
#--------------------------------------------------------------------------------------------------------------------
IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
include_directories (math util inshape3dcore inshape3dcore/collision inshape3dcore/distance inshape3dcore/grid 
inshape3dcore/fortrancppinterface inshape3dcore/intersection inshape3dcore/physics inshape3dcore/postprocessing
inshape3dcore/preprocessing inshape3dcore/shapes inshape3dcore/shapes/modelsandmeshes)
ENDIF(CMAKE_SYSTEM_NAME MATCHES "Linux")

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
    SET(APP_DIRS gpuparticles)
  else(FC_CUDA_SUPPORT)
    SET(APP_DIRS duckpond dem_sim dem_sim_test0 dem_sim_test1 dem_sim_test2 gjktest meshtest meshmeshtest gridgeneration velocitybased friction dynamics distancegridtest inclinedplane mathtest shapestest sequentialimpulses penetrationcorrection test0 test1)  
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
#                                               generate documentation
#--------------------------------------------------------------------------------------------------------------------
# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

#--------------------------------------------------------------------------------------------------------------------
#                                                    add tests
#--------------------------------------------------------------------------------------------------------------------
ADD_TEST(NAME VelocityBased WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/velocitybased COMMAND velocitybased)
ADD_TEST(NAME ShapesTest WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/shapestest COMMAND shapestest)
ADD_TEST(NAME SequentialImpulses WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/sequentialimpulses COMMAND sequentialimpulses)
ADD_TEST(NAME MeshMesh WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/meshmeshtest COMMAND meshmeshtest)



