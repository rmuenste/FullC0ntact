
# set module directory
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_scripts/modules")

# enable testing
ENABLE_TESTING()

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

# output the system name
MESSAGE(STATUS "Configuring FullC0ntact for a ${CMAKE_SYSTEM} system")

SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

MESSAGE(STATUS "module path ${CMAKE_MODULE_PATH}")

#==================================================================================================
#                             Architecture and Compiler Configuration
#==================================================================================================

if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
  message(STATUS "Checking build type...")
  include(./cmake_scripts/GenerateBuildIds.cmake)
endif()

#==================================================================================================
#                                      CMake options
#==================================================================================================
option(FC_SHARED_LIBS
  "Build shared libraries"
  OFF
  )

IF(FC_SHARED_LIBS)
  SET(BUILD_SHARED_LIBS ON)
ENDIF(FC_SHARED_LIBS)

# set the default build type to featflowlib ()
option(FC_FEATFLOW
  "Build the FullC0ntact library for use with the FEATFLOW simulator"
  OFF
  )

option(USE_OPENMESH
  "Use the OpenMesh library"
  OFF
  )

option(USE_OPENVOLUMEMESH
  "Use the OpenVolumeMesh library"
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

option(USE_EIGEN
  "Use the EIGEN library"
  OFF
  )

option(BUILD_FC_UNIT_TESTS
  "Build the unit tests of the FC library"
  OFF
  )

option(BUILD_BOUNDARY_LAYER_TOOLS
  "Build the boundary layer tools"
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

if(BUILD_BOUNDARY_LAYER_TOOLS)
  SET(USE_OPENMESH ON CACHE BOOL "Build OpenMesh library" FORCE) 
endif(BUILD_BOUNDARY_LAYER_TOOLS)

IF(FC_CUDA_SUPPORT)
  include(./cmake_scripts/GenerateBuildIds.cmake)
ENDIF(FC_CUDA_SUPPORT)

#==================================================================================================
#                                     External Libraries    
#==================================================================================================
if(USE_OPENMESH)
  if(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenMesh")
    ADD_SUBDIRECTORY(libs/OpenMesh)
  endif(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenMesh")
endif()

if(USE_OPENVOLUMEMESH)
  if(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenVolumeMesh")
    ADD_SUBDIRECTORY(libs/OpenVolumeMesh/)
  endif(EXISTS "${CMAKE_SOURCE_DIR}/libs/OpenVolumeMesh")
endif()

if(USE_ODE)
  if(EXISTS "${CMAKE_SOURCE_DIR}/libs/ode-cmake")
    set(ODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/ode-cmake")
    ADD_SUBDIRECTORY(libs/ode-cmake)
  endif(EXISTS "${CMAKE_SOURCE_DIR}/libs/ode-cmake")
endif()

if(USE_CGAL)
  find_package(CGAL REQUIRED)
  if(CGAL_FOUND)
    find_package(GMP REQUIRED)
    find_package(MPFR REQUIRED)

    if(NOT GMP_FOUND)
      message(FATAL_ERROR "GMP library not found. Cannot build cgal applications.")
    endif()

    if(NOT MPFR_FOUND)
      message(WARNING "MPFR library not found. Cannot build cgal applications.")
    endif()

  endif()	
endif(USE_CGAL)

if(USE_OPTICALTWEEZERS)
  set(OPTICALTWEEZERS_LIBRARY True)
  
  ExternalProject_Add(OPTICALTWEEZERS_PRJ
    GIT_REPOSITORY ssh://rmuenste@arryn.mathematik.tu-dortmund.de:22122/home/user/rmuenste/nobackup/code/test_system15/Feat_FloWer/extern/libraries/opticaltweezers
    GIT_TAG mit_octree
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers
    PREFIX ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers-dir
    UPDATE_DISCONNECTED True
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/extern/libraries/opticaltweezers-install -DBUILD_SHARED_LIBS=False -DOPTICALTWEEZERS_LIBRARY=True
    )

  set(OPTICALTWEEZERS_LIBRARIES 
      ${CMAKE_BINARY_DIR}/extern/libraries/opticaltweezers-install/lib/libvector.a
      ${CMAKE_BINARY_DIR}/extern/libraries/opticaltweezers-install/lib/libstrahl.a
      ${CMAKE_BINARY_DIR}/extern/libraries/opticaltweezers-install/lib/libot.a
     )

  add_definitions(-DOPTIC_FORCES)
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

include(cmake_scripts/ConfigureIncludeDirs.cmake)

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
      distancegridtest inclinedplane mathtest shapestest sequentialimpulses boxmesher test0 test1 openmeshtest openmeshtest2 openmeshtest3 openmeshtest4 interpolation 
      qt_opengl_test trypanosome soft_body2 ecoli multiblockgrid taylor_line peristaltic peristaltic_ogl ode-test ode-app meshtest_cgal meshtest_cgal2 preprocessing_test) 
  endif(FC_CUDA_SUPPORT)

#==================================================================================================
#                                   Configure Application Directories
#==================================================================================================
  foreach(app ${APP_DIRS})
    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/meshes AND EXISTS ${CMAKE_SOURCE_DIR}/applications/${app}/meshes)
      execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/applications/${app}/meshes ${CMAKE_BINARY_DIR}/applications/${app}/meshes)
    endif ()

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

#==================================================================================================
#                                   Add the unit tests
#==================================================================================================
add_subdirectory(unit_tests)

#==================================================================================================
#                                   generate documentation
#==================================================================================================
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

#==================================================================================================
#                                     Add Tests for CTest
#==================================================================================================
ADD_TEST(NAME VelocityBased WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/velocitybased COMMAND velocitybased)
ADD_TEST(NAME ShapesTest WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/shapestest COMMAND shapestest)
ADD_TEST(NAME SequentialImpulses WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/sequentialimpulses COMMAND sequentialimpulses)
ADD_TEST(NAME MeshMesh WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/meshmeshtest COMMAND meshmeshtest)