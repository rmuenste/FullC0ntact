#
# The following module is based on FindVTK.cmake
#

# - Find a CGAL installation or binary tree.
# The following variables are set if CGAL is found.  If CGAL is not
# found, CGAL_FOUND is set to false.
#
#  CGAL_FOUND         - Set to true when CGAL is found.
#  CGAL_USE_FILE      - CMake file to use CGAL.
#

# Construct consitent error messages for use below.
set(CGAL_DIR_DESCRIPTION "directory containing CGALConfig.cmake. This is either the binary directory where CGAL was configured or PREFIX/lib/CGAL for an installation.")
set(CGAL_DIR_MESSAGE     "CGAL not found.  Set the CGAL_DIR cmake variable or environment variable to the ${CGAL_DIR_DESCRIPTION}")

if ( NOT CGAL_DIR )
  
  # Get the system search path as a list.
  if(UNIX)
    string(REGEX MATCHALL "[^:]+" CGAL_DIR_SEARCH1 "$ENV{PATH}")
  else()
    string(REGEX REPLACE "\\\\" "/" CGAL_DIR_SEARCH1 "$ENV{PATH}")
  endif()
  
  string(REGEX REPLACE "/;" ";" CGAL_DIR_SEARCH2 "${CGAL_DIR_SEARCH1}")

  # Construct a set of paths relative to the system search path.
  set(CGAL_DIR_SEARCH "")
  
  foreach(dir ${CGAL_DIR_SEARCH2})
  
    set(CGAL_DIR_SEARCH ${CGAL_DIR_SEARCH} ${dir}/../lib/CGAL )
      
  endforeach()


  #
  # Look for an installation or build tree.
  #
  find_path(CGAL_DIR CGALConfig.cmake

    # Look for an environment variable CGAL_DIR.
    $ENV{CGAL_DIR}

    # Look in places relative to the system executable search path.
    ${CGAL_DIR_SEARCH}

    # Look in standard UNIX install locations.
    /usr/local/lib/CGAL
    /usr/lib/CGAL

    # Read from the CMakeSetup registry entries.  It is likely that
    # CGAL will have been recently built.
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild1]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild2]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild3]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild4]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild5]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild6]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild7]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild8]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild9]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild10]

    # Help the user find it if we cannot.
    DOC "The ${CGAL_DIR_DESCRIPTION}"
  )
  
endif()

if ( CGAL_DIR )
  set(CGAL_ROOT "")  
  if ( EXISTS "${CGAL_DIR}/CGALConfig.cmake" )
    include( "${CGAL_DIR}/CGALConfig.cmake" )
    set( CGAL_FOUND TRUE )
    if (CGAL_MAJOR_VERSION EQUAL 4)
      set(CGAL_ROOT ${CGAL_DIR}/../../)  
    elseif (CGAL_MAJOR_VERSION EQUAL 5)
      set(CGAL_ROOT ${CGAL_DIR}/../../../)  
    else (CGAL_MAJOR_VERSION EQUAL 4)
    endif (CGAL_MAJOR_VERSION EQUAL 4)

    # These are for debuggin purposes
    #MESSAGE(STATUS "We found the CGAL_DIR: ${CGAL_DIR}")
  endif()

 find_path(CGAL_INCLUDE_DIR CGAL/basic.h
      ${CGAL_ROOT}/include
      ${CGAL_ROOT}
      /usr/include
      /usr/local/include
      $ENV{ProgramFiles}/CGAL/*/include
      $ENV{SystemDrive}/CGAL/*/include
      )

  find_library(CGAL_LIB_MAIN NAMES CGAL libCGAL
         PATHS
         ${CGAL_DIR}/lib
         ${CGAL_ROOT}/lib
         /usr/lib
         /usr/local/lib
         /usr/lib/CGAL
         /usr/lib64
         /usr/local/lib64
         /usr/lib64/CGAL
         $ENV{ProgramFiles}/CGAL/*/lib
         $ENV{SystemDrive}/CGAL/*/lib
         )

  find_library(CGAL_LIB_CORE NAMES CGAL_Core libCGAL_Core
         PATHS
         ${CGAL_DIR}/lib
         ${CGAL_ROOT}/lib
         /usr/lib
         /usr/local/lib
         /usr/lib/CGAL
         /usr/lib64
         /usr/local/lib64
         /usr/lib64/CGAL
         $ENV{ProgramFiles}/CGAL/*/lib
         $ENV{SystemDrive}/CGAL/*/lib
         )
  
  set(CGAL_LIBRARIES "")

  if(CGAL_LIB_MAIN-NOTFOUND)
    MESSAGE(FATAL_ERROR "We did not find the CGAL_MAIN_LIBRARY.")
  else()
    set(CGAL_LIBRARIES ${CGAL_LIBRARIES} ${CGAL_LIB_MAIN})
  endif()

  if(CGAL_LIB_CORE-NOTFOUND)
    MESSAGE(FATAL_ERROR "We did not find the CGAL_CORE_LIBRARY.")
  else()
    set(CGAL_LIBRARIES ${CGAL_LIBRARIES} ${CGAL_LIB_CORE})
  endif()
    
endif()
  
#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(CGAL DEFAULT_MSG CGAL_LIBRARIES)
if(NOT ${CGAL_VERSION_PUBLIC_RELEASE_VERSION} STREQUAL "")
  message(STATUS "CGAL Version: ${CGAL_VERSION_PUBLIC_RELEASE_VERSION}")
endif(NOT ${CGAL_VERSION_PUBLIC_RELEASE_VERSION} STREQUAL "")
message(STATUS "CGAL Major.Minor: ${CGAL_MAJOR_VERSION}.${CGAL_MINOR_VERSION}")
#set(CGAL_VERSION_PUBLIC_RELEASE_VERSION "5.3-dev")

message(STATUS "CGAL_INCLUDE_DIR=${CGAL_INCLUDE_DIR}")
message(STATUS "CGAL_LIBRARIES=${CGAL_LIBRARIES}")
message(STATUS "CGAL_HEADER_ONLY=${CGAL_HEADER_ONLY}")

if( NOT CGAL_FOUND)
  if(CGAL_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR ${CGAL_DIR_MESSAGE})
  else()
    if(NOT CGAL_FIND_QUIETLY)
      MESSAGE(STATUS ${CGAL_DIR_MESSAGE})
    endif()
  endif()
endif()
