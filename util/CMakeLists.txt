#________________________________________________________________
#
# Inshape3d Library : util folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# set the include directories
include_directories (.)

# set a variable for the source code
set(utilities_src dynamicarray.cpp log.cpp perftimer.cpp)

# set a variable for the header files
set(utilities_hdr dynamicarray.h
log.h
perftimer.h)

IF(CMAKE_COMPILER_IS_GNUCXX)
  # We need to add the rt library
  FIND_LIBRARY(SYSTEM_LIBRARY_RT NAMES rt PATHS /usr/lib/)
  IF(SYSTEM_LIBRARY_RT)
      MESSAGE(STATUS "Found librt library: ${SYSTEM_LIBRARY_RT}")
  ELSE(SYSTEM_LIBRARY_RT)
      MESSAGE(STATUS "Could NOT find librt library.")
      MESSAGE(FATAL_ERROR "The util library requires librt library.")
  ENDIF(SYSTEM_LIBRARY_RT) 
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# add a library
if(FC_CUDA_SUPPORT)
  cuda_add_library (Utility ${utilities_src} ${utilities_hdr})
else(FC_CUDA_SUPPORT)
  add_library (Utility ${utilities_src} ${utilities_hdr})
endif(FC_CUDA_SUPPORT)

target_link_libraries(Utility ${SYSTEM_LIBRARY_RT})

target_compile_options(Utility PRIVATE ${CXX_FLAGS_FC})
