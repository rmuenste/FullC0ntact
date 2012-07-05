set(FC_BUILD_ID_FOUND false)

# Intel compiler builds

IF(FC_BUILD_ID STREQUAL "nehalem-linux-intel-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -xSSE4.2 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -xSSE4.2 -funroll-loops -fp-model precise -no-prec-div")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "nehalem-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "core2duo-linux-intel-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -xSSE3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -xSSE3 -funroll-loops -fp-model precise -no-prec-div")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "core2duo-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "opteron-linux-intel-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteron-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-intel-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-intel-release")

# GNU compiler builds

IF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=amdfam10 -msse4a -ffast-math -funroll-loops")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=amdfam10 -msse4a -ffast-math -funroll-loops")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")

#--------------------------------------------------------------------------------------------------------------------------------
#                                                      Debug builds
#--------------------------------------------------------------------------------------------------------------------------------

IF(FC_BUILD_ID MATCHES "debug")
  SET(CMAKE_BUILD_TYPE "Debug")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID MATCHES "debug")

# IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")
#   SET(CMAKE_BUILD_TYPE "Debug")
#   SET(FC_BUILD_ID_FOUND true)
# ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")

