#________________________________________________________________
#
# Inshape3d Library : math folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# set the include directories
include_directories (. ../util ../path)

# set the source code to the variable math_src
set(math_src
	lemkesolver.cpp
	linearsolvergauss.cpp
	lcpsolvergaussseidel.cpp
	lcpsolverjacobi.cpp
	lcpsolver.cpp
	linearsolverlu.cpp
	matrix2x2.cpp 
	matrix3x3.cpp 
	matrix4x4.cpp
	matrixnxn.cpp 
	matrixcsr.cpp 	
	mymath.cpp 
	quaternion.cpp 
	torus.cpp 
	vector2.cpp 
	vector3.cpp 
	vector4.cpp
	vectorn.cpp)
	
# set the header files to the variable math_hdr	
set(math_hdr
	lemkesolver.h
	linearsolvergauss.h
	lcpsolvergaussseidel.h
	lcpsolverjacobi.h
	lcpsolver.h
	linearsolverlu.h
	mathglobals.h
	matrix2x2.h 
	matrix3x3.h 
	matrix4x4.h
	matrixcsr.h 	
	matrixnxn.h 	
	mymath.h 
	quaternion.h 
	torus.h 
	vector2.h 
	vector3.h 
	vector4.h
	vectorn.h)

# add the math library
if(FC_CUDA_SUPPORT)
  cuda_add_library(Math ${math_src} ${math_hdr})
else(FC_CUDA_SUPPORT)
  add_library(Math ${math_src} ${math_hdr})
endif(FC_CUDA_SUPPORT)