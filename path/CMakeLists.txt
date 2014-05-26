#________________________________________________________________
#
# Inshape3d Library : path folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# set the include directories
include_directories (. ../util)

# set the source code to the variable path_src
set(path_src SimpleLCP_Path.c)

	
# set the header files to the variable path_hdr	
set(path_hdr SimpleLCP.h)	
	
# add the path interface library	
add_library (path ${path_src} ${path_hdr})

