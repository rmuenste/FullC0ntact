# add a binary
add_executable(boundarylayer_generator meshtraits.cpp)

get_target_property(MYTARGET boundarylayer_generator NAME)

# add dependent libraries
#  target_link_libraries(${MYTARGET} OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
target_link_libraries(${MYTARGET} OpenMeshCoreStatic OpenMeshToolsStatic Utility Math inshape3dcore)
target_compile_definitions(${MYTARGET} PUBLIC OM_STATIC_BUILD)
#endif(FC_SHARED_LIBS)


if(USE_CGAL)
  target_link_libraries(${MYTARGET} ${CGAL_LIBRARIES}) 
endif(USE_CGAL)

# add include directories
target_include_directories(${MYTARGET} PUBLIC ${FC_APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/libs/OpenMesh/src)


#=========================================================================
#                       INSTALL CONFIGURATION
#=========================================================================
install(TARGETS ${MYTARGET}
    RUNTIME DESTINATION bin/${MYTARGET}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

file(GLOB BL_SCRIPTS "../boundarylayer_generation/*.py")
install(FILES ${BL_SCRIPTS} DESTINATION bin/${MYTARGET}
    PERMISSIONS WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

#set(DEF_DIRS _mesh _vtk _data _dump start _1D _hist _prot0 _prot1 _RTD _data_BU _ianus)
#
#FOREACH(dir ${DEF_DIRS})
#  list(APPEND mydirs ${CMAKE_CURRENT_BINARY_DIR}/${dir})
#ENDFOREACH()
#
#install(DIRECTORY ${mydirs} DESTINATION bin/${SSE_TARGET_NAME})


