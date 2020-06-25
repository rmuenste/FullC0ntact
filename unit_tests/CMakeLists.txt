if(BUILD_FC_UNIT_TESTS)
  add_subdirectory(integration)
  add_subdirectory(fixed_point)
  add_subdirectory(interpolation)
  add_subdirectory(eigen_test)
  add_subdirectory(eigen_solving)
  add_subdirectory(xml_reader)
  add_subdirectory(json_reader)

  if(USE_OPENMESH)
    add_subdirectory(openmesh_unit_tests)
    add_subdirectory(openmesh_io)
    add_subdirectory(openmesh_traits_extend)
    add_subdirectory(pbd_dist)
    add_subdirectory(openmesh_traits)
    if(USE_CGAL)
      add_subdirectory(pbd_sim)
    endif(USE_CGAL)
  endif()

  if(USE_OPENVOLUMEMESH)
    add_subdirectory(ovm)
    add_subdirectory(elastic_hexahedron)
    add_subdirectory(mesh_converter)
  endif(USE_OPENVOLUMEMESH)
else(BUILD_FC_UNIT_TESTS)

  if(BUILD_BOUNDARY_LAYER_TOOLS)
    add_subdirectory(openmesh_traits)
  endif(BUILD_BOUNDARY_LAYER_TOOLS)
endif(BUILD_FC_UNIT_TESTS)





