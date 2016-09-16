macro(compile_customizer _libname _sources)
  add_library(${_libname} SHARED ${_sources})
  if(UNIX)
    target_link_libraries(
      ${_libname}
      hrpModel-${OPENHRP_LIBRARY_VERSION})
  endif()
endmacro()

compile_customizer(BUSHCustomizer bush_customizer.cpp)
set(libnames BUSHCustomizer)

install(TARGETS ${libnames} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/customizer CONFIGURATIONS Release Debug RelWithDebInfo)

install(FILES springJoint_param.conf sample1_bush_customizer_param.conf DESTINATION ${CMAKE_INSTALL_PREFIX}/share/customizer CONFIGURATIONS Release Debug RelWithDebInfo)