# @author Shin'ichiro Nakaoka
find_package(LibXml2)
if(LIBXML2_FOUND)
  include_directories(${LIBXML2_INCLUDE_DIR})
endif()

set(program openhrp-model-loader)
set(exporter openhrp-export-collada)
set(generator openhrp-project-generator)

set(sources
  ShapeSetInfo_impl.cpp
  SceneInfo_impl.cpp
  BodyInfo_impl.cpp
  ModelLoader_impl.cpp
  VrmlUtil.cpp
  server.cpp )

set(sources2
  exportCollada.cpp
  BodyInfo_impl.cpp
  ShapeSetInfo_impl.cpp
  VrmlUtil.cpp )

set(sources3
  exportVrml.cpp
  VrmlWriter.cpp )

set(sources4
  projectGenerator.cpp
  BodyInfo_impl.cpp
  ShapeSetInfo_impl.cpp
  VrmlUtil.cpp )


if( COLLADA_DOM_FOUND )
  set(sources ${sources} BodyInfoCollada_impl.cpp)
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  link_directories(${COLLADA_DOM_LIBRARY_DIRS})
  add_executable(${exporter} ${sources2})
  set(extralibraries  ${COLLADA_DOM_LIBRARIES} boost_system boost_thread)
else( COLLADA_DOM_FOUND )
  set(extralibraries)
endif( COLLADA_DOM_FOUND )

add_executable(${program} ${sources})

if(UNIX)
  target_link_libraries(${program}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    hrpModel-${OPENHRP_LIBRARY_VERSION}
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    ${OMNIORB_LIBRARIES}
    ${extralibraries}
    )

  if( COLLADA_DOM_FOUND )
    target_link_libraries(${exporter}
      hrpUtil-${OPENHRP_LIBRARY_VERSION}
      hrpModel-${OPENHRP_LIBRARY_VERSION}
      hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
      ${OMNIORB_LIBRARIES}
      ${extralibraries}
      )
    set_target_properties(${exporter} PROPERTIES COMPILE_FLAGS "${COLLADA_DOM_CXX_FLAGS}" LINK_FLAGS "${COLLADA_DOM_LINK_FLAGS}")
  endif( COLLADA_DOM_FOUND )

endif()

if( COLLADA_DOM_FOUND )
  set_target_properties(${program} PROPERTIES COMPILE_FLAGS "${COLLADA_DOM_CXX_FLAGS}" LINK_FLAGS "${COLLADA_DOM_LINK_FLAGS}")
endif( COLLADA_DOM_FOUND )

add_executable(export-vrml ${sources3})

target_link_libraries(export-vrml
  hrpModel-${OPENHRP_LIBRARY_VERSION}
  hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
  )

install(TARGETS ${program} export-vrml DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
if( COLLADA_DOM_FOUND )
  install(TARGETS ${exporter} DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
  install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E copy ${exporter} export-collada WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin)") # for backword compatibility
endif( COLLADA_DOM_FOUND )

if(LIBXML2_FOUND)
  if( COLLADA_DOM_FOUND )
    set(sources4 ${sources4} BodyInfoCollada_impl.cpp)
  endif()
  add_executable(${generator} ${sources4})
  target_link_libraries(${generator}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    hrpModel-${OPENHRP_LIBRARY_VERSION}
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    ${LIBXML2_LIBRARIES} ${OPENRTM_LIBRARIES}
    ${extralibraries}
    )
  if( COLLADA_DOM_FOUND )
    set_target_properties(${generator} PROPERTIES COMPILE_FLAGS "${COLLADA_DOM_CXX_FLAGS}" LINK_FLAGS "${COLLADA_DOM_LINK_FLAGS}")
  endif( COLLADA_DOM_FOUND )
  install(TARGETS ${generator} DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
endif()
