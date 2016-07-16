add_executable(controller-sample
  sample-robot-o2sr.cpp
  server-sample.cpp)

add_required_dependency(sot-core)
add_required_dependency(dynamic_graph_bridge)

add_executable(controller-hrp2
  hrp2-oh2sot.cpp
  server-hrp2.cpp)

if(UNIX)
  target_link_libraries(
    controller-sample
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY}
    #boost_filesystem boost_program_options boost_regex
    #boost_system
    )

  target_link_libraries(
    controller-hrp2
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    sot_loader roscpp xmlrpcpp
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY}
    dl
    #boost_filesystem boost_program_options boost_regex
    #boost_system
    )
  pkg_config_use_dependency(controller-hrp2 sot-core)
  pkg_config_use_dependency(controller-hrp2 dynamic_graph_bridge)

endif()

install(TARGETS controller-sample DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
install(TARGETS controller-hrp2 DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
