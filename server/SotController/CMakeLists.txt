add_executable(controller-sample
  sample-robot-o2sr.cpp
  server-sample.cpp)


if(UNIX)
  target_link_libraries(
    controller-sample
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY}
    #boost_filesystem boost_program_options boost_regex
    #boost_system
    )

endif()

install(TARGETS controller-sample DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
