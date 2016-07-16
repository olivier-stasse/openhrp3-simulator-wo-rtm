set(program scheduler)

set(sources 
  scheduler.cpp
)

add_executable(${program} ${sources})

if (UNIX)
  SET(SH_SFX "sh")
endif()

if(UNIX)
  target_link_libraries(
    ${program}
     hrpUtil-${OPENHRP_LIBRARY_VERSION}
     hrpModel-${OPENHRP_LIBRARY_VERSION})
   configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${program}.${SH_SFX}.in"
     "${CMAKE_CURRENT_BINARY_DIR}/${program}.${SH_SFX}")

   configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${program}-hrp2.${SH_SFX}.in"
     "${CMAKE_CURRENT_BINARY_DIR}/${program}-hrp2.${SH_SFX}")

elseif(WIN32)
  set_target_properties(${program} PROPERTIES DEBUG_POSTFIX d )
  target_link_libraries(${program}
    optimized hrpUtil-${OPENHRP_LIBRARY_VERSION}
    optimized hrpModel-${OPENHRP_LIBRARY_VERSION}
    optimized hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    debug hrpUtil-${OPENHRP_LIBRARY_VERSION}d
    debug hrpModel-${OPENHRP_LIBRARY_VERSION}d
    debug hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}d
    ${OMNIORB_LIBRARIES})
endif()

install(TARGETS ${program} 
  DESTINATION example/${PROJECT_NAME}/scheduler 
  CONFIGURATIONS Release Debug RelWithDebInfo)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${program}.${SH_SFX} 
  DESTINATION example/${PROJECT_NAME}/scheduler
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${program}-hrp2.${SH_SFX} 
  DESTINATION example/${PROJECT_NAME}/scheduler
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)