set(program scheduler)

set(sources 
  simscheduler.cpp
  scheduler.cpp
)

# Scheduler: Tries to launch a specific C++ simulator for the sample humanoid robot.
add_executable(${program} ${sources})
include_directories(${CMAKE_SOURCE_DIR}/sample/example/scheduler)
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


# Scheduler project: Load a project
configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/schedulerproject.${SH_SFX}.in"
  "${CMAKE_CURRENT_BINARY_DIR}/schedulerproject.${SH_SFX}")

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/schedulerproject.${SH_SFX} 
  DESTINATION example/${PROJECT_NAME}/scheduler
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/schedulerproject-sample1-bush.${SH_SFX}.in"
  "${CMAKE_CURRENT_BINARY_DIR}/schedulerproject-sample1-bush.${SH_SFX}")

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/schedulerproject-sample1-bush.${SH_SFX} 
  DESTINATION example/${PROJECT_NAME}/scheduler
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

add_executable(schedulerproject
  mainschedulerproject.cpp
  loadproject.cpp
  schedulerproject.cpp
  logproject.cpp
)

target_link_libraries(
  schedulerproject
  hrpUtil-${OPENHRP_LIBRARY_VERSION}
  hrpModel-${OPENHRP_LIBRARY_VERSION})

pkg_config_use_dependency(schedulerproject libxml-2.0)

install(TARGETS schedulerproject
  DESTINATION example/${PROJECT_NAME}/scheduler
  CONFIGURATIONS Release Debug RelWithDebInfo)
