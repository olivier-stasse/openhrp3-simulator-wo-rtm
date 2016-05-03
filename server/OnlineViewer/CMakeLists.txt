# @author Shin'ichiro Nakaoka

set(program gepetto-online-viewer)

set(sources OnlineViewer_impl.cpp 
server.cpp 
from-body-to-osg.cpp
main-window.cpp
keyboard-handler.cpp
create-axis.cpp
)

add_executable(${program} ${sources})

if(UNIX)
  pkg_config_use_dependency(${program} gepetto-viewer)
  target_link_libraries(${program}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    hrpModel-${OPENHRP_LIBRARY_VERSION}
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    ${GEPETTO_VIEWER_LIBRARIES}
    pthread
    ${OMNIORB_LIBRARIES} ${Boost_REGEX_LIBRARY} )
elseif(WIN32)
  set_target_properties(${program} PROPERTIES DEBUG_POSTFIX d)
  target_link_libraries(${program}
    optimized hrpUtil-${OPENHRP_LIBRARY_VERSION}
    optimized hrpModel-${OPENHRP_LIBRARY_VERSION}
    optimized hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    debug hrpUtil-${OPENHRP_LIBRARY_VERSION}d
    debug hrpModel-${OPENHRP_LIBRARY_VERSION}d
    debug hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}d
    ${OMNIORB_LIBRARIES})  
endif()

if(WIN32)
install(TARGETS ${program} DESTINATION ${PROJECT_BINARY_DIR}/bin CONFIGURATIONS Release)
endif()

install(TARGETS ${program} DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
