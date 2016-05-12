# @author Shin'ichiro Nakaoka

set(program gepetto-online-viewer)

set(sources OnlineViewer_impl.cpp 
server.cpp 
from-body-to-osg.cpp
main-window.cpp
keyboard-handler.cpp
create-axis.cpp
olv-link-info.cpp
)

INCLUDE_DIRECTORIES(${GEPETTO_VIEWER_INCLUDEDIR})
add_executable(${program} ${sources})

if(UNIX)
  target_link_libraries(${program}
    hrpUtil-${OPENHRP_LIBRARY_VERSION}
    hrpModel-${OPENHRP_LIBRARY_VERSION}
    hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    pthread
    ${OMNIORB_LIBRARIES} ${Boost_REGEX_LIBRARY} )
  pkg_config_use_dependency(${program} gepetto-viewer)
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
