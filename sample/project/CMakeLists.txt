configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/SpringJoint.xml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/SpringJoint.xml")

cmake_policy(PUSH)
#cmake_policy(SET CMP0053 OLD)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/SpringJoint.xml
  DESTINATION share/@PROJECT_NAME@/project
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ  WORLD_READ )

configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/sample1_bush.xml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/sample1_bush.xml")

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/sample1_bush.xml
  DESTINATION share/@PROJECT_NAME@/project
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ  WORLD_READ )

configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/sample1_bush_tiltplate.xml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/sample1_bush_tiltplate.xml")

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/sample1_bush_tiltplate.xml
  DESTINATION share/@PROJECT_NAME@/project
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ  WORLD_READ )

cmake_policy(POP)

