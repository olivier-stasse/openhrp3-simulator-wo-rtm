
include(IDL_FILE_BASENAMES.cmake)

set(idl_files)
foreach(name ${IDL_FILE_BASENAMES})
  list(APPEND idl_files ${name}.idl)
endforeach()

install(FILES ${idl_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/idl/OpenHRP)
