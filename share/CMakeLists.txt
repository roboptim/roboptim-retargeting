INSTALL(FILES
  roboptim/retargeting/data/human.mars
  roboptim/retargeting/data/human.trc
  DESTINATION share/roboptim/retargeting/data)

FIND_PROGRAM(GZIP gzip)
MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/man/man1/")

SET(MANPAGES
  roboptim-retargeting-markers
  roboptim-retargeting-markers-to-joints
  roboptim-retargeting-joints)

FOREACH(MANPAGE ${MANPAGES})
  EXECUTE_PROCESS(
    COMMAND "${GZIP}" "-c"
    "${CMAKE_CURRENT_SOURCE_DIR}/man/man1/${MANPAGE}.1"
    OUTPUT_FILE
    ${CMAKE_CURRENT_BINARY_DIR}/man/man1/${MANPAGE}.1.gz)

  INSTALL(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/man/man1/${MANPAGE}.1.gz"
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
ENDFOREACH()
