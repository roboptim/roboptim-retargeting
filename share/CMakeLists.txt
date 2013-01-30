INSTALL(FILES
  roboptim/retargeting/log4cxx.xml
  DESTINATION share/roboptim/retargeting)

INSTALL(FILES
  roboptim/retargeting/data/character-cgvu-hrp4c.yaml
  roboptim/retargeting/data/dance_longer-markers.yaml
  roboptim/retargeting/data/dance_one-frame.yaml
  roboptim/retargeting/data/dance_ten-frames.yaml
  DESTINATION share/roboptim/retargeting/data)

FIND_PROGRAM(GZIP gzip)
MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/man/man1/")
EXECUTE_PROCESS(
  COMMAND "${GZIP}" "-c"
  "${CMAKE_CURRENT_SOURCE_DIR}/man/man1/roboptim-retarget-motion.1"
  OUTPUT_FILE
  ${CMAKE_CURRENT_BINARY_DIR}/man/man1/roboptim-retarget-motion.1.gz)

INSTALL(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/man/man1/roboptim-retarget-motion.1.gz"
  DESTINATION share/man/man1)
