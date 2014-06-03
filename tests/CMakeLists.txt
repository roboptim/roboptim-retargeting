# Make sure local headers are found
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/tests)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

MACRO(ROBOPTIM_RETARGETING_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against main library.
  TARGET_LINK_LIBRARIES(${NAME} roboptim-retargeting)

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})

  # roboptim-core and roboptim-trajectory
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-trajectory)

  # Choreonoid
  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid-body-plugin)

  # Unfortunately, the motion capture part does not provide any
  # pkg-config file.
  TARGET_LINK_LIBRARIES(${NAME} CnoidMocap CnoidMocapPlugin)
ENDMACRO()


# Look for the HRP4C model.
FIND_PATH(HRP4C_DIRECTORY
  NAMES HRP4Cg2main.wrl HRP4Cg2.yaml HRP4Cmain.wrl HRP4C.yaml
  DOC "HRP-4C directory (as expected by Choreonoid, i.e. containing YAML files)"
  )

IF(NOT HRP4C_DIRECTORY)
  MESSAGE(FATAL_ERROR "HRP4-C model is required for the test suite")
ENDIF()
ADD_DEFINITIONS(-DHRP4C_YAML_FILE="${HRP4C_DIRECTORY}/HRP4Cg2.yaml")

# Data directory.
ADD_DEFINITIONS(
  -DDATA_DIR="${CMAKE_SOURCE_DIR}/share/roboptim/retargeting/data")



ROBOPTIM_RETARGETING_TEST(morphing)

ADD_SUBDIRECTORY(function)
