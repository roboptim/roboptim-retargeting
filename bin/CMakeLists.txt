MACRO(ADD_ROBOPTIM_RETARGETING_TOOL NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} roboptim-retargeting)

  TARGET_LINK_LIBRARIES(${NAME} boost_program_options)

  PKG_CONFIG_USE_DEPENDENCY(${NAME} libmocap)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)

  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid-body-plugin)

  PKG_CONFIG_USE_DEPENDENCY(${NAME} yaml-cpp)

  # Unfortunately, the motion capture part does not provide any
  # pkg-config file.
  TARGET_LINK_LIBRARIES(${NAME} CnoidMocap CnoidMocapPlugin)

  INSTALL(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
ENDMACRO()

ADD_ROBOPTIM_RETARGETING_TOOL(roboptim-retargeting-joints)
ADD_ROBOPTIM_RETARGETING_TOOL(roboptim-retargeting-markers)
ADD_ROBOPTIM_RETARGETING_TOOL(roboptim-retargeting-markers-to-joints)
