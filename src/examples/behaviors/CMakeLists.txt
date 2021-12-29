if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

  #
  # ARK loop function : dhtf
  #
  add_executable(kilobot_ALF_dhtf kilobot_ALF_dhtf.c)  
  target_link_libraries(kilobot_ALF_dhtf argos3plugin_simulator_kilolib)
  
  #
  # ARK loop function : aggregation
  #
  add_executable(kilobot_ALF_aggregation kilobot_ALF_aggregation.c)  
  target_link_libraries(kilobot_ALF_aggregation argos3plugin_simulator_kilolib)

  #
  # Debugging example
  #
  add_executable(test_debug test_debug.h test_debug.c)
  target_link_libraries(test_debug argos3plugin_simulator_kilolib)
  endif(ARGOS_BUILD_FOR_SIMULATOR)
