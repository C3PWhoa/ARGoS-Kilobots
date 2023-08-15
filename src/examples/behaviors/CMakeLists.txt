if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

  #
  # Lab0: Blinky
  #
  add_executable(blinky blinky.c)
  target_link_libraries(blinky argos3plugin_simulator_kilolib)

  #
  # Lab1.2: Simple Movement
  #
  add_executable(simple_movement simple_movement.c)
  target_link_libraries(simple_movement argos3plugin_simulator_kilolib)

  #
  # Lab1.3: Non-blocked Movement
  #
  add_executable(nonblocked_movement nonblocked_movement.c)
  target_link_libraries(nonblocked_movement argos3plugin_simulator_kilolib)

  #
  # Lab2.1-2.2: Test Speaker and Test Listener
  #
  add_executable(test_speaker test_speaker.c)
  target_link_libraries(test_speaker argos3plugin_simulator_kilolib)
  add_executable(test_listener test_listener.c)
  target_link_libraries(test_listener argos3plugin_simulator_kilolib)

  #
  # Lab2.3-2.4: Modified Test Speaker and Test Listener
  #
  add_executable(test_speaker_mod test_speaker_mod.c)
  target_link_libraries(test_speaker_mod argos3plugin_simulator_kilolib)
  add_executable(test_listener_mod test_listener_mod.c)
  target_link_libraries(test_listener_mod argos3plugin_simulator_kilolib)

  #
  # Lab3: Disperse
  #
  add_executable(disperse disperse.c)
  target_link_libraries(disperse argos3plugin_simulator_kilolib)

  #
  # Lab4: Orbit
  #
  add_executable(orbit_star orbit_star.c)
  target_link_libraries(orbit_star argos3plugin_simulator_kilolib)
  add_executable(orbit_planet orbit_planet.c)
  target_link_libraries(orbit_planet argos3plugin_simulator_kilolib)

  #
  # Lab5: Move to Light
  #
  add_executable(move_to_light move_to_light.c)
  target_link_libraries(move_to_light argos3plugin_simulator_kilolib)

  #
  # Lab6: Simple Gradient
  #
  add_executable(gradient_simple gradient_simple.c)
  target_link_libraries(gradient_simple argos3plugin_simulator_kilolib)

  #
  # Lab7: Sync
  #
  add_executable(sync sync.c)
  target_link_libraries(sync argos3plugin_simulator_kilolib)

  #
  # Debugging example
  #
  add_executable(test_debug test_debug.h test_debug.c)
  target_link_libraries(test_debug argos3plugin_simulator_kilolib)

  #
  # Lab 0 : Distance
  #
  add_executable(distance distance.c)
  target_link_libraries(distance argos3plugin_simulator_kilolib)

  #
  # Lab 8 : Orbit with 3 stars
  #
  add_executable(orbit_star_2 orbit_star_2.c)
  target_link_libraries(orbit_star_2 argos3plugin_simulator_kilolib)
  add_executable(orbit_planet_2 orbit_planet_2.c)
  target_link_libraries(orbit_planet_2 argos3plugin_simulator_kilolib)

  #
  # Lab 9 : Orbit with 3 stars and stop between 1 & 2
  #
  add_executable(stop_orbit_planet stop_orbit_planet.c)
  target_link_libraries(stop_orbit_planet argos3plugin_simulator_kilolib)
  add_executable(stop_orbit_star stop_orbit_star.c)
  target_link_libraries(stop_orbit_star argos3plugin_simulator_kilolib)

  #
  # Lab 10 : Orbit with 3 stars and 4 planets
  #
  add_executable(multi_orbit_planet multi_orbit_planet.c)
  target_link_libraries(multi_orbit_planet argos3plugin_simulator_kilolib)
  add_executable(multi_orbit_star multi_orbit_star.c)
  target_link_libraries(multi_orbit_star argos3plugin_simulator_kilolib)

  #
  # Lab 11 : Shape
  #
  add_executable(shape shape.c)
  target_link_libraries(shape argos3plugin_simulator_kilolib)
  add_executable(shape_star shape_star.c)
  target_link_libraries(shape_star argos3plugin_simulator_kilolib)

  #
  # Lab 12 : Phototaxis
  #
  add_executable(phototaxis phototaxis.c)
  target_link_libraries(phototaxis argos3plugin_simulator_kilolib)
  add_executable(phototaxis_star phototaxis_star.c)
  target_link_libraries(phototaxis_star argos3plugin_simulator_kilolib)

  #
  # Lab 13 : big_form
  #
  add_executable(big_form big_form.c)
  target_link_libraries(big_form argos3plugin_simulator_kilolib)
  add_executable(big_form_star big_form_star.c)
  target_link_libraries(big_form_star argos3plugin_simulator_kilolib)

  #
  # Lab 14 : shape54
  #
  add_executable(shape54 shape54.c)
  target_link_libraries(shape54 argos3plugin_simulator_kilolib)
  add_executable(shape54_star shape54_star.c)
  target_link_libraries(shape54_star argos3plugin_simulator_kilolib)


  #
  # Lab 15 : oneBotMove
  #
  add_executable(oneBotMove oneBotMove.c)
  target_link_libraries(oneBotMove argos3plugin_simulator_kilolib)

  #
  # Lab 16 : Collision
  #
  add_executable(shapeCollision shapeCollision.c)
  target_link_libraries(shapeCollision argos3plugin_simulator_kilolib)
  add_executable(shapeCollision_star shapeCollision_star.c)
  target_link_libraries(shapeCollision_star argos3plugin_simulator_kilolib)

  #
  # Lab 17 : exp2 - id aléatoire
  #
  add_executable(exp2 exp2.c)
  target_link_libraries(exp2 argos3plugin_simulator_kilolib)
  add_executable(exp2_star exp2_star.c)
  target_link_libraries(exp2_star argos3plugin_simulator_kilolib)

  #
  # Lab 18 : exp3 - collision
  #
  add_executable(exp3 exp3.c)
  target_link_libraries(exp3 argos3plugin_simulator_kilolib)
  add_executable(exp3_satr exp3_satr.c)
  target_link_libraries(exp3_satr argos3plugin_simulator_kilolib)

  #
  # Lab 19 : copie exp2 + collision
  #
  add_executable(collision collision.c)
  target_link_libraries(collision argos3plugin_simulator_kilolib)
  add_executable(collision_star collision_star.c)
  target_link_libraries(collision_star argos3plugin_simulator_kilolib)

  #
  # Lab 20 : Coordiantes
  #
  add_executable(coordinates_v1 coordinates_v1.c)
  target_link_libraries(coordinates_v1 argos3plugin_simulator_kilolib)


endif(ARGOS_BUILD_FOR_SIMULATOR)
