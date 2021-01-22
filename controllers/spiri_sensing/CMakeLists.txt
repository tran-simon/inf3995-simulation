add_library(spiri_sensing SHARED spiri_sensing.h spiri_sensing.cpp)
target_link_libraries(spiri_sensing
  argos3core_simulator
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)
