add_library(crazyflie_sensing SHARED 
            crazyflie_sensing.h 
            crazyflie_sensing.cpp
            crazyflie_qt_user_functions.h
            crazyflie_qt_user_functions.cpp)
target_link_libraries(crazyflie_sensing
  argos3core_simulator
  argos3plugin_simulator_crazyflie
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl)
