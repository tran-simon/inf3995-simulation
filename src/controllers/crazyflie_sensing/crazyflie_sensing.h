/*
 * AUTHORS: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *          Pierre-Yves Lajoie <lajoie.py@gmail.com>
 *
 * An example crazyflie drones sensing.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/foraging.argos
 */

#ifndef CRAZYFLIE_SENSING_H
#define CRAZYFLIE_SENSING_H

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the crazyflie distance sensor */
#include <argos3/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h>
/* Definition of the crazyflie position actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the crazyflie position sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the crazyflie range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the crazyflie range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the crazyflie battery sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include "explore_map.c"
#include "node_array.c"
#include <iostream>
#include <fstream>

/**
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
**/
using namespace argos;

/**
 * A controller is simply an implementation of the CCI_Controller class.
**/
class CCrazyflieSensing : public CCI_Controller {
public:
   /****************************************/
   /*            Enumerations              */
   /****************************************/

   /**
    * @brief Enum that represents mission states.
    * 
    * @param STATE_START      = 0
    * @param STATE_TAKE_OFF   = 1
    * @param STATE_EXPLORE    = 2
    * @param STATE_GO_TO_BASE = 3
    * @param STATE_LAND       = 4
   **/
   enum CfState {
      STATE_START,
      STATE_TAKE_OFF,
      STATE_EXPLORE,
      STATE_GO_TO_BASE,
      STATE_LAND
   };

   /**
    * @brief Enum that represent drone gettable values.
    * 
    * @param STATE    = 0
    * @param BATTERY  = 1
    * @param VELOCITY = 2
    * @param POSITION = 3
    * @param POINT    = 4
    */
   enum CfValue {
      STATE,
      BATTERY,
      VELOCITY,
      POSITION,
      POINT
   };

   /**
    * @brief Enum that represent drone direction of mouvement.
    * 
    * @param FRONT = Y_NEG = 3
    * @param LEFT  = X_POS = 0
    * @param BACK  = Y_POS = 2
    * @param RIGHT = X_NEG = 1
    * @param STOP  = NONE  = 4
    */
   enum CfDir {
      FRONT = MapExplorationDir::Y_NEG,
      LEFT  = MapExplorationDir::X_POS,
      BACK  = MapExplorationDir::Y_POS,
      RIGHT = MapExplorationDir::X_NEG,
      STOP  = MapExplorationDir::NONE
   };

   /* Class constructor. */
   CCrazyflieSensing();

   /* Class destructor. */
   virtual ~CCrazyflieSensing() {}

   /**
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
   **/
   virtual void Init(TConfigurationNode& t_node);

   /**
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
   **/
   virtual void ControlStep();

   /****************************************/
   /*        Connectivity functions        */
   /****************************************/
   /** 
    * @brief This function connects to the socket running on the backend in order 
    * to communicate when to start and land.
    * @return Returns an int value corresponding to the socket or -1 if an error has occured;
   **/
   int ConnectToSocket();

   /** TODO
    * @brief ???
    * 
    * @param fd Socket file descriptor.
    * @return char ???
   **/
   char ReadCommand(int fd);

   /** TODO
    * @brief Create a Command object ???
    * 
    * @param fd ???
    * @param message ???
    * @param value ???
   **/
   void CreateCommand(int fd, char* message, int value);

   /** TODO
    * @brief ???
    * 
    * @param fd ???
    * @param message ???
    * @return int ???
   **/
   int SendCommand(int fd, char* message);

   /****************************************/
   /*       Mission States functions       */
   /****************************************/
   
   /**
    * This function lifts the drone from the ground
   **/
   void TakeOff();

   /**
    * This function makes the drone go on an adventure. Will he survive
    * the long and dangerous journey?
   **/
   void Explore();

   /**
    * This function makes the drone go back to its take off position
   **/
   void GoToBase();
   
   /**
    * This function returns the drone to the ground
   **/
   void Land();   

   /****************************************/
   /*           Movements functions        */
   /****************************************/

   /** 
    * @brief This function return the best direction for the drone
    * @return cfDir:CfDir the best direction to explore
    **/
   enum CfDir GetBestDir();

   /*** 
    * @brief This function makes the drone moves forward
    * @param c_z_angle Angle at which the drone is.
    * @param dist if given a non-zero value, determines the distance
    * the drone is supposed to travel from is current location in a 
    * fixed amount of time.
   ***/
   void MoveForward(CRadians c_z_angle, float dist = 0);

   /*** 
    * @brief This function makes the drone moves to the left
    * @param c_z_angle Angle at which the drone is.
    * @param dist if given a non-zero value, determines the distance
    * the drone is supposed to travel from is current location in a 
    * fixed amount of time. 
   ***/
   void MoveLeft(CRadians c_z_angle, float dist = 0);

   /*** 
    * @brief This function makes the drone moves back
    * @param c_z_angle Angle at which the drone is. 
    * @param dist if given a non-zero value, determines the distance
    * the drone is supposed to travel from is current location in a 
    * fixed amount of time.
   ***/
   void MoveBack(CRadians c_z_angle, float dist = 0);

   /*** 
    * @brief This function makes the drone moves to the right
    * @param c_z_angle Angle at which the drone is.
    * @param dist if given a non-zero value, determines the distance
    * the drone is supposed to travel from is current location in a 
    * fixed amount of time.
   ***/
   void MoveRight(CRadians c_z_angle, float dist = 0);

   /** 
    * This function makes the drone stop moving
    * by setting the desired pos to the current pos 
   **/
   CVector3& StopMvmt();

   /**
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
   **/
   virtual void Reset();

   /**
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
   **/
   virtual void Destroy() {}
   
   /**
    * @brief Debug function. Write the content of LOG into a file.
   **/
   void printMap();

private:

   /* Pointer to the crazyflie distance sensor */
   CCI_CrazyflieDistanceScannerSensor* m_pcDistance;

   /* Pointer to the position actuator */
   CCI_QuadRotorPositionActuator* m_pcPropellers;
   
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;

   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;

   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPos;

   /* Pointer to the battery sensor */
   CCI_BatterySensor* m_pcBattery;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   /* Current and previous state of the drone */
   CfState m_cState;

   /* Current drone direction of mouvement */
   CfDir m_cDir;

   /* Current state of the battery */
   CCI_BatterySensor::SReading sBatRead;

   /**
   *  Current drone to object distance in cm
   *  - m_cDist[0]: front distance
   *  - m_cDist[1]: left distance
   *  - m_cDist[2]: back distance
   *  - m_cDist[3]: right distance
   **/
   Real m_cDist[4];
   
   /* Internal map of the arena */
   ExploreMap map;
};

#endif