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

/*
 * Include some necessary headers.
 */
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
#include <iostream>
#include <fstream>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CCrazyflieSensing : public CCI_Controller {
public:
   /**
    * Enum that represents mission states.
    * 
    * STATE_START      = 0
    * STATE_TAKE_OFF   = 1
    * STATE_EXPLORE    = 2
    * STATE_GO_TO_BASE = 3
    * STATE_LAND       = 4
   **/
   enum CfState {
      STATE_START,
      STATE_TAKE_OFF,
      STATE_EXPLORE,
      STATE_GO_TO_BASE,
      STATE_LAND
   };

   enum CfValue {
      STATE,
      BATTERY,
      VELOCITY
   };

   enum CfDir {
      FRONT,
      LEFT,
      BACK,
      RIGHT
   };

   /* Gives the distance at which drones returns -2 as mesured distance */
   static uint const MAX_VIEW_DIST = 200;

   /* Arbitrary distance value from which the drone can 
      no longer approach an object*/
   static uint const DETECTION_THRESHOLD = 45U;

   /* Arbitrary number of steps between each mobility evaluation */
   static uint const MOBILITY_DELAY = 30U;

   /* Class constructor. */
   CCrazyflieSensing();

   /* Class destructor. */
   virtual ~CCrazyflieSensing() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /***This function connects to the socket running
    * on the backend in order to communicate when
    * to start and land.
   ***/
   int ConnectToSocket();

   char ReadCommand(int fd);

   void CreateCommand(int fd, char* message, int value);

   void SendCommand(int fd, char* message);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /****************************************/
   /*       Mission States functions       */
   /****************************************/
   
   void TakeOff();
   void Explore();
   void GoToBase();
   void Land();
   
   
   /*
    * This function lifts the drone from the ground

    This function makes the drone go on an adventure. Will he survive
    * the long and dangerous journey?

    This function makes the drone go back to its take off position

    This function returns the drone to the ground
    */   

   /*** 
    * This function makes the drone moves forward
    * @param c_z_angle Angle at which the drone is. 
   ***/
   void MoveForward(CRadians c_z_angle, float dist = 0);

   /*** 
    * This function makes the drone moves to the left
    * @param c_z_angle Angle at which the drone is. 
   ***/
   void MoveLeft(CRadians c_z_angle, float dist = 0);

   /*** 
    * This function makes the drone moves back
    * @param c_z_angle Angle at which the drone is. 
   ***/
   void MoveBack(CRadians c_z_angle, float dist = 0);

   /*** 
    * This function makes the drone moves to the right
    * @param c_z_angle Angle at which the drone is. 
   ***/
   void MoveRight(CRadians c_z_angle, float dist = 0);

   /*** 
    * This function makes the drone stop moving
    * by setting the desired pos to the current pos 
   ***/
   void StopMvmt();

   /*** 
    * This function makes the drone rotate
    * @param angle Angle at which the drone rotates. 
   ***/
   void Rotate(CRadians angle);

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

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

   /* Base position on take off */
   CVector3 m_cBasePos;

   /*Current and previous state of the drone*/
   CfState m_cState;

   /*Current state of the battery*/
   CCI_BatterySensor::SReading sBatRead;
   
   CRadians m_desiredAngle;

   /***
   *  Current drone to object distance
   *  m_cDist[0]: front distance
   *  m_cDist[1]: left distance
   *  m_cDist[2]: back distance
   *  m_cDist[3]: right distance
   ***/
   Real m_cDist[4];
   
   Real frontDist;

   /*Current drone to object distance in the left direction*/
   Real leftDist;

   /*Current drone to object distance in the back direction*/
   Real backDist;

   /*Current drone to object distance in the right direction*/
   Real rightDist;

   /* Current step */
   uint m_uiCurrentStep;

   bool isReturning; 

   CfDir m_cDir;
   ExploreMap map;

};

#endif
