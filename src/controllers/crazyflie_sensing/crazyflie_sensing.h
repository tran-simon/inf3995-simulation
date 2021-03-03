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

   /**
    * Enum that represents cardinal direction relative to the drone orientation.
    * 
    * FRONT = 0
    * LEFT =  1
    * BACK =  2
    * RIGHT = 3
    * NONE =  4
   **/
   enum CfDir {
      FRONT,
      LEFT,
      BACK,
      RIGHT, 
      NONE
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

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function lifts the drone from the ground
    */
   void TakeOff();

   /*
    * This function makes the drone go on an adventure. Will he survive
    * the long and dangerous journey?
    */
   void Explore();

   /*
    * This function makes the drone go back to its take off position
    */
   void GoToBase();

   /*
    * This function returns the drone to the ground
    */
   void Land();

   /*
    * This function verifies that the drones
    * arent about to crash together and deviates the
    * drones according to the situation
    */
   virtual bool VerifieDroneProximity();

   /*
    * This function checks the current distances between drones
    * and logs it.
    */
   virtual void CheckDronePosition();

   /***
    * This function evaluates, based on the current position 
    * and environment of the drone, the cardinal direction 
    * that would shorten the distance between a point and  
    * the position of the drone the most.
    * @param destination A 3D vector that represents the 
    * destination point in space aimed by the drone.
    * @param pCheck If true, the environmental constraints such as
    * walls and drones proximity won't be taken into account.
    * @param possibilities Fixed length bool array that states 
    * if a given cardinal direction is a valid choice (1) or not (0).
    * The directions are  given be array indexes such that FRONT = 0,
    * LEFT = 1, BACK = 2 and RIGHT = 3.
    * @return The cardinal direction index as an integer.
   ***/
   CfDir GetBestDirection(const CVector3& destination, bool pCheck, bool possibilities[4]);
   
   /***
    * This function determines the opposite direction to the one given.
    * @param direction The direction of which we want to know the opposite of.
    * @return The opposite direction.
   ***/
   CfDir InvDirection(CfDir direction);

   /***
    * This function gives the number of directions the drone can not go to.
    * @param possibilities Fixed length bool array that states 
    * if a given cardinal direction is a valid choice (1) or not (0).
    * The directions are  given be array indexes such that FRONT = 0,
    * LEFT = 1, BACK = 2 and RIGHT = 3.
    ***/
   int CountObstructions(bool possibilities [4]);

   /**
    * This function checks the distance a drone has travelled in a fixed amount
    * of time. If the drone did not achieve a minimal distance delta, the corresponding
    * status value is modified. This function is used to check if a drone is stuck or 
    * unable to move for a long period of time.
    **/
   void VerifyMobility();

   /*** This function makes the drone moves forward
    * @param velocity Speed at which the drone moves.
   ***/
   void MoveForward(float velocity);

   /*** 
    * This function makes the drone moves to the left
    * @param velocity Speed at which the drone moves.
   ***/
   void MoveLeft(float velocity);

   /*** 
    * This function makes the drone moves backwards
    * @param velocity Speed at which the drone moves. 
   ***/
   void MoveBack(float velocity);

   /*** 
    * This function makes the drone moves to the right
    * @param velocity Speed at which the drone moves. 
   ***/
   void MoveRight(float velocity);

   /***
    * This function allows the drone to move in the 
    * desired direction at a given velocity.
    * @param direction gives the direction of movement 
    * @param velocity gives the velocity at which the
    * drone will go in the given direction 
   ***/
   void Move(CfDir direction, float velocity = 0.5);

   /*** 
    * This function makes the drone rotate to the 
    * desired angle relative to current orientation
    * @param angle Angle at which the drone will be
    * rotated. A value of PI/2 rotates 90 degrees to 
    * the left. A value of -PI/2 rotates 90 degrees
    * to the right. A small delay needs to be respected
    * for the drone to be in the right orientation
   ***/
   void RotateToward(CRadians angle);

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
   CfState m_pState;

   /*Current state of the battery*/
   CCI_BatterySensor::SReading sBatRead;

   /*Current and previous mvmt of the drone*/
   CfDir m_cDir;
   CfDir m_pDir;

   /*Current drone to object distance in the front direction*/
   Real frontDist;

   /*Current drone to object distance in the left direction*/
   Real leftDist;

   /*Current drone to object distance in the back direction*/
   Real backDist;

   /*Current drone to object distance in the right direction*/
   Real rightDist;

   /* Current step */
   uint m_uiCurrentStep;

   /*Steps relative to the current state*/
   uint m_uiMobilityStep;

   /*Previous position before mobility check*/
   CVector3 m_pPos;

   /*Mobility status*/
   bool isMobile;
};

#endif
