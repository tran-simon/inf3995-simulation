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
    * This function checks the current distances between drones
    * and logs it.
    */
   virtual void CheckDronePosition();

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

   /***This function makes the drone moves forward
    * @param velocity Speed at which the drone moves.
   ***/
   void MoveFoward(float velocity);

   /***This function makes the drone moves to the left
    * @param velocity Speed at which the drone moves.
   ***/
   void MoveLeft(float velocity);

   /***This function makes the drone moves backwards
    * @param velocity Speed at which the drone moves. 
   ***/
   void MoveBack(float velocity);

   /***This function makes the drone moves to the right
    * @param velocity Speed at which the drone moves. 
   ***/
   void MoveRight(float velocity);

   /***This function makes the drone rotate
    * @param velocity Speed at which the drone moves. 
   ***/
   void Rotate(CRadians angle);

private:
   enum CfState {
      STATE_START,
      STATE_TAKE_OFF,
      STATE_EXPLORE,
      STATE_GO_TO_BASE,
      STATE_LAND
   };

   enum CfDir {
      FRONT,
      LEFT,
      BACK,
      RIGHT
   };

   enum CfExplorationState {
      FORWARD,
      WALL_END,
      ROTATE,
      DEBOUNCE
   };

   enum CfExplorationDir {
      LEFT_WALL,
      RIGHT_WALL
   };

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

   /*Current state of the drone*/
   CfState m_cState;

   /*Current state of the battery*/
   CCI_BatterySensor::SReading sBatRead;

   /*Current and previous mvmt of the drone*/
   CfDir m_cDir;
   CfDir m_pDir;
   CRadians m_desiredAngle;

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

   /*Robot exploration direction (left / right wall follower)*/
   CfExplorationDir m_CfExplorationDir;
   CfExplorationState m_CdExplorationState;
   Real previousDist;
   CVector3 previousPos;

};

#endif
