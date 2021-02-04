/* Include the controller definition */
#include "crazyflie_sensing.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CCrazyflieSensing::CCrazyflieSensing() :
   m_pcDistance(NULL),
   m_pcPropellers(NULL),
   m_pcRNG(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcPos(NULL),
   m_pcBattery(NULL),
   m_uiCurrentStep(0) {}

/****************************************/
/****************************************/

void CCrazyflieSensing::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcDistance   = GetSensor  <CCI_CrazyflieDistanceScannerSensor>("crazyflie_distance_scanner");
      m_pcPropellers = GetActuator  <CCI_QuadRotorPositionActuator>("quadrotor_position");
      /* Get pointers to devices */
      m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
      m_pcRABS   = GetSensor  <CCI_RangeAndBearingSensor  >("range_and_bearing");
      try {
         m_pcPos = GetSensor  <CCI_PositioningSensor>("positioning");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcBattery = GetSensor<CCI_BatterySensor>("battery");
      }
      catch(CARGoSException& ex) {}
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the crazyflie sensing controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   m_cState = STATE_START;
   m_cDir = CfDir::FRONT;
   m_uiCurrentStep = 0;
   Reset();
}

/****************************************/
/****************************************/

void CCrazyflieSensing::ControlStep() {
   ++m_uiCurrentStep;

   // Look battery level
   const CCI_BatterySensor::SReading& sBatRead = m_pcBattery->GetReading();
   LOG << "Battery level: " << sBatRead.AvailableCharge  << std::endl;

   CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead = m_pcDistance->GetReadingsMap();
   auto iterDistRead = sDistRead.begin();

   if(sDistRead.size() == 4) {
      /*Updates of the distance sensor*/
      frontDist = (iterDistRead++)->second;
      leftDist = (iterDistRead++)->second;
      backDist = (iterDistRead++)->second;
      rightDist = (iterDistRead)->second;
      LOG <<"State: " << m_cState;
      /*States management*/
      switch (m_cState)
      {
         case STATE_START:
            TakeOff();
         case STATE_TAKE_OFF:
            TakeOff();
            break;
         case STATE_EXPLORE:
            Explore();
            break;
         case STATE_GO_TO_BASE:
            GoToBase();
            break;
         case STATE_LAND:
            Land();
            break;
            
         default:
            break;
      }
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::TakeOff() {
   CVector3 cPos;
   if(m_cState != STATE_TAKE_OFF) {
      m_cState = STATE_TAKE_OFF;
      cPos = m_pcPos->GetReading().Position;
      cPos.SetZ(1.5f);
      m_pcPropellers->SetAbsolutePosition(cPos);
   } else {
      cPos = m_pcPos->GetReading().Position;
      if(Abs(cPos.GetZ() - 1.5f) < 0.1f) {
         m_cBasePos = m_pcPos->GetReading().Position;
         Explore();
      }
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::Explore() {
   if(m_cState != STATE_EXPLORE) {
      m_cState = STATE_EXPLORE;
   }

   if (m_uiCurrentStep > 250) { GoToBase();}
   
   if (m_cDir == CfDir::FRONT && frontDist < 30 && frontDist != -2) { m_cDir = CfDir::LEFT;}
   if (m_cDir == CfDir::LEFT && leftDist < 30 && leftDist != -2) { m_cDir = CfDir::BACK;}
   if (m_cDir == CfDir::BACK && backDist < 30 && backDist != -2) { m_cDir = CfDir::RIGHT;}
   if (m_cDir == CfDir::RIGHT && rightDist < 30 && rightDist != -2) { m_cDir = CfDir::FRONT;}

   switch(m_cDir) {
      case CfDir::FRONT: MoveFoward(1); break;
      case CfDir::LEFT: MoveLeft(1); break;
      case CfDir::BACK: MoveBack(1); break;
      case CfDir::RIGHT: MoveRight(1); break;
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::GoToBase() {
   if(m_cState != STATE_GO_TO_BASE) {
      m_cState = STATE_GO_TO_BASE;
   }

   if(argos::SquareDistance(m_cBasePos, m_pcPos->GetReading().Position) < 0.01f) {
      Land();
   } else {
      m_pcPropellers->SetAbsolutePosition(m_cBasePos);
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::Land() {
   if(m_cState != STATE_LAND) {
      m_cState = STATE_LAND;
      CVector3 cPos = m_pcPos->GetReading().Position;
      if(!(Abs(cPos.GetZ()) < 0.01f)) {
         cPos.SetZ(0.0f);
         m_pcPropellers->SetAbsolutePosition(cPos);
      }
   } else {
      LOG << "MISSION COMPLETE";
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveFoward(float velocity) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cpos.Position + CVector3(velocity * Sin(cZAngle), -velocity*Cos(cZAngle), cpos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveLeft(float velocity) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cpos.Position + CVector3(velocity * Cos(cZAngle), -velocity*Sin(cZAngle), cpos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveBack(float velocity) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cpos.Position + CVector3(-velocity*Sin(cZAngle), velocity*Cos(cZAngle), cpos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveRight(float velocity) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cpos.Position + CVector3(-velocity * Cos(cZAngle), velocity*Sin(cZAngle), cpos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::Reset() {
   m_cState = STATE_START;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
