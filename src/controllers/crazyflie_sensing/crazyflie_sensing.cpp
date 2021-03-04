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
   m_pDir = CfDir::FRONT;
   m_CdExplorationState = CfExplorationState::FORWARD;
   m_CfExplorationDir = static_cast<CfExplorationDir>(stoi(GetId().substr(6))%2);
   previousDist = -2;
   previousPos = m_pcPos->GetReading().Position;
   m_uiCurrentStep = 0;
   Reset();
}

/****************************************/
/****************************************/

void CCrazyflieSensing::ControlStep() {
   ++m_uiCurrentStep;

   // Look battery level
   sBatRead = m_pcBattery->GetReading();
   LOG << "Battery level: " << sBatRead.AvailableCharge  << std::endl;

   CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead = m_pcDistance->GetReadingsMap();
   auto iterDistRead = sDistRead.begin();

   if(sDistRead.size() == 4) {
      /*Updates of the distance sensor*/
      frontDist = (iterDistRead++)->second;
      leftDist = (iterDistRead++)->second;
      backDist = (iterDistRead++)->second;
      rightDist = (iterDistRead)->second;
      /*States management*/
      LOG << "State : " << m_cState;
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
      cPos.SetZ(1.0f);
      m_pcPropellers->SetAbsolutePosition(cPos);
   } else {
      cPos = m_pcPos->GetReading().Position;
      if(Abs(cPos.GetZ() - 1.0f) < 0.1f) {
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

   if (sBatRead.AvailableCharge < 0.3) { GoToBase();}

   CRadians c_z_angle, c_y_angle, c_x_angle;
   m_pcPos->GetReading().Orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

   switch(m_CdExplorationState) {
      case CfExplorationState::FORWARD: 
      {
         // If the wall that the drone was following ended
         Real explorationDist = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? leftDist : rightDist;
         LOG << "left " << leftDist << std::endl;
         LOG << "right " << rightDist << std::endl;
         LOG << "dist " << explorationDist << std::endl;
         if((explorationDist == -2 || explorationDist > 1.5 * previousDist) && previousDist != -2) {
            LOG << "THE SIDE : "<< explorationDist << std::endl;
            m_CdExplorationState = CfExplorationState::WALL_END;
            break;
         }
         
         // If there is a wall in front of the drone
         if (frontDist < 40 && frontDist != -2) { 
            LOG << "THE FRONT : "<< frontDist << std::endl;
            MoveFoward(0); // Stop any ongoing mvmt
            m_CdExplorationState = CfExplorationState::ROTATE;
            m_desiredAngle = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? -1 * CRadians::PI_OVER_TWO : CRadians::PI_OVER_TWO;
            m_desiredAngle += c_z_angle;
            break;
         }
         // Move the drone forward
         float velocity = (frontDist < 100 && frontDist != -2) ? (frontDist - 40.0) / 60.0 : 1.0;
         MoveFoward(velocity * 0.2);  

         /*
         // Correct the drone orientation if its path is not perpendicular to the wall
         TODO : put the cathetus and the hypothenuse in the same scale
         Real cathetus = previousDist;
         cathetus -= (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? leftDist : rightDist;
         Real hypotenuse = argos::SquareDistance(previousPos, m_pcPos->GetReading().Position);
         CRadians deviationAngle = CRadians(ASin(Abs(cathetus) / hypotenuse));
         Real orientationSign = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? 1.0 : -1.0;
         m_pcPropellers->SetRelativeYaw(deviationAngle * Sign(cathetus) * orientationSign);
         */

         break;
      }
      case CfExplorationState::WALL_END:
      {
         MoveFoward(0); // Stop any ongoing mvmt
         m_CdExplorationState = CfExplorationState::ROTATE;
         m_desiredAngle = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? CRadians::PI_OVER_TWO : -1 * CRadians::PI_OVER_TWO;
         m_desiredAngle += c_z_angle;
         break;
      }
      case CfExplorationState::ROTATE:
      {
         if (Abs(c_z_angle - m_desiredAngle).UnsignedNormalize() < CRadians(0.01)) {
            m_CdExplorationState = CfExplorationState::DEBOUNCE;
            break;
         } 
         Rotate(m_desiredAngle);
         break;
      }
      case CfExplorationState::DEBOUNCE:
      {
         if (Abs(c_z_angle - m_desiredAngle).UnsignedNormalize() < CRadians(0.01)) {
            m_CdExplorationState = CfExplorationState::FORWARD;
         } else {
            m_CdExplorationState = CfExplorationState::ROTATE;
         }
         break;
      }
   }

   previousDist = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? leftDist : rightDist;
   previousPos = m_pcPos->GetReading().Position;
   
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

void CCrazyflieSensing::Rotate(CRadians angle) {
   m_pcPropellers->SetAbsoluteYaw(angle);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::Reset() {
   m_cState = STATE_START;
}

/****************************************/
/****************************************/
void CCrazyflieSensing::CheckDronePosition() {
   for (int i = 0; i < m_pcRABS->GetReadings().size(); i++ ){
      Real droneDistance = m_pcRABS->GetReadings()[i].Range;
      if (droneDistance < 60){ // Figure out a good distance
         LOG << "DANGER TOO CLOSE!"<< std::endl; // Remplace par disperese function call
      }
   }
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
