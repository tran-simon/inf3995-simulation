/* Include the controller definition */
#include "crazyflie_sensing.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
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
   m_pState = STATE_START;
   m_cDir = FRONT;
   m_pDir = FRONT;
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
      LOG <<"Current State: " << m_cState << std::endl;
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
         m_pState = STATE_TAKE_OFF;
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

   if (sBatRead.AvailableCharge < 0.85) { GoToBase();}
      
   VerifieDroneProximity();
   
   if (m_cDir == FRONT && frontDist < DETECTION_THRESHOLD && frontDist != -2) { m_cDir = LEFT; m_pDir = FRONT;}
   if (m_cDir == LEFT && leftDist < DETECTION_THRESHOLD && leftDist != -2) { m_cDir = BACK; m_pDir = LEFT;}
   if (m_cDir == BACK && backDist < DETECTION_THRESHOLD && backDist != -2) { m_cDir = RIGHT; m_pDir = BACK;}
   if (m_cDir == RIGHT && rightDist < DETECTION_THRESHOLD && rightDist != -2) { m_cDir = FRONT; m_pDir = RIGHT;}

   if (m_cDir == LEFT && m_pDir == FRONT && (frontDist > DETECTION_THRESHOLD || frontDist == -2)) { m_cDir = FRONT;}
   if (m_cDir == BACK && m_pDir == LEFT && (leftDist > DETECTION_THRESHOLD || leftDist == -2)) { m_cDir = LEFT;}
   if (m_cDir == RIGHT && m_pDir == BACK && (backDist > DETECTION_THRESHOLD || backDist == -2)) { m_cDir = BACK;}
   if (m_cDir == FRONT && m_pDir == RIGHT && (rightDist > DETECTION_THRESHOLD || rightDist == -2)) { m_cDir = RIGHT;}

   LOG<<"Current Dir : "<< m_cDir << std::endl;
   switch(m_cDir) {
      case FRONT: MoveFoward(1); break;
      case LEFT: MoveLeft(1); break;
      case BACK: MoveBack(1); break;
      case RIGHT: MoveRight(1); break;
   }
}

int CCrazyflieSensing::GetBestDirection(bool possibilities[4], const CVector3& destination) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   Real distToBase = SquareDistance(cpos.Position, destination);
   
   /*Best cardinal direction for the drone to go*/
   CfDir bestDir = NONE;
   
   /*Smallest distance to the goal position*/
   Real bestDist = -1;

   for(int i = 0; i < 4; i++) {
      Real cX, cY, predictDist;
      if(possibilities[i]) {
         switch (i)
         {
            case FRONT: 
               cX = Sin(cZAngle);
               cY = -Cos(cZAngle);
               break;
            case LEFT:
               cX = Cos(cZAngle);
               cY = -Sin(cZAngle);
               break;
            case BACK:
               cX = -Sin(cZAngle);
               cY = Cos(cZAngle);
               break;
            case RIGHT:
               cX = -Cos(cZAngle);
               cY = Sin(cZAngle);
               break;
            default:
               break;
         }
         predictDist = SquareDistance(cpos.Position + CVector3(cX, cY, cpos.Position.GetZ()), destination);
         if(bestDist == -1 || bestDist > predictDist) {
            bestDist = predictDist;
            bestDir = (CfDir)i;
         }
      }
   }
   return bestDir;
}

/****************************************/
/****************************************/

void CCrazyflieSensing::GoToBase() {
   if(m_cState != STATE_GO_TO_BASE) {
      m_cState = STATE_GO_TO_BASE;
   }
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   VerifieDroneProximity();
   if(SquareDistance(m_cBasePos, cpos.Position) < 0.01f) {
      Land();
   } else {
      LOG << "X : " << cpos.Position.GetX() << std::endl;
      LOG << "Y : " << cpos.Position.GetY() << std::endl;

      /*We define condition of valid direction*/
      /*TODO: drone collision*/
      bool possibilities[4] = {(frontDist >= DETECTION_THRESHOLD || frontDist == -2),
                               (leftDist >= DETECTION_THRESHOLD || leftDist == -2),
                               (backDist >= DETECTION_THRESHOLD || backDist == -2),
                               (rightDist >= DETECTION_THRESHOLD || rightDist == -2)};
      m_cDir = (CfDir)GetBestDirection(possibilities, m_cBasePos);
      LOG<<"Current Dir : "<< m_cDir << std::endl;
      switch(m_cDir) {
         case FRONT: MoveFoward(1); break;
         case LEFT: MoveLeft(1); break;
         case BACK: MoveBack(1); break;
         case RIGHT: MoveRight(1); break;
         case NONE: LOGERR << "Could not move on this step" << std::endl; break;
      }
   }
}

/****************************************/
/****************************************/

void CCrazyflieSensing::VerifieDroneProximity() {
   if (m_pcRABS->GetReadings()[0].Range < DETECTION_THRESHOLD){
      int positionInDegrees = (int)(m_pcRABS->GetReadings()[0].HorizontalBearing.GetValue() * CRadians::RADIANS_TO_DEGREES);

      if ( positionInDegrees >= 0 && positionInDegrees < 90){ //Drone is located between 0 and PI/2
            m_cDir = CfDir::RIGHT;
            LOG << "MOVING AWAY, GOING RIGHT" << std::endl;
      } else if(positionInDegrees >= 90 && positionInDegrees <= 180){ //Drone is located between PI/2 and PI
            m_cDir = CfDir::LEFT;
            LOG << "MOVING AWAY, GOING LEFT" << std::endl;
      } else if(positionInDegrees <= -1 && positionInDegrees >= -90){ // Drone is located between 0 and 3PI/2
            m_cDir = CfDir::BACK;
            LOG << "MOVING AWAY, GOING BACK" << std::endl;
      } else if(positionInDegrees < -90 && positionInDegrees > -180){ //Drone is located between 3PI/2 and PI
            m_cDir = CfDir::BACK;
            LOG << "MOVING AWAY, GOING BACK" << std::endl;
      }
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

void CCrazyflieSensing::RotateToward(CRadians angle) {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   m_pcPropellers->SetAbsoluteYaw(cZAngle + angle);
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
void CCrazyflieSensing::MoveForward(float step) {
   CCI_PositioningSensor::SReading positionRead = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   positionRead.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = positionRead.Position + CVector3(step * Sin(cZAngle), -step * Cos(cZAngle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}
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
