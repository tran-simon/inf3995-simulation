/* Include the controller definition */
#include "crazyflie_sensing.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* 3D vector definition */
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
   m_uiMobilityStep = 0;
   isMobile = true;
   Reset();
}

/****************************************/
/*           Main loop function         */
/****************************************/

void CCrazyflieSensing::ControlStep() {
   ++m_uiCurrentStep;
   LOG << "+====START====+" << std::endl;
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

      LOG << "Current State: " << m_cState << std::endl;
      /*States management*/
      switch (m_cState) {
         case STATE_START: TakeOff();
         case STATE_TAKE_OFF: TakeOff(); break;
         case STATE_EXPLORE: Explore(); break;
         case STATE_GO_TO_BASE: GoToBase(); break;
         case STATE_LAND: Land(); break;
         default: break;
      }
   }
   LOG << "+====END====+" << std::endl;
}

/****************************************/
/*       Mission States functions       */
/****************************************/

void CCrazyflieSensing::TakeOff() {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   if(m_cState != STATE_TAKE_OFF) {
      m_cState = STATE_TAKE_OFF;
      cPos.Position.SetZ(1.5f);
      m_pcPropellers->SetAbsolutePosition(cPos.Position);
   } else if(Abs(cPos.Position.GetZ() - 1.5f) < 0.1f) {
         m_cBasePos = cPos.Position;
         Explore();
   }
}

void CCrazyflieSensing::Explore() {
   if(m_cState != STATE_EXPLORE) {
      m_cState = STATE_EXPLORE;
   }
   if (sBatRead.AvailableCharge < 0.83) { GoToBase();}
   if(!VerifieDroneProximity()) {
      if (m_cDir == FRONT && frontDist < DETECTION_THRESHOLD && frontDist != -2) { m_cDir = LEFT; m_pDir = FRONT;}
      if (m_cDir == LEFT && leftDist < DETECTION_THRESHOLD   && leftDist != -2 ) { m_cDir = BACK; m_pDir = LEFT;}
      if (m_cDir == BACK && backDist < DETECTION_THRESHOLD   && backDist != -2 ) { m_cDir = RIGHT; m_pDir = BACK;}
      if (m_cDir == RIGHT && rightDist < DETECTION_THRESHOLD && rightDist != -2) { m_cDir = FRONT; m_pDir = RIGHT;}

      if (m_cDir == LEFT && m_pDir == FRONT && (frontDist > DETECTION_THRESHOLD || frontDist == -2)) { m_cDir = FRONT;}
      if (m_cDir == BACK && m_pDir == LEFT && (leftDist > DETECTION_THRESHOLD || leftDist == -2 )) { m_cDir = LEFT;}
      if (m_cDir == RIGHT && m_pDir == BACK && (backDist > DETECTION_THRESHOLD || backDist == -2)) { m_cDir = BACK;}
      if (m_cDir == FRONT && m_pDir == RIGHT && (rightDist > DETECTION_THRESHOLD || rightDist == -2)) { m_cDir = RIGHT;}
   }
   Move(m_cDir);
}

void CCrazyflieSensing::GoToBase() {
   if(m_cState != STATE_GO_TO_BASE) {
      m_cState = STATE_GO_TO_BASE;
   }

   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();

   LOG << "X : " << cPos.Position.GetX() << std::endl;
   LOG << "Y : " << cPos.Position.GetY() << std::endl;

   if(Distance(m_cBasePos, cPos.Position) < 0.05f) {
      Land();
   }

   /*We define condition of valid direction*/
   bool possibilities[4] = {(frontDist >= DETECTION_THRESHOLD || frontDist == -2),
                            (leftDist  >= DETECTION_THRESHOLD || leftDist  == -2),
                            (backDist  >= DETECTION_THRESHOLD || backDist  == -2),
                            (rightDist >= DETECTION_THRESHOLD || rightDist == -2)};

   VerifyMobility();
   if(!isMobile && !VerifieDroneProximity()) {
      if (m_cDir == FRONT && !possibilities[FRONT]) { m_cDir = LEFT;  m_pDir = FRONT;}
      if (m_cDir == LEFT  && !possibilities[LEFT])  { m_cDir = BACK;  m_pDir = LEFT; }
      if (m_cDir == BACK  && !possibilities[BACK])  { m_cDir = RIGHT; m_pDir = BACK; }
      if (m_cDir == RIGHT && !possibilities[RIGHT]) { m_cDir = FRONT; m_pDir = RIGHT;}
      
      if (m_cDir == LEFT  && m_pDir == FRONT && possibilities[FRONT]) { m_cDir = FRONT;}
      if (m_cDir == BACK  && m_pDir == LEFT  && possibilities[LEFT])  { m_cDir = LEFT; }
      if (m_cDir == RIGHT && m_pDir == BACK  && possibilities[BACK])  { m_cDir = BACK; }
      if (m_cDir == FRONT && m_pDir == RIGHT && possibilities[RIGHT]) { m_cDir = RIGHT;}

   } else {
      m_cDir = GetBestDirection(m_cBasePos, false, possibilities);
      m_pDir = m_cDir;
   }
   Move(m_cDir);
}

void CCrazyflieSensing::Land() {
   if(m_cState != STATE_LAND) {
      m_cState = STATE_LAND;
   }
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   if((Abs(cPos.Position.GetZ()) > 0.01f)) {
      m_pcPropellers->SetAbsolutePosition(m_cBasePos);
      cPos.Position.SetZ(0.0f);
   } else {
      LOG << "MISSION COMPLETE" << std::endl;
   }
} 

/****************************************/
/*          Utilities functions         */
/****************************************/

bool CCrazyflieSensing::VerifieDroneProximity() {
   bool state = false;
   if (m_pcRABS->GetReadings()[0].Range < 45){
      int positionInDegrees = (int)(m_pcRABS->GetReadings()[0].HorizontalBearing.GetValue() * CRadians::RADIANS_TO_DEGREES);
      if ( positionInDegrees >= 0 && positionInDegrees < 90){ //Drone is located between 0 and PI/2
            m_cDir = RIGHT;
            LOG << "MOVING AWAY, GOING RIGHT" << std::endl;
      } else if(positionInDegrees >= 90 && positionInDegrees <= 180){ //Drone is located between PI/2 and PI
            m_cDir = LEFT;
            LOG << "MOVING AWAY, GOING LEFT" << std::endl;
      } else if(positionInDegrees <= -1 && positionInDegrees >= -90){ // Drone is located between 0 and 3PI/2
            m_cDir = BACK;
            LOG << "MOVING AWAY, GOING BACK" << std::endl;
      } else if(positionInDegrees < -90 && positionInDegrees > -180){ //Drone is located between 3PI/2 and PI
            m_cDir = BACK;
            LOG << "MOVING AWAY, GOING BACK" << std::endl;
      }
      state = true;
   }
   return state; 
}

void CCrazyflieSensing::CheckDronePosition() {
   for (int i = 0; i < m_pcRABS->GetReadings().size(); i++ ){
      Real droneDistance = m_pcRABS->GetReadings()[i].Range;
      if (droneDistance < 60){ // Figure out a good distance
         LOG << "DANGER TOO CLOSE!"<< std::endl; // Remplace par disperese function call
      }
   }
}

CCrazyflieSensing::CfDir CCrazyflieSensing::GetBestDirection(const CVector3& destination, bool pCheck, bool possibilities[4]) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

   /*Distance between current drone location and the base*/
   Real distToBase = Distance(cPos.Position, destination);
   
   /*Best cardinal direction for the drone to go*/
   CfDir bestDir = NONE;
   
   /*Smallest distance to the goal position*/
   Real bestDist = -1;
   for(int i = 0; i < 4; i++) {
      Real cX, cY, predictDist;

      /*If checkPossibilities is true, then every direction is evaluated*/
      if(possibilities[i] || pCheck) {
         switch (i) {
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
         predictDist = Distance(cPos.Position + CVector3(cX, cY, cPos.Position.GetZ()), destination);
         if(bestDist == -1 || bestDist > predictDist) {
            bestDist = predictDist;
            bestDir = (CfDir)i;
         }
      }
   }
   return bestDir;
}

CCrazyflieSensing::CfDir CCrazyflieSensing::InvDirection(CfDir direction) {
   CfDir inverse = NONE;
   switch (direction) {
      case FRONT: inverse = BACK;  break;
      case LEFT:  inverse = RIGHT; break;
      case BACK:  inverse = FRONT; break;
      case RIGHT: inverse = LEFT;  break;
      case NONE: break;
      default: break;
   }
   return inverse;
}

int CCrazyflieSensing::CountObstructions(bool possibilities [4]){
   int obstructionsCount = 0;
   for(int i = 0; i < 4; i++) {
      if(!possibilities[i]) {
         obstructionsCount++;
      }
   }
   return obstructionsCount;
}

void CCrazyflieSensing::VerifyMobility() {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   if(m_uiMobilityStep == 0) {
      m_pPos = cPos.Position;
   }
   m_uiMobilityStep++;
   if(m_uiMobilityStep >= MOBILITY_DELAY) {
      Real currentDelta = Distance(m_pPos, cPos.Position);
      Real distFromBase = Distance(m_cBasePos, cPos.Position);
      isMobile = !(currentDelta < 0.8f && Abs(distFromBase) > 0.5);
      m_uiMobilityStep = 0;

      LOG << "Delta: " << currentDelta << std::endl;
   }
}

/****************************************/
/*           Movements functions        */
/****************************************/

void CCrazyflieSensing::MoveForward(float velocity) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cPos.Position+ CVector3(velocity * Sin(cZAngle), -velocity*Cos(cZAngle), cPos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveLeft(float velocity) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cPos.Position + CVector3(velocity * Cos(cZAngle), -velocity*Sin(cZAngle), cPos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveBack(float velocity) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cPos.Position + CVector3(-velocity*Sin(cZAngle), velocity*Cos(cZAngle), cPos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveRight(float velocity) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cPos.Position + CVector3(-velocity * Cos(cZAngle), velocity*Sin(cZAngle), cPos.Position.GetZ());
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::Move(CfDir direction, float velocity) {
   LOG << "Current Dir : " << direction << std::endl;
      switch(direction) {
         case FRONT: MoveForward(velocity); break;
         case LEFT:  MoveLeft(velocity);   break;
         case BACK:  MoveBack(velocity);   break;
         case RIGHT: MoveRight(velocity);  break;
         case NONE: 
            LOG << "Didn't move" << std::endl;
            break;
         default: break;
      }
}

void CCrazyflieSensing::RotateToward(CRadians angle) {
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cPos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   m_pcPropellers->SetAbsoluteYaw(cZAngle + angle);
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
