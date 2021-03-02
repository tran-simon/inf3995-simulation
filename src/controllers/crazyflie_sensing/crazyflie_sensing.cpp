/* Include the controller definition */
#include "crazyflie_sensing.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#define PORT 80
static bool waitingForStart = true;

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
   m_uiCurrentStep = 0;
   Reset();
}

/****************************************/
/****************************************/
// https://www.geeksforgeeks.org/socket-programming-cc/
void CCrazyflieSensing::ConnectToSocket() {
   int server_fd, new_socket, valread;
   struct sockaddr_in servaddr;
   int addrlen = sizeof(servaddr);
   int opt = 1;
   
   if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
      LOG << "socket creation failed.." << std::endl;
      return;
   }
   LOG << "Socket created succesfully" << std::endl;
   if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
         LOG << "socket failed forceful" << std::endl;
        return;
   }
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr = INADDR_ANY;
   servaddr.sin_port = htons(PORT);

   if (bind(server_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) <0) {
      LOG << "Binding error" << std::endl;
      return;
   }

   LOG << "Socket binded succesfully" << std::endl;
   if (listen(server_fd, 2) < 0) {
      LOG << "Listening error" << std::endl;
      return;
   }

   LOG << "Socket listening succesfully" << std::endl;
   if ((new_socket = accept(server_fd, (struct sockaddr *)&servaddr,(socklen_t*)&addrlen))<0) {
      LOG << "Accepting error" << std::endl;
      return;
   }

   char buffer[1024] = {0};
   while(true){
      valread = recv( new_socket , buffer, 1, 0);
      if( memcmp(buffer, "s", strlen("s")) == 0){
         send(new_socket, "Started Simulation successfully", strlen("Started Simulation successfully"), 0);
         close(new_socket);
         return;
      }
      if ((new_socket = accept(server_fd, (struct sockaddr *)&servaddr,(socklen_t*)&addrlen))<0) {
         LOG << "Accepting error" << std::endl;
         return;
      }
   }
   
}

/****************************************/
/****************************************/

void CCrazyflieSensing::ControlStep() {
   while(waitingForStart){
      ConnectToSocket();
      waitingForStart = false;
   }
   int firstime =0; 
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
   firstime ++;
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

   if (sBatRead.AvailableCharge < 0.3) { GoToBase();}

   VerifieDroneProximity();
   
   if (m_cDir == CfDir::FRONT && frontDist < 30 && frontDist != -2) { m_cDir = CfDir::LEFT; m_pDir = CfDir::FRONT;}
   if (m_cDir == CfDir::LEFT && leftDist < 30 && leftDist != -2) { m_cDir = CfDir::BACK; m_pDir = CfDir::LEFT;}
   if (m_cDir == CfDir::BACK && backDist < 30 && backDist != -2) { m_cDir = CfDir::RIGHT; m_pDir = CfDir::BACK;}
   if (m_cDir == CfDir::RIGHT && rightDist < 30 && rightDist != -2) { m_cDir = CfDir::FRONT; m_pDir = CfDir::RIGHT;}

   if (m_cDir == CfDir::LEFT && m_pDir == CfDir::FRONT && (frontDist > 30 || frontDist == -2)) { m_cDir = CfDir::FRONT;}
   if (m_cDir == CfDir::BACK && m_pDir == CfDir::LEFT && (leftDist > 30 || leftDist == -2)) { m_cDir = CfDir::LEFT;}
   if (m_cDir == CfDir::RIGHT && m_pDir == CfDir::BACK && (backDist > 30 || backDist == -2)) { m_cDir = CfDir::BACK;}
   if (m_cDir == CfDir::FRONT && m_pDir == CfDir::RIGHT && (rightDist > 30 || rightDist == -2)) { m_cDir = CfDir::RIGHT;}
    LOG<<"Current Dir : "<<m_cDir << std::endl;
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

void CCrazyflieSensing::VerifieDroneProximity() {
   if (m_pcRABS->GetReadings()[0].Range < 30){
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
