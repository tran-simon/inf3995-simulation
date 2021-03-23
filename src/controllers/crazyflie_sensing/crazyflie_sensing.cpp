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
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#define PORT 80
static bool waitingForStart = true;
static int firstTime = 0; 
static float velocity = 1;
static int fd[] = {-1,-1,-1,-1};


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
   
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   m_cState = CfState::STATE_START;
   m_CdExplorationState = CfExplorationState::FORWARD;
   m_CfExplorationDir = static_cast<CfExplorationDir>(stoi(GetId().substr(6))%2);
   previousDist = -2;
   previousPos = m_pcPos->GetReading().Position;
   m_uiCurrentStep = 0;
   isReturning = false;
   Reset();
}

/****************************************/
/*           Main loop function         */
/****************************************/
// https://www.geeksforgeeks.org/socket-programming-cc/
int CCrazyflieSensing::ConnectToSocket() {
   int sock, valread;
   struct sockaddr_in servaddr;
   int addrlen = sizeof(servaddr);
   int opt = 1;
   char mess[5];
   char buffer[1024] = {0}; 
   int port = PORT + stoi(GetId().substr(6));

   if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
      LOG << "socket creation failed.." << std::endl;
      return -1;
   }

   LOG << "Socket created succesfully" << std::endl;

   servaddr.sin_family = AF_INET;
   servaddr.sin_port = htons(port);

   if(inet_pton(AF_INET, "172.17.0.1", &servaddr.sin_addr)<=0)  
   { 
      LOG << "Invalid address/ Address not supported" << std::endl; 
      return -1; 
   } 
   
   if (connect(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) 
   { 
        LOG << "Connection Failed" << std::endl; 
        return -1; 
   }
   //sprintf(mess, "%d", sock);
   //send(sock, mess, strlen(mess), 0);
   //LOG << "Hello message sent" << std::endl; 
   valread = read(sock, buffer, 1024);
   
   if(valread > 0) {
      return sock;
   }
   else {
      return -1;
   }
}

char CCrazyflieSensing::ReadCommand(int fd) {
   char buffer[1024] = {0};
   int valRead = 0;
   int index = -1;

   valRead = recv(fd, buffer, sizeof(buffer), MSG_PEEK);
   LOG << fd << "MESSAGE RECEIVED : " << valRead << std::endl;

   for (int i = 0; i < sizeof(buffer); i++){
      if (buffer[i] != '\0') {
         index++;
      }
      else {
         break;
      }
   }

   if (index == -1) return 'f';

   LOG << "MESSAGE RECEIVED : " << buffer[index] << std::endl;
   return buffer[index];
}


void CCrazyflieSensing::CreateCommand(int fd, char* message, int value) {
   char markedBuffer[1024] = {0};
   switch (value) {
      case STATE:
         markedBuffer[0] = 's';
         break;
      case BATTERY:
         markedBuffer[0] = 'b';
         break;
      case VELOCITY:
         markedBuffer[0] = 'v';
         break;
      
   }
   strcat(markedBuffer, message);
   SendCommand(fd, markedBuffer);
}

int CCrazyflieSensing::SendCommand(int fd, char* message) {
   return send(fd, message, sizeof(message), 0);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::ControlStep() {
   char command;

   while (fd[stoi(GetId().substr(6))] <= 0) {
      fd[stoi(GetId().substr(6))] = ConnectToSocket();
   }
   LOG << fd[0] << " " << fd[1] << " " << fd[2] << " " << fd[3] << std::endl;
   if(fd[0] > 0 && fd[1] > 0 && fd[2] > 0 && fd[3] > 0 && m_cState == STATE_START) {
      TakeOff();
   }
   /*char stateBuffer[1024] = {0};
   stateBuffer[0] = '0' + m_cState;
   char batteryBuffer[1024] = {0};
   strcpy(batteryBuffer, std::to_string(sBatRead.AvailableCharge).c_str());
   char velocityBuffer[1024] = {0};
   strcpy(velocityBuffer, std::to_string(velocity).c_str());

   CreateCommand(fd[stoi(GetId().substr(6))], stateBuffer, STATE);
   CreateCommand(fd[stoi(GetId().substr(6))], batteryBuffer, BATTERY);
   CreateCommand(fd[stoi(GetId().substr(6))], velocityBuffer, VELOCITY);*/

   try {
      ++m_uiCurrentStep;
      LOG << "+====START====+" << std::endl;
      // Look battery level
      sBatRead = m_pcBattery->GetReading();
      LOG << "Battery level: " << sBatRead.AvailableCharge  << std::endl;

      CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead = m_pcDistance->GetReadingsMap();
      auto iterDistRead = sDistRead.begin();

      //char currentCommand = ReadCommand(fd[stoi(GetId().substr(6))]);
      //LOG << currentCommand << std::endl;
      // Check if drone are too close
      // CheckDronePosition();

      //if (currentCommand == 's') {
         //m_cState = STATE_TAKE_OFF;
      //}
      //else if (currentCommand == 'l' && !isReturning) { 
         //m_cState = STATE_GO_TO_BASE;
      //}

      if(sDistRead.size() == 4) {
         /*Updates of the distance sensor*/
         frontDist = (iterDistRead++)->second;
         leftDist = (iterDistRead++)->second;
         backDist = (iterDistRead++)->second;
         rightDist = (iterDistRead)->second;

         LOG << "Current State: " << m_cState << std::endl;
         /*States management*/
         switch (m_cState) {
            case STATE_START: break;
            case STATE_TAKE_OFF: TakeOff(); break;
            case STATE_EXPLORE: Explore(); break;
            case STATE_GO_TO_BASE: GoToBase(); break;
            case STATE_LAND: Land(); break;
            default: break;
         }
      }
      LOG << "+====END====+" << std::endl;
   } catch(std::exception e) {
      LOGERR << "AN EXCEPTION AS OCCURED IN CONTROLSTEP" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
   firstTime ++;
}

/****************************************/
/*       Mission States functions       */
/****************************************/

void CCrazyflieSensing::TakeOff() {
   try {
      CVector3 cPos = m_pcPos->GetReading().Position;
      Real height = 0.3 * stoi(GetId().substr(6)) + 0.3;
      if(m_cState != STATE_TAKE_OFF) {
         m_cState = STATE_TAKE_OFF;
         cPos.SetZ(height);
         m_pcPropellers->SetAbsolutePosition(cPos);
      } else if(Abs(cPos.GetZ() - height) < 0.1f) {
         m_cBasePos = m_pcPos->GetReading().Position;
         Explore();
      }
   } catch (std::exception e){
      LOGERR << "EXCEPTION AS OCCURED IN TAKEOFF" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::Explore() {
   try {
      if(m_cState != STATE_EXPLORE) {
         m_cState = STATE_EXPLORE;
      }

      if (sBatRead.AvailableCharge < 0.3
         && !isReturning 
         && m_CdExplorationState != CfExplorationState::ROTATE 
         && m_CdExplorationState != CfExplorationState::DEBOUNCE) { 
         GoToBase();
      }

      CRadians c_z_angle, c_y_angle, c_x_angle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);
      CVector3 cpos = m_pcPos->GetReading().Position;
      
      if (isReturning) {
         Real distToBase = Distance(cpos, m_cBasePos);
         if(distToBase < 0.5) {
            Land();
         }
      }

      switch(m_CdExplorationState) {
         case CfExplorationState::FORWARD: {
            // If the wall that the drone was following ended
            Real explorationDist = (m_CfExplorationDir == LEFT_WALL)? leftDist : rightDist;
            if((explorationDist == -2 || explorationDist > 1.5 * previousDist) && previousDist != -2) {
               m_CdExplorationState = WALL_END;
               break;
            }

            // If the drone is too close from a wall
            if(explorationDist < 30 && explorationDist != -2) {
               m_CdExplorationState = CfExplorationState::AVOID_WALL;
            }

            // If there is a wall in front of the drone
            if(frontDist < 50 && frontDist != -2) { 
               velocity =0;
               MoveForward(); // Stop any ongoing mvmt
               m_CdExplorationState = ROTATE;
               m_desiredAngle = (m_CfExplorationDir == LEFT_WALL)? -1 * CRadians::PI_OVER_TWO : CRadians::PI_OVER_TWO;
               m_desiredAngle += c_z_angle;
               break;
            }
            // Move the drone forward
            velocity = ((frontDist < 100 && frontDist != -2) ? (frontDist - 50.0) / 50.0 : 1.0) * 0.4;
            MoveForward();  
            break;
         }
         case CfExplorationState::WALL_END: {
            velocity = 0.1;
            MoveForward(); // Stop any ongoing mvmt
            m_CdExplorationState = ROTATE;
            m_desiredAngle = (m_CfExplorationDir == LEFT_WALL)? CRadians::PI_OVER_TWO : -1 * CRadians::PI_OVER_TWO;
            m_desiredAngle += c_z_angle;
            break;
         }
         case CfExplorationState::ROTATE: {
            if(Abs(c_z_angle - m_desiredAngle).UnsignedNormalize() < CRadians(0.01)) {
               m_CdExplorationState = DEBOUNCE;
               break;
            } 
            Rotate(m_desiredAngle);
            break;
         }
         case CfExplorationState::DEBOUNCE: {
            if(Abs(c_z_angle - m_desiredAngle).UnsignedNormalize() < CRadians(0.01)) {
               m_CdExplorationState = FORWARD;
            } else {
               m_CdExplorationState = ROTATE;
            }
            break;
         }
         case CfExplorationState::AVOID_WALL:
         {
            Real explorationDist = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL)? leftDist : rightDist;
            if(explorationDist > 40) {
               m_CdExplorationState = CfExplorationState::FORWARD;
               break;
            } else if (explorationDist == -2) {
               m_CdExplorationState = CfExplorationState::WALL_END;
               break;
            }

            if(m_CfExplorationDir == CfExplorationDir::LEFT_WALL) {
               m_pcPropellers->SetRelativePosition(CVector3(-0.1, 0, 0));
            } else {
               m_pcPropellers->SetRelativePosition(CVector3(0.1, 0, 0));
            }
            break;
         }
      }
      previousDist = (m_CfExplorationDir == LEFT_WALL)? leftDist : rightDist;
      previousPos = m_pcPos->GetReading().Position;
   } catch (std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED IN EXPLORATION" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::GoToBase() {
   try {
      m_CfExplorationDir = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL) ? CfExplorationDir::RIGHT_WALL : CfExplorationDir::LEFT_WALL;
      m_CdExplorationState = CfExplorationState::ROTATE;

      CRadians cZAngle, cYAngle, cXAngle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      m_desiredAngle = (cZAngle + CRadians::PI).UnsignedNormalize();
      isReturning = true;
   } catch(std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED ON THE WAY BACK" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::Land() {
   try {
      if(m_cState != STATE_LAND) {
         m_cState = STATE_LAND;
      }
      CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
      if((Abs(cPos.Position.GetZ()) > 0.01f)) {
         m_pcPropellers->SetAbsolutePosition(m_cBasePos);
         cPos.Position.SetZ(0.0f);
      } else {
         isReturning = false;
         LOG << "MISSION COMPLETE" << std::endl;
      }
   } catch(std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED WHILE LANDING" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
} 

/****************************************/
/*           Movements functions        */
/****************************************/

void CCrazyflieSensing::MoveForward() {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CRadians cZAngle, cYAngle, cXAngle;
   cpos.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   CVector3 desiredPos = cpos.Position + CVector3(velocity * Sin(cZAngle), -velocity*Cos(cZAngle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

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
