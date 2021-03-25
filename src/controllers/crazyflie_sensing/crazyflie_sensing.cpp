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
static bool waitingForStart = false;
static int fd = 0;
static int firstTime = 0; 
static float velocity = 0.2;

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
   m_uiCurrentStep = 0;
   isReturning = false;
   Reset();

   /* Initialise the current direction */
   m_cDir = CfDir::FRONT;

   /* Initialise the map */
   CVector3 cPos = m_pcPos->GetReading().Position;
   ExploreMapNew(&map);
   map.Construct(&map, (int) ((cPos.GetX() + 5) * 100), (int) ((cPos.GetY() + 5) * 100));
}

/****************************************/
/*           Main loop function         */
/****************************************/
// https://www.geeksforgeeks.org/socket-programming-cc/
int CCrazyflieSensing::ConnectToSocket() {
   int server_fd, new_socket, valread;
   struct sockaddr_in servaddr;
   int addrlen = sizeof(servaddr);
   int opt = 1;

   if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){
      LOG << "socket creation failed.." << std::endl;
      return 1000;
   }

   LOG << "Socket created succesfully" << std::endl;
   if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
         LOG << "socket failed forceful" << std::endl;
        return 1000;
   }

   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr = INADDR_ANY;
   servaddr.sin_port = htons(PORT);

   if (bind(server_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) <0) {
      LOG << "Binding error" << std::endl;
      return 1000;
   }

   LOG << "Socket binded succesfully" << std::endl;
   if (listen(server_fd, 2) < 0) {
      LOG << "Listening error" << std::endl;
      return 1000;
   }

   LOG << "Socket listening succesfully" << std::endl;
   if ((new_socket = accept(server_fd, (struct sockaddr *)&servaddr,(socklen_t*)&addrlen))<0) {
      LOG << "Accepting error : " << errno << std::endl;
      return 1000;
   }

   return new_socket;

}

char CCrazyflieSensing::ReadCommand(int fd) {
   char buffer[1024] = {0};
   int valRead = 0;
   int index = -1;

   valRead = recv(fd, buffer, sizeof(buffer), MSG_PEEK);

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

void CCrazyflieSensing::SendCommand(int fd, char* message) {
   send(fd, message, sizeof(message), 0);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::ControlStep() {
   /*char command;
   if(firstTime == 0){
      fd = ConnectToSocket();
      std::string id = "4";
      const char *buf = id.c_str();
      send(fd, buf, strlen(buf), 0);
   }

   while(waitingForStart){
      if(fd != 1000){
         command = ReadCommand(fd);
         if(command == 's'){
            waitingForStart = false; 
         }
      }
   }

   char stateBuffer[1024] = {0};
   stateBuffer[0] = '0' + m_cState;
   char batteryBuffer[1024] = {0};
   strcpy(batteryBuffer, std::to_string(sBatRead.AvailableCharge).c_str());
   char velocityBuffer[1024] = {0};
   strcpy(velocityBuffer, std::to_string(velocity).c_str());
   CreateCommand(fd, stateBuffer, STATE);
   CreateCommand(fd, batteryBuffer, BATTERY);
   CreateCommand(fd, velocityBuffer, VELOCITY);*/

   try {
      ++m_uiCurrentStep;
      LOG << "+====START====+" << std::endl;
      // Look battery level
      sBatRead = m_pcBattery->GetReading();
      //LOG << "Battery level: " << sBatRead.AvailableCharge  << std::endl;

      CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead = m_pcDistance->GetReadingsMap();
      auto iterDistRead = sDistRead.begin();

      // Check if drone are too close
      //CheckDronePosition();
      if ((sBatRead.AvailableCharge < 0.3 /*|| ReadCommand(fd) == 'l'*/)
         && !isReturning) { 
         m_cState = CfState::STATE_GO_TO_BASE;
      }

      if(sDistRead.size() == 4) {
         /*Updates of the distance sensor*/
         m_cDist[0] = (iterDistRead++)->second; /* Front distance */
         m_cDist[1] = (iterDistRead++)->second; /* left distance  */
         m_cDist[2] = (iterDistRead++)->second; /* back distance  */
         m_cDist[3] = (iterDistRead)->second;   /* right distance */

         //LOG << "Current State: " << m_cState << std::endl;
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

      CRadians c_z_angle, c_y_angle, c_x_angle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

      /* Move the drone in the map*/
      CVector3 cPos = m_pcPos->GetReading().Position;
      map.Move(&map, (int) ((cPos.GetX() + 5) * 100), (int) ((cPos.GetY() + 5) * 100));
      
      /* Add the sensor value to the map */
      map.AddData(&map,
                  static_cast<int>(m_cDist[0]), /* Front distance in cm */
                  static_cast<int>(m_cDist[1]), /* left distance  in cm */
                  static_cast<int>(m_cDist[2]), /* back distance  in cm */
                  static_cast<int>(m_cDist[3]));/* right distance in cm */
      
      /* If the drone is too close to an obstacle, move away */
      
      Real minimalDist = 30;
      if (m_cDist[0] < minimalDist && m_cDist[0] != -2) { MoveBack(c_z_angle); return;}
      if (m_cDist[1] < minimalDist && m_cDist[1] != -2) { MoveRight(c_z_angle); return;}
      if (m_cDist[2] < minimalDist && m_cDist[2] != -2) { MoveForward(c_z_angle); return;}
      if (m_cDist[3] < minimalDist && m_cDist[3] != -2) { MoveLeft(c_z_angle); return;}
      
      /* If there is an obstacle in the direction of the drone, choose another direction */
      LOG << "Curr dist: " << m_cDist[m_cDir] << std::endl;
      if (m_cDist[m_cDir] < 70 && m_cDist[m_cDir] != -2) {
         StopMvmt();
         MapExplorationDir mapExplorationDir = map.GetBestDir(&map);
         /* Convert the map direction to the simulated drone direction */
         switch (mapExplorationDir) {
            case MapExplorationDir::Y_NEG: m_cDir = CfDir::FRONT; LOG << "FRONT\n"; printMap(); break;
            case MapExplorationDir::X_POS: m_cDir = CfDir::LEFT; LOG << "LEFT\n"; printMap(); break;
            case MapExplorationDir::Y_POS: m_cDir = CfDir::BACK; LOG << "BACK\n"; printMap(); break;
            case MapExplorationDir::X_NEG: m_cDir = CfDir::RIGHT; LOG << "RIGHT\n"; printMap(); break;
         }
      }

      /* Move in the direction of exploration */
      switch (m_cDir) {
         case CfDir::FRONT: MoveForward(c_z_angle); break;
         case CfDir::LEFT: MoveLeft(c_z_angle); break;
         case CfDir::BACK: MoveBack(c_z_angle); break;
         case CfDir::RIGHT: MoveRight(c_z_angle); break;
      }
   } catch (std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED IN EXPLORATION" << std::endl;
      LOGERR <<"Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::printMap() {
   std::ofstream MyFile("map.txt");
   for (unsigned int i = 0; i < 50; i++) {
      for (unsigned int j = 0; j < 50; j++) {
         MyFile << map.map[i][j] << " ";
      }
      MyFile << std::endl;
   }

   MyFile.close();
}

void CCrazyflieSensing::GoToBase() {
   try {
      /*m_CfExplorationDir = (m_CfExplorationDir == CfExplorationDir::LEFT_WALL) ? CfExplorationDir::RIGHT_WALL : CfExplorationDir::LEFT_WALL;
      m_CdExplorationState = CfExplorationState::ROTATE;

      CRadians cZAngle, cYAngle, cXAngle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      m_desiredAngle = (cZAngle + CRadians::PI).UnsignedNormalize();
      isReturning = true;*/
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

void CCrazyflieSensing::MoveForward(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CVector3 desiredPos = cpos.Position + CVector3(param * Sin(c_z_angle), -param*Cos(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveLeft(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CVector3 desiredPos = cpos.Position + CVector3(param * Cos(c_z_angle), -param * Sin(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveBack(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CVector3 desiredPos = cpos.Position + CVector3(-param * Sin(c_z_angle), param * Cos(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::MoveRight(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   CVector3 desiredPos = cpos.Position + CVector3(-param * Cos(c_z_angle), param * Sin(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

/****************************************/
/****************************************/

void CCrazyflieSensing::StopMvmt() {
   CCI_PositioningSensor::SReading cpos = m_pcPos->GetReading();
   m_pcPropellers->SetAbsolutePosition(cpos.Position);
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
