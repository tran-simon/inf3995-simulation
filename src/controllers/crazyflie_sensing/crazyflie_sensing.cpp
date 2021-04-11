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
#include <cstdlib>

#define PORT 80

static bool waitingForStart = true;
static int firstTime = 0; 
static float velocity = 0.5;
static int fd[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
static float posX[10];
static float posY[10];

/****************************************/
/****************************************/

CCrazyflieSensing::CCrazyflieSensing() :
   m_pcDistance(NULL),
   m_pcPropellers(NULL),
   m_pcRNG(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcPos(NULL),
   m_pcBattery(NULL) {}

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
   Reset();

   /* Initialise the map */
   CVector3 cPos = m_pcPos->GetReading().Position;
   ExploreMapNew(&map);
   map.Construct(&map, (int) ((cPos.GetX() + 5) * 100), (int) ((cPos.GetY() + 5) * 100));
   printMap();

   /* Initialise the current direction */
   m_cDir = GetBestDir();
}

/****************************************/
/*        Connectivity functions        */
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

   if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      LOG << "socket creation failed.." << std::endl;
      return -1;
   }

   servaddr.sin_family = AF_INET;
   servaddr.sin_port = htons(port);

   if (inet_pton(AF_INET, "172.17.0.1", &servaddr.sin_addr) <= 0) { 
      LOG << "Invalid address/ Address not supported" << std::endl; 
      return -1; 
   } 
   
   if (connect(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { 
      LOG << "Connection Failed" << std::endl; 
      return -1; 
   }
   //sprintf(mess, "%d", sock);
   //send(sock, mess, strlen(mess), 0);
   //LOG << "Hello message sent" << std::endl; 
   valread = recv(sock, buffer, 1024, MSG_PEEK);
   return (valread > 0) ? sock:-1;
}

char CCrazyflieSensing::ReadCommand(int sock) {
   char buffer[1024] = {0};
   int valRead = 0;
   int index = -1;

   valRead = recv(sock, buffer, sizeof(buffer), MSG_PEEK);

   for (int i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] != '\0') {
         index++;
      } else break;
   }
   if (index == -1) return 'f';
   return buffer[index];
}


void CCrazyflieSensing::CreateCommand(int sock, char* message, int value) {
   char markedBuffer[1024] = {0};
   switch (value) {
      case STATE:   markedBuffer[0] = 's'; break;
      case BATTERY: markedBuffer[0] = 'b'; break;
      case VELOCITY:markedBuffer[0] = 'v'; break;
      case POSITION:markedBuffer[0] = 'l'; break;
      case POINT:   markedBuffer[0] = 'p'; break;
      default: break;
   }
   strcat(markedBuffer, message);
   SendCommand(sock, markedBuffer);
}

int CCrazyflieSensing::SendCommand(int sock, char* message) {
   return send(sock, message, std::string(message).size(), 0);
}

/****************************************/
/*           Main loop function         */
/****************************************/

void CCrazyflieSensing::ControlStep() {
   char command;
   
   while (fd[stoi(GetId().substr(6))] <= 0) {
      fd[stoi(GetId().substr(6))] = ConnectToSocket();
   }

   if (!posX[stoi(GetId().substr(6))] & !posY[stoi(GetId().substr(6))]) {

      posX[stoi(GetId().substr(6))] = m_pcPos->GetReading().Position.GetX();
      posY[stoi(GetId().substr(6))] = m_pcPos->GetReading().Position.GetY();

      if (stoi(GetId().substr(6)) != 0) {
         float differenceY = posY[0] - posY[stoi(GetId().substr(6))];
         float newPosY = posY[0] + differenceY;
         LOG << "Initial position: " << " id " << std::to_string(fd[stoi(GetId().substr(6))]).c_str() 
                                     << " x: " << posX[stoi(GetId().substr(6))] << " y: " 
                                     << newPosY << std::endl;
      } else {
         LOG << "Initial position: " << " id " << std::to_string(fd[stoi(GetId().substr(6))]).c_str() 
                                     << " x: " << posX[stoi(GetId().substr(6))] << " y: " 
                                     << posY[stoi(GetId().substr(6))] << std::endl;
      }
   }

   char stateBuffer[1024] = {0};
   stateBuffer[0] = '0' + m_cState;

   char batteryBuffer[1024] = {0};
   strcpy(batteryBuffer, std::to_string(sBatRead.AvailableCharge).c_str());

   char velocityBuffer[1024] = {0};
   strcpy(velocityBuffer, std::to_string(velocity).c_str());

   // CreateCommand(fd[stoi(GetId().substr(6))], stateBuffer, STATE);
   // CreateCommand(fd[stoi(GetId().substr(6))], batteryBuffer, BATTERY);
   // CreateCommand(fd[stoi(GetId().substr(6))], velocityBuffer, VELOCITY);

   try {
      // Look battery level
      sBatRead = m_pcBattery->GetReading();
      LOG << "Battery: "<< sBatRead.AvailableCharge << std::endl;

      char currentCommand = ReadCommand(fd[stoi(GetId().substr(6))]);   

      if (currentCommand == 's') {
         TakeOff();
      } 
      if ((sBatRead.AvailableCharge < 0.3 || currentCommand == 'l') && m_cState != STATE_GO_TO_BASE) { 
         GoToBase();
      }

      CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead = m_pcDistance->GetReadingsMap();
      auto iterDistRead = sDistRead.begin();
      if(sDistRead.size() == 4) {
         /* Updates of the distance sensors */
         m_cDist[0] = (iterDistRead++)->second; /* Front distance */
         m_cDist[1] = (iterDistRead++)->second; /* left distance  */
         m_cDist[2] = (iterDistRead++)->second; /* back distance  */
         m_cDist[3] = (iterDistRead)->second;   /* right distance */

         char posBuffer[1024] = {0};
         strcpy(posBuffer, std::to_string(m_pcPos->GetReading().Position.GetX() - posX[stoi(GetId().substr(6))]).c_str());
         strcat(posBuffer, ";");
         strcat(posBuffer, std::to_string(-1.0*(m_pcPos->GetReading().Position.GetY() - posY[stoi(GetId().substr(6))])).c_str());

         char pointBuffer[1024] = {0};
         strcpy(pointBuffer, std::to_string(m_cDist[0]).c_str());
         strcat(pointBuffer, ";");
         strcat(pointBuffer, std::to_string(m_cDist[2]).c_str());
         strcat(pointBuffer, ";");
         strcat(pointBuffer, std::to_string(m_cDist[1]).c_str());
         strcat(pointBuffer, ";");
         strcat(pointBuffer, std::to_string(m_cDist[3]).c_str());

         CreateCommand(fd[stoi(GetId().substr(6))], stateBuffer, STATE);
         CreateCommand(fd[stoi(GetId().substr(6))], batteryBuffer, BATTERY);
         CreateCommand(fd[stoi(GetId().substr(6))], velocityBuffer, VELOCITY);
         CreateCommand(fd[stoi(GetId().substr(6))], posBuffer, POSITION);
         CreateCommand(fd[stoi(GetId().substr(6))], pointBuffer, POINT);

         /* States management */
         switch (m_cState) {
            case STATE_START: break;
            case STATE_TAKE_OFF: TakeOff(); break;
            case STATE_EXPLORE: Explore(); break;
            case STATE_GO_TO_BASE: GoToBase(); break;
            case STATE_LAND: Land(); break;
            default: break;
         }
      }
   } catch (std::exception e) {
      LOGERR << "AN EXCEPTION AS OCCURED IN CONTROLSTEP" << std::endl;
      LOGERR << "Exception details: " << e.what() << std::endl;
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
      } else if (Abs(cPos.GetZ() - height) < 0.1f) {
         m_cBasePos = m_pcPos->GetReading().Position;
         Explore();
      }
   } catch (std::exception e){
      LOGERR << "EXCEPTION AS OCCURED IN TAKEOFF" << std::endl;
      LOGERR << "Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::Explore() {
    try {
      if(m_cState != STATE_EXPLORE) {
         m_cState = STATE_EXPLORE;
      }

      CRadians c_z_angle, c_y_angle, c_x_angle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

      /* Move the drone in the map */
      CVector3 cPos = m_pcPos->GetReading().Position;
      map.Move(&map, (int) ((cPos.GetX() + 5) * 100), (int) ((cPos.GetY() + 5) * 100));
      
      /* Add the sensor value to the map */
      map.AddData(&map,
                  static_cast<int>(m_cDist[0]), /* Front distance in cm */
                  static_cast<int>(m_cDist[1]), /* left distance  in cm */
                  static_cast<int>(m_cDist[2]), /* back distance  in cm */
                  static_cast<int>(m_cDist[3]));/* right distance in cm */
      
      /* If the drone is too close to an obstacle, move away */
      Real minimalDist = 120;
      if (m_cDist[0] < minimalDist && m_cDist[0] != -2) { MoveBack(c_z_angle, 7.0);}
      if (m_cDist[1] < minimalDist && m_cDist[1] != -2) { MoveRight(c_z_angle, 7.0);}
      if (m_cDist[2] < minimalDist && m_cDist[2] != -2) { MoveForward(c_z_angle, 7.0);}
      if (m_cDist[3] < minimalDist && m_cDist[3] != -2) { MoveLeft(c_z_angle, 7.0);}
      
      /* If there is an obstacle in the direction of the drone, choose another direction 
      if (m_cDir != CfDir::STOP && m_cDist[m_cDir] < 75 && m_cDist[m_cDir] != -2) {
         CVector3& stopCPos = StopMvmt();
         stopCPos = cPos;
         m_cDir = CfDir::STOP;
         return;
      }*/
      m_cDir = GetBestDir();

      /* Move in the direction of exploration */
      switch (m_cDir) {
         case CfDir::FRONT: MoveForward(c_z_angle); break;//LOG << "Curr dist : "<< m_cDist[0] << std::endl; break;
         case CfDir::LEFT: MoveLeft(c_z_angle); break;//LOG << "Curr dist : "<< m_cDist[1] << std::endl; break;
         case CfDir::BACK: MoveBack(c_z_angle); break;//LOG << "Curr dist : "<< m_cDist[2] << std::endl; break;
         case CfDir::RIGHT: MoveRight(c_z_angle); break;//LOG << "Curr dist : "<< m_cDist[3] << std::endl; break;
         case CfDir::STOP: {
            /* This case is used to stop any ongoing mouvement before switching direction */
            CVector3& stopCPos = StopMvmt();
            if (Distance(stopCPos, cPos) < 0.05) {
               m_cDir = GetBestDir();
               printMap();
            } else {
               StopMvmt();
            }
            break;
         }
      }
   } catch (std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED IN EXPLORATION" << std::endl;
      LOGERR << "Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::printMap() {
   std::ofstream MyFile("distMap"+GetId()+".txt");
   std::ofstream MyFile2("map"+GetId()+".txt");
   for (unsigned int i = 0; i < 50; i++) {
      for (unsigned int j = 0; j < 50; j++) {
         MyFile << map.distMap[i][j] << " ";
         MyFile2 << map.map[i][j] << " ";
      }
      MyFile << std::endl;
      MyFile2 << std::endl;
   }
   MyFile.close();
   MyFile2.close();
}

void CCrazyflieSensing::GoToBase() {
   try {
      CVector3 cpos = m_pcPos->GetReading().Position;
      if (m_cState != STATE_GO_TO_BASE) {
         // We update the current drone position on the map
         map.Move(&map, (int) ((cpos.GetX() + 5) * 100), (int) ((cpos.GetY() + 5) * 100));

         // We build the flowmap (distMap) only once
         map.BuildFlow(&map);

         m_cState = STATE_GO_TO_BASE;
      }

      /***
       * Return to base 
       * -> we approximate base node position [TODO]
       * -> based on current node, we find the closest explored node from base we know [TODO]
       * -> from this node we run Wave Propagation algorithm to generate flowmap [DONE]
       * -> we then use the flowmap to find the closest path to the base [DONE]
       * -> from this path, the drone moves from node to node choosing the one with the
       *    smallest distance. [DONE]
       * -> when the current node is the approximated base node, we land [DONE]
      ***/

      CRadians c_z_angle, c_y_angle, c_x_angle;
      m_pcPos->GetReading().Orientation.ToEulerAngles(c_z_angle, c_y_angle, c_x_angle);

      // Update the drone position on the map
      map.Move(&map, (int) ((cpos.GetX() + 5) * 100), (int) ((cpos.GetY() + 5) * 100));

      // Are we at destination? If so we land.
      if (map.currX == map.mBase.x && map.currY == map.mBase.y) {
         Land();
      }

      // We get the next direction
      MapExplorationDir nextDir = map.NextNode(&map,  
                  static_cast<int>(m_cDist[0]), /* Front distance in cm */
                  static_cast<int>(m_cDist[1]), /* Left  distance in cm */
                  static_cast<int>(m_cDist[2]), /* Back  distance in cm */
                  static_cast<int>(m_cDist[3]));/* Right distance in cm */
      
      LOG << "<X, Y> := <"<< map.currX << ", " << map.currY << ">;" << std::endl;

      // Move in the chosen direction
      switch (nextDir) {
         case MapExplorationDir::X_POS: MoveLeft(c_z_angle);    break;
         case MapExplorationDir::X_NEG: MoveRight(c_z_angle);   break;
         case MapExplorationDir::Y_POS: MoveBack(c_z_angle);    break;
         case MapExplorationDir::Y_NEG: MoveForward(c_z_angle); break;
         case MapExplorationDir::NONE: break;
         default: break;
      }
   } catch (std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED ON THE WAY BACK" << std::endl;
      LOGERR << "Exception details: " << e.what() << std::endl;
   }
}

void CCrazyflieSensing::Land() {
   try {
      if (m_cState != STATE_LAND) {
         m_cState = STATE_LAND;
      }

      CVector3 cpos = m_pcPos->GetReading().Position;
      if ((Abs(cpos.GetZ()) > 0.01f)) {
         cpos.SetZ(0.0f);
      } else {
         LOG << "MISSION COMPLETE" << std::endl;
      }
   } catch (std::exception e) {
      LOGERR << "EXCEPTION AS OCCURED WHILE LANDING" << std::endl;
      LOGERR << "Exception details: " << e.what() << std::endl;
   }
} 

/****************************************/
/*           Movements functions        */
/****************************************/

enum CCrazyflieSensing::CfDir CCrazyflieSensing::GetBestDir() {
   /* Recurcive call to GetBestDir() */
   MapExplorationDir mapExplorationDir = map.GetBestDir(&map, (MapExplorationDir) m_cDir);

   /* Convert the map direction to the simulated drone direction*/
   switch ((CfDir) mapExplorationDir) {
      case CfDir::FRONT: LOG << "FRONT\n"; break;
      case CfDir::LEFT : LOG << "LEFT\n";  break;
      case CfDir::BACK : LOG << "BACK\n";  break;
      case CfDir::RIGHT: LOG << "RIGHT\n"; break;
      case CfDir::STOP : LOG << "STOP\n";  break;
      default: break;
   } 
   return (CfDir) mapExplorationDir;
}

void CCrazyflieSensing::MoveForward(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CVector3 desiredPos = cPos.Position + CVector3(param * Sin(c_z_angle), -param*Cos(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveLeft(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CVector3 desiredPos = cPos.Position + CVector3(param * Cos(c_z_angle), -param * Sin(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveBack(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CVector3 desiredPos = cPos.Position + CVector3(-param * Sin(c_z_angle), param * Cos(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

void CCrazyflieSensing::MoveRight(CRadians c_z_angle, float dist) {
   float param = (dist == 0)? velocity : dist;
   CCI_PositioningSensor::SReading cPos = m_pcPos->GetReading();
   CVector3 desiredPos = cPos.Position + CVector3(-param * Cos(c_z_angle), param * Sin(c_z_angle), 0);
   m_pcPropellers->SetAbsolutePosition(desiredPos);
}

CVector3& CCrazyflieSensing::StopMvmt() {
   static CVector3 cPos;
   m_pcPropellers->Reset();
   m_pcPropellers->SetAbsolutePosition(cPos);
   return cPos;
}

/****************************************/
/****************************************/

void CCrazyflieSensing::Reset() {
   // TODO
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