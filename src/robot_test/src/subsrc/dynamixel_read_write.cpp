/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <trajectory.h>
#include <controller.h>
#include <dynamixel_sdk.h>
#include <sync_read_write2.h>                               // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in Dynamixel model

#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_GOAL_VELOCITY         552

#define ADDR_PRO_PRESENT_POSITION       580
#define ADDR_PRO_PRESENT_VELOCITY       576

#define VELOCITY_PGAIN                  526
#define VELOCITY_IGAIN                  524

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRO_GOAL_VELOCITY           4
#define LEN_PRO_PRESENT_VELOCITY        4

#define PROFILE_VELOCITY                 560
#define PROFILE_ACCELERATION                 556
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#2 ID: 3
#define DXL4_ID                         4                   // Dynamixel#2 ID: 3
#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/ttyUSB2"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      0              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define PI 3.141592

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

int dxl_comm_result = COMM_TX_FAIL;               // Communication result

//int fprintf(FILE* stream, const char* format, ...);
CRobot_Arm_TR::CRobot_Arm_TR()
{
}
CRobot_Arm_TR::~CRobot_Arm_TR()
{
}


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void CRobot_Arm_TR::goalposition(int* _goal_position)
{
  for(int i = 0 ; i<5 ; i++)
  {
     _dxl_goal_position[i] = _goal_position[i];
  }
  //std::cout << dxl_goal_position[4] << std:: endl;
}

void CRobot_Arm_TR::start()
{
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
  }

  //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, VELOCITY_PGAIN, 500, &_dxl_error);
  //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, VELOCITY_IGAIN, 0, &_dxl_error);
  //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, VELOCITY_PGAIN, 500, &_dxl_error);
  //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, VELOCITY_IGAIN, 0, &_dxl_error);

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &_dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (_dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

   // Enable Dynamixel#2 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
   }

    // Enable Dynamixel#3 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
   }
    // Enable Dynamixel#4 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL4_ID);
   }

   // Add parameter storage for Dynamixel#1 present position value
  _dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (_dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
  }

  // // Add parameter storage for Dynamixel#2 present position value
   _dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
   if (_dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
   }

   // Add parameter storage for Dynamixel#3 present position value
   _dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
   if (_dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL3_ID);
   }

   // Add parameter storage for Dynamixel#4 present position value
   _dxl_addparam_result = groupSyncRead.addParam(DXL4_ID);
   if (_dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL4_ID);
   }
}

void CRobot_Arm_TR::end()
{
  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &_dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (_dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
  }

   // Disable Dynamixel#2 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }

    // Disable Dynamixel#3 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }

    // Disable Dynamixel#4 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &_dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (_dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(_dxl_error));
   }
  // Close port
  portHandler->closePort();
}

void CRobot_Arm_TR::TXRX()
{
       dxl_comm_result = groupSyncRead.txRxPacket(); // << long time
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (groupSyncRead.getError(DXL1_ID, &_dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(_dxl_error));
      }
       else if (groupSyncRead.getError(DXL2_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(_dxl_error));
       }
       else if (groupSyncRead.getError(DXL3_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(_dxl_error));
       }
        else if (groupSyncRead.getError(DXL4_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(_dxl_error));
       }

     _dxl_present_position[1] = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[2] = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[3] = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[4] = groupSyncRead.getData(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

     //Control.get_present_position(dxl_present_position);

     _param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[1]));
     _param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[1]));
     _param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[1]));
     _param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[1]));
    
     _param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[2]));
     _param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[2]));
     _param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[2]));
     _param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[2]));

     _param_goal_position3[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[3]));
     _param_goal_position3[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[3]));
     _param_goal_position3[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[3]));
     _param_goal_position3[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[3]));

     _param_goal_position4[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[4]));
     _param_goal_position4[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[4]));
     _param_goal_position4[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[4]));
     _param_goal_position4[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[4]));


    groupSyncWrite.clearParam();

    // Add Dynamixel#1 goal position value to the Syncwrite storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, _param_goal_position1);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
     }

     // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, _param_goal_position2);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
     }

     // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, _param_goal_position3);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
     }

     // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, _param_goal_position4);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
     }
    // Syncwrite goal position
     dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    
       //Check if groupsyncread data of Dynamixel#1 is available
       _dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (_dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
       }

       // Check if groupsyncread data of Dynamixel#2 is available
       _dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (_dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
       }

       // Check if groupsyncread data of Dynamixel#3 is available
       _dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (_dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL3_ID);
       }

        // Check if groupsyncread data of Dynamixel#4 is available
       _dxl_getdata_result = groupSyncRead.isAvailable(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (_dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL4_ID);
       }
     printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d \t[ID:%03d] GoalPos:%03d  PresPos:%03d \n", DXL1_ID, _dxl_goal_position[1], _dxl_present_position[1], DXL2_ID, _dxl_goal_position[2], _dxl_present_position[2], DXL3_ID, _dxl_goal_position[3], _dxl_present_position[3], DXL4_ID, _dxl_goal_position[4], _dxl_present_position[4]);
}

void CRobot_Arm_TR::TX()
{
  // Setting Trajectory Profile
  for(int i = 1; i<3 ; i++)
  {
    _profile_velocity[i] =  600 * (_dxl_goal_position[i] - _dxl_present_position[i]) / (35 * 6075); // = 6,000,000 * (goal-present) / (profile_time2<3500ms> * resolution<607500>)
    _profile_acceleration[i] = 0.6 * _profile_velocity[i] ; // 0.6 = 600/Profile_t1<1000ms>    
  }
    _profile_velocity[3] =  30000 * (_dxl_goal_position[3] - _dxl_present_position[3]) / (35 * 263187); // = 6,000,000 * (goal-present) / (profile_time2<3500ms> * resolution<526374>)
    _profile_acceleration[3] = 0.6 * _profile_velocity[3];   // 0.6 = 600/Profile_t1<1000ms>  

    _profile_velocity[4] =  60000 * (_dxl_goal_position[4] - _dxl_present_position[4]) / (35 * 4096); // = 6,000,000 * (goal-present) / (profile_time2<3500ms> * resolution<4096>)
    _profile_acceleration[4] = 0.6 * _profile_velocity[4];   // 0.6 = 600/Profile_t1<1000ms>  

     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, PROFILE_VELOCITY,  abs(_profile_velocity[1]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, PROFILE_ACCELERATION,  abs(_profile_acceleration[1]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, PROFILE_VELOCITY,  abs(_profile_velocity[2]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, PROFILE_ACCELERATION,  abs(_profile_acceleration[2]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, PROFILE_VELOCITY,  abs(_profile_velocity[3]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, PROFILE_ACCELERATION,  abs(_profile_acceleration[3]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL4_ID, PROFILE_VELOCITY,  abs(_profile_velocity[4]), &_dxl_error);
     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL4_ID, PROFILE_ACCELERATION,  abs(_profile_acceleration[4]), &_dxl_error);

    // goal position 
     _param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[1]));
     _param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[1]));
     _param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[1]));
     _param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[1]));
    
     _param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[2]));
     _param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[2]));
     _param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[2]));
     _param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[2]));

     _param_goal_position3[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[3]));
     _param_goal_position3[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[3]));
     _param_goal_position3[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[3]));
     _param_goal_position3[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[3]));

     _param_goal_position4[0] = DXL_LOBYTE(DXL_LOWORD(_dxl_goal_position[4]));
     _param_goal_position4[1] = DXL_HIBYTE(DXL_LOWORD(_dxl_goal_position[4]));
     _param_goal_position4[2] = DXL_LOBYTE(DXL_HIWORD(_dxl_goal_position[4]));
     _param_goal_position4[3] = DXL_HIBYTE(DXL_HIWORD(_dxl_goal_position[4]));


    groupSyncWrite.clearParam();

    // Add Dynamixel#1 goal position value to the Syncwrite storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, _param_goal_position1);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
     }

     // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, _param_goal_position2);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
     }

     // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, _param_goal_position3);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
     }

     // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
     _dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, _param_goal_position4);
     if (_dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
     }
    // Syncwrite goal position
     dxl_comm_result = groupSyncWrite.txPacket();
}

void CRobot_Arm_TR::RX()
{
       dxl_comm_result = groupSyncRead.txRxPacket(); // << long time
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (groupSyncRead.getError(DXL1_ID, &_dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(_dxl_error));
      }
       else if (groupSyncRead.getError(DXL2_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(_dxl_error));
       }
       else if (groupSyncRead.getError(DXL3_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(_dxl_error));
       }
        else if (groupSyncRead.getError(DXL4_ID, &_dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(_dxl_error));
       }

     _dxl_present_position[1] = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[2] = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[3] = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     _dxl_present_position[4] = groupSyncRead.getData(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     //std::cout<<" error1 : " << _dxl_goal_position[1]-_dxl_present_position[1] <<" error2 : " << _dxl_goal_position[2]-_dxl_present_position[2] <<" error3 : " << _dxl_goal_position[3]-_dxl_present_position[3] <<" error4 : " << _dxl_goal_position[4]-_dxl_present_position[4] <<std::endl;
    // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d \t[ID:%03d] GoalPos:%03d  PresPos:%03d \n", DXL1_ID, _dxl_goal_position[1], _dxl_present_position[1], DXL2_ID, _dxl_goal_position[2], _dxl_present_position[2], DXL3_ID, _dxl_goal_position[3], _dxl_present_position[3], DXL4_ID, _dxl_goal_position[4], _dxl_present_position[4]);

}


