 // C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
//#include <string>
#include <cmath>
#include <stdlib.h>
//#include <pthread.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function0
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <wheel.h>
#include <sys/select.h>

#define PI 3.141592
#define radius 0.103
#define _PI_radius PI*radius
#define MOTOR_CONTROLLER_MACHINE_ID   183
#define USER_MACHINE_ID               184
#define ID                            1
//Parameter ID
#define PID_PNT_VEL_CMD              207
#define PID_MAIN_DATA                 193

#define ENABLE                        1  
#define RETURN_PNT_MAIN_DATA          2 

typedef unsigned int 	WORD;
typedef unsigned char   BYTE;

struct timeval tv;
fd_set readfds;
int ret;

typedef struct {
	BYTE byLow;
	BYTE byHigh;
} IByte;

short Byte2Int(BYTE byLow, BYTE byHigh)
{
return (byLow | (short)byHigh<<8);
}

IByte Int2Byte(short nIn)
{
	IByte Ret;

	Ret.byLow = nIn & 0xff;
	Ret.byHigh = nIn>>8 & 0xff;
	return Ret;
}

short Byte2Long(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
	return((int)byData1 | (int)byData2<<8 | (int)byData3<<16 | 
		(int)byData4<<24);
}

BYTE GetCheckSum(short nPacketSize, BYTE *byArray)
{
    BYTE byTmp=0;
    short i;
    for(i=0; i<nPacketSize; i++) 
    {   
        byTmp += *(byArray+i);
    }
    return (~byTmp + 1); 
}

short IsChkSumOK(BYTE *byArray, short nPacketSize)
{
short i;
BYTE cbySum;
cbySum = 0;
for(i=0; i<nPacketSize; i++) {
cbySum += *(byArray + i);
}
if(*byArray + 9 == 10)
{
 if( cbySum == 253 )
  {
     cbySum = 0;
  }
}
//std::cout<<(int)cbySum<<std::endl;
if(cbySum == 0) return 1;
else return 0;
}

int getVel()
{
	int vellocity;
	std::cout << "\n vellocity : ";
	std::cin >> vellocity;
	return vellocity;
}

double MS2RPM(double M_s)
{
	double RPM = 0.0;
	RPM = M_s * 60 / (_PI_radius * 2);
	return RPM;
}
double RPM2MS(double RPM)
{
	double M_s;
	M_s = RPM * 2 * PI * radius / 60;
	return M_s;
}

Cwheel::Cwheel()
{
  BYTE zero=0;
  BYTE one=1;
  speed_mode = 1;
  _REQ_DATA[0] = 183;
  _REQ_DATA[1] = 184;
  _REQ_DATA[2] = 1;
  _REQ_DATA[3] = 4;
  _REQ_DATA[4] = 1;
  _REQ_DATA[5] = 210;
  _REQ_DATA[6] = GetCheckSum(sizeof(_REQ_DATA)-1,_REQ_DATA); 

  _servo[0] = 183 ;
  _servo[1] = 184;
  _servo[2] = 1;
  _servo[3] = 24;
  _servo[4] = 1;
  _servo[5] = 1;
  _servo[6] = GetCheckSum(sizeof(_servo)-1,_servo); 

  _TQ_off[0] = 183;
  _TQ_off[1] = 184;
  _TQ_off[2] = 1;
  _TQ_off[3] = 5;
  _TQ_off[4] = 1;
  _TQ_off[5] = 0;
  _TQ_off[6] = GetCheckSum(sizeof(_TQ_off)-1,_TQ_off);

  _Motor_Reverse[0] = 183;
  _Motor_Reverse[1] = 184;
  _Motor_Reverse[2] = 1;
  _Motor_Reverse[3] = 71;
  _Motor_Reverse[4] = 1;
  _Motor_Reverse[5] = zero;
  _Motor_Reverse[6] = GetCheckSum(sizeof(_Motor_Reverse)-1,_Motor_Reverse);

  _Motor2_Reverse[0] = 183;
  _Motor2_Reverse[1] = 184;
  _Motor2_Reverse[2] = 1;
  _Motor2_Reverse[3] = 18;
  _Motor2_Reverse[4] = 1;
  _Motor2_Reverse[5] = 1;
  _Motor2_Reverse[6] = GetCheckSum(sizeof(_Motor2_Reverse)-1,_Motor2_Reverse);

  _Encoder_Reverse[0] = 183;
  _Encoder_Reverse[1] = 184;
  _Encoder_Reverse[2] = 1;
  _Encoder_Reverse[3] = 22;
  _Encoder_Reverse[4] = 1;
  _Encoder_Reverse[5] = 0;
  _Encoder_Reverse[6] = GetCheckSum(sizeof(_Encoder_Reverse)-1,_Encoder_Reverse);

  _Encoder2_Reverse[0] = 183;
  _Encoder2_Reverse[1] = 184;
  _Encoder2_Reverse[2] = 1;
  _Encoder2_Reverse[3] = 23;
  _Encoder2_Reverse[4] = 1;
  _Encoder2_Reverse[5] = 1;
  _Encoder2_Reverse[6] = GetCheckSum(sizeof(_Encoder2_Reverse)-1,_Encoder2_Reverse);
  _x = 0.2;
}
Cwheel::~Cwheel()
{

}

void Cwheel::Open_port()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  _port = open("/dev/ttyUSB1", O_RDWR|O_NOCTTY);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  
  bzero(&tty, sizeof(tty)); 
  // Read in existing settings, and handle any error
  if(tcgetattr(_port, &tty) != 0) 
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  //tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  //tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  //tty.c_cflag |= CS8; // 8 bits per byte (most common)
  //tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  //tty.c_cflag = CREAD; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  //tty.c_cflag =  IGNPAR | ICRNL;
  //tty.c_ lflag &= ~ICANON;
  //tty.c_lflag &= ECHO; // Disable echo
  //tty.c_lflag &= ~ECHOE; // Disable erasure
  //tty.c_lflag &= ~ECHONL; // Disable new-line echo
  //tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  //tty.c_iflag = INPCK;
  //tty.c_oflag = 0;
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  //tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  //tty.c_cc[VTIME] = 0.01;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  //tty.c_cc[VMIN] = 1;
  //tty.c_lflag = ICANON;
  
  // Set in/out baud rate to be 9600
  //cfsetispeed(&tty, B9600);

  // Set in/out baud rate to be 115200
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_lflag = 0;
  bzero(tty.c_cc, NCCS);
  tty.c_cc[VTIME] = 0; 
  tty.c_cc[VMIN] = 1;  

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CS8;
  tty.c_lflag &= ~ECHO; 
  //tty.c_lflag &= ~ECHO; // Disable echo
  //tty.c_lflag &= ~ECHOE; // Disable erasure
  //tty.c_lflag &= ~ECHONL; // Disable new-line echo
  //tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  //tty.c_cflag &= CREAD;
  //tty.c_iflag = ICRNL | INPCK | IGNPAR;
  tty.c_oflag = 0;
 // tty.c_lflag  =  ICANON;

  
  tcsetattr(_port,  TCSANOW,  &tty);
  tcflush(_port,  TCIOFLUSH);//
  // Save tty settings, also checking for error
  if (tcsetattr(_port, TCSANOW, &tty) != 0) 
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  write(_port, _servo,sizeof(_servo));
  usleep(200000);
  tcflush(_port,  TCIOFLUSH);//
   write(_port, _Motor_Reverse,sizeof(_Motor_Reverse));
   usleep(200000);
   //tcflush(_port,  TCIOFLUSH);//
   //write(_port, _Motor2_Reverse,sizeof(_Motor2_Reverse));
   //usleep(200000);
   //tcflush(_port,  TCIOFLUSH);//
   //write(_port, _Encoder_Reverse,sizeof(_Encoder_Reverse));
   //usleep(20000);
  tcflush(_port,  TCIOFLUSH);//
  write(_port, _Encoder2_Reverse,sizeof(_Encoder2_Reverse));
  usleep(20000);
}

void Cwheel::velocity_target(double Linear_vel, double Angular_vel)
{
  double LeftVel2 , RightVel2;
  LeftVel2 = 0.0;
  RightVel2 = 0.0;
  if(Angular_vel == 0)
  {
    LeftVel2 = MS2RPM(Linear_vel);
    velocity_cmd(LeftVel2,LeftVel2);
  }
  else
  {
    LeftVel2 = Linear_vel - Angular_vel * _x;
    RightVel2 = 2 * Linear_vel - LeftVel2;
    LeftVel2 = MS2RPM(LeftVel2);
    RightVel2 = MS2RPM(RightVel2);
    velocity_cmd(LeftVel2,RightVel2);
  }
}

void Cwheel::velocity_cmd(int nLeftRPM , int nRightRPM)
{
  if(speed_mode == 4)
  {
   write(_port, _TQ_off,sizeof(_TQ_off));
   usleep(20000);
   tcflush(_port, TCIOFLUSH);
  // std::cout<<"Now : Torque Off Mode / Button 'B' = Normal Speed Mode / Button 'Y' = High Speed Mode" <<std::endl;
   write(_port, _REQ_DATA, sizeof(_REQ_DATA));
   Read_encoder();
 }
  else
  {
    // write(_port, _REQ_DATA, sizeof(_REQ_DATA));
    // read(_port, _buf, sizeof(_buf));
    // std::cout<<(int)_buf[5]<<std::endl;
  //std::cout<<nLeftRPM<<nRightRPM<<std::endl;
    tcflush(_port,  TCIOFLUSH);
    BYTE byChkSum, byDataNum;
    IByte iData;

      byDataNum = 7;

    _vellocity[0]  = MOTOR_CONTROLLER_MACHINE_ID;      //RMID
    _vellocity[1]  = USER_MACHINE_ID;                  //TMID
    _vellocity[2]  = ID;
    _vellocity[3]  = PID_PNT_VEL_CMD;
    _vellocity[4]  = byDataNum;
    _vellocity[5]  = ENABLE;
    iData   = Int2Byte(nLeftRPM);
    _vellocity[6]  = iData.byLow;
    _vellocity[7]  = iData.byHigh;
    _vellocity[8]  = ENABLE;
    iData   = Int2Byte(nRightRPM);
    _vellocity[9]  = iData.byLow;
    _vellocity[10] = iData.byHigh;

    _vellocity[11] = RETURN_PNT_MAIN_DATA;

    for(int i = 0 ; i < 12; i++) 
    {
      byChkSum += _vellocity[i];
    }
    _vellocity[12] = ~(byChkSum) + 1;
    write(_port, _vellocity, sizeof(_vellocity));
    Read_encoder();
  }
}

void Cwheel::Read_encoder()
{ 
  for(int i = 0 ; i < 255 ; i++)
  {
    _buf[i] = 0;
  }
  //FD_ZERO(&readfds);
  //FD_SET(_port, &readfds);
  //tv.tv_sec = 0;
  //tv.tv_usec = 20000; // 0.02s
  //ret = select(_port + 1, &readfds, NULL, NULL, &tv);
  //if (FD_ISSET(_port, &readfds)) 
  //{
  read(_port, _buf, sizeof(_buf));
  //}
    byDataSum = IsChkSumOK(_buf,24);
    if(byDataSum == 1)
    {
        byRMID            = _buf[0];      //184 <- user machine id
        byTMID            = _buf[1];      //183 <- motor controller machine id
        byID              = _buf[2];      //1   <- id
        byPID             = _buf[3];      //210 <- PID  
        byDataNum         = _buf[4];      //18  <- bytes

        _RightRPM          = Byte2Int(_buf[5], _buf[6]);
        //nRefLeftCurrent   = Byte2Int(_buf[7], _buf[8]);
        byRightMotState    = _buf[9];
        _RightPosition     = Byte2Long(_buf[10], _buf[11], _buf[12], _buf[13]);
        _LeftRPM         = Byte2Int(_buf[14], _buf[15]);
        //nRefRightCurrent  = Byte2Int(_buf[16], _buf[17]);
        byLeftMotState   = _buf[18];
        _LeftPosition    = Byte2Long(_buf[19], _buf[20], _buf[21], _buf[22]);
	_LeftVelocity_MS = RPM2MS(_LeftRPM);
	_RightVelocity_MS = RPM2MS(_RightRPM);
	//std::cout<<(int)_buf[19]<<std::endl;
    }
    else
    {
      std::cout<<"Check Sum Error"<<std::endl;
     // std::cout<<"RIGHT : " <<(int)_buf[10]<<std::endl;
     // std::cout<<"LEFT  : " <<(int)_buf[19]<<std::endl;
      //for(int i= 0; i<24; i++)
      //{
      //  std::cout<<"_buf["<<i<<"] : "<<(int)_buf[i]<<std::endl; 
      //}
    }
    ///////////// cout to check encoder //////////////////////////
    // std::cout<<"LeftRPM : "<<_RightRPM <<" (Rotate Per Minute)" <<std::endl;
    // std::cout<<"RightRPM : "<<_LeftRPM <<" (Rotate Per Minute)" <<std::endl;
    // std::cout<<"LeftVelocity : "<<_LeftVelocity_MS<< " (m/s)"<<std::endl;
    // std::cout<<"RightVelocity : "<<_RightVelocity_MS<< " (m/s)"<<std::endl;
    // std::cout<<"LeftPosition : " << _LeftPosition << " (CNT)" << std::endl;
    // std::cout<<"RightPosition : " << _RightPosition << " (CNT)" << std::endl;
    // std::cout<<"RIGHT : " <<(int)_buf[10]<<"  LEFT  : " <<(int)_buf[19]<< "  VEL(L) : " << (int)_buf[14] << " VEL(R) : " << (int)_buf[5] << std::endl;
}

void Cwheel::JoyStick_msg(float axes_msg[], int button_msg[])
{
 //std::cout<<" 1 : "<< button_msg[1]<<" 2 : "<<button_msg[2]<<"3 : "<<button_msg[3]<<std::endl;
int LeftVel , RightVel;
  if(button_msg[1] == 1)
  {
  speed_mode = 1; //normal
  }
  else if (button_msg[3] == 1)
  {
    speed_mode = 2; // fast
  }
  else if (button_msg[2] == 1)
  {
    speed_mode = 4; // slow
  }
 // else if (button_msg[0] == 1)
 // {
 //   speed_mode = 4; // stop
 // }
////////FORWARD////////////////
switch (speed_mode)
{
case 1:
  if(axes_msg[7] == 1)
  {
    RightVel = 30;
    LeftVel = 30;
      if(axes_msg[6] == 1)
      {
        RightVel = 15;
        LeftVel = 45;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 45;
        LeftVel = 15;
      }
  }
///////BACKWARD///////////////////
  else if ( axes_msg[7] == -1)
  {
    RightVel = -30;
    LeftVel = -30;
      if(axes_msg[6] == 1)
      {
        RightVel = -15;
        LeftVel = -45;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = -45;
        LeftVel = -15;
      }
  }
////////STOP////////////////
  else
  {
    RightVel = 0;
    LeftVel = 0;
      if(axes_msg[6] == 1)
      {
        RightVel = -20;
        LeftVel = 20;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 20;
        LeftVel = -20;
      }
  }

  // if(axes_msg[1] == 0)
  // {
  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*15;
  //     RightVel += axes_msg[1]*15;
  //   }
  // }

  // else
  // {
  //   LeftVel += axes_msg[1]*20;
  //   RightVel += axes_msg[1]*20;

  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*15;
  //     RightVel += axes_msg[1]*15;
  //   }
  // }
  break;

case 2:
if(axes_msg[7] == 1)
  {
    RightVel = 45;
    LeftVel = 45;
      if(axes_msg[6] == 1)
      {
        RightVel = 30;
        LeftVel = 60;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 60;
        LeftVel = 30;
      }
  }
///////BACKWARD///////////////////
  else if ( axes_msg[7] == -1)
  {
    RightVel = -45;
    LeftVel = -45;
      if(axes_msg[6] == 1)
      {
        RightVel = -30;
        LeftVel = -60;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = -60;
        LeftVel = -30;
      }
  }
////////STOP////////////////
  else
  {
    RightVel = 0;
    LeftVel = 0;
      if(axes_msg[6] == 1)
      {
        RightVel = -30;
        LeftVel = 30;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 30;
        LeftVel = -30;
      }
  }

  // if(axes_msg[1] == 0)
  // {
  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*30;
  //     RightVel += axes_msg[1]*30;
  //   }
  // }

  // else
  // {
  //   LeftVel += axes_msg[1]*40;
  //   RightVel += axes_msg[1]*40;

  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*30;
  //     RightVel += axes_msg[1]*30;
  //   }
  // }
  break;

  case 3:
  if(axes_msg[7] == 1)
  {
    RightVel = 10;
    LeftVel = 10;
      if(axes_msg[6] == 1)
      {
        RightVel = 5;
        LeftVel = 10;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 10;
        LeftVel = 5;
      }
  }
///////BACKWARD///////////////////
  else if ( axes_msg[7] == -1)
  {
    RightVel = -10;
    LeftVel = -10;
      if(axes_msg[6] == 1)
      {
        RightVel = -5;
        LeftVel = -10;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = -10;
        LeftVel = -5;
      }
  }
////////STOP////////////////
  else
  {
    RightVel = 0;
    LeftVel = 0;
      if(axes_msg[6] == 1)
      {
        RightVel = -10;
        LeftVel = 10;
      }
      else if ( axes_msg[6] == -1)
      {
        RightVel = 10;
        LeftVel = -10;
      }
  }

  // if(axes_msg[1] == 0)
  // {
  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*4;
  //     RightVel += axes_msg[1]*4;
  //   }
  // }

  // else
  // {
  //   LeftVel += axes_msg[1]*5;
  //   RightVel += axes_msg[1]*5;

  //   if(axes_msg[0] == 0)
  //   {}

  //   else
  //   {
  //     LeftVel -= axes_msg[1]*4;
  //     RightVel += axes_msg[1]*4;
  //   }
  // }
  break;

  case 4:
    RightVel = 0;
    LeftVel = 0;
  break;
default:
  break;
}
  if(button_msg[4] == 1)
  {
    if(RightVel > 0)
    {
      RightVel += 40;
      if(LeftVel > 0)
      {
        LeftVel+=40;
      }
      else if (LeftVel < 0)
      {
        LeftVel -= 40;
      }
    }
    else if (RightVel < 0)
    {
      RightVel -= 40;
            if(LeftVel > 0)
      {
        LeftVel+=40;
      }
      else if (LeftVel < 0)
      {
        LeftVel -= 40;
      }
    }
  }

//////DO NOT TURN//////////////

LeftVel = LeftVel*0.3 + _LeftRPM *0.7;
RightVel = RightVel*0.3 + _RightRPM *0.7;
velocity_cmd(RightVel,LeftVel);
}
