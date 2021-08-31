#pragma once // ?


typedef unsigned char 	BYTE;
class Cwheel
{

public:
	
    Cwheel();
	virtual ~Cwheel(); //	

	void Open_port();
    void Read_encoder();
    void Select_Speed(int tar_vel);
    void velocity_cmd(int nLeftRPM , int nRightRPM);
    void JoyStick_msg(float axes_msg[],int button_msg[]);
    void velocity_target(double Linear_vel, double Angular_vel);
    double _RightVelocity_MS, _LeftVelocity_MS;
    int _LeftPosition, _RightPosition;

private:
    int _port , _LeftRPM, _RightRPM, speed_mode, LeftVel_before, RightVel_before;
    BYTE byRMID, byTMID, byID, byPID, byDataNum, byDataSum, byLeftMotState, byRightMotState , _vellocity[13];
    unsigned char _buf[255];
    double _PI_radius;
    double radius;
    double _x;
    unsigned char _REQ_DATA[7];
    unsigned char _servo[8];
    unsigned char _TQ_off[7];
    unsigned char _Motor_Reverse[7];
    unsigned char _Encoder_Reverse[7];
    unsigned char _Motor2_Reverse[7];
    unsigned char _Encoder2_Reverse[7];

};
