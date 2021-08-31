#pragma once // ?
#ifndef __CONTROLLER_H //?
#define __CONTROLLER_H //?
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;



class CController
{
public:
	CController();
	virtual ~CController(); //	

	void inverseKin(double x, double y, double alpha); // inverse Kinematics
	void Finite_State_Machine(int button, float axes1, float axes2);
	void get_present_position(int32_t* dxl_present_position);
	void get_linear_present_position(int linear_present_position);
	//void SetPosition(double target[], double time, double duration);
	// void SetTorque(double target[], double time, double duration);
	//void Finite_State_Machine(double time); *******되돌리기

	bool _mode, _mode2;
	int _goal_position[5];
	int _dxl_present_position[5];
	int _x_goal[5];
	

public:
	// VectorXd _torque;
	// VectorXd _q; //joint angle vector
	// VectorXd _qdot; //joint velocity vector
	double _q[5],_z,_x,_y, _gamma_1, _k1_1, _k2_1;
	int _linear_goal_position;
	//int32_t _dxl_present_position[5];
	int _new_mode;

private:
	double _cos_q2, _sin_q2_1, _sin_q2_2, _q2_1, _q2_2, _k1_2, _k2_2, _gamma_2, _q1_1, _q1_2, _q3_1, _q3_2;
	
	
	// VectorXd _qdes; //desired joint angle vector
	// VectorXd _qdotdes; //desired joint velocity vector
	int getValue();
	int getValue2();
	
	double RANGE(double angle);
	double _length_link1 ;
	double _length_link2 ;
	double _length_link3 ;
	bool _rev;

	double _now_time4;
	int _box, _table;
	int _cnt;


};

#endif