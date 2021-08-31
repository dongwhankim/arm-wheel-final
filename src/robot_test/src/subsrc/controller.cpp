#include "controller.h"
#include <math.h>
#include <stdio.h>
#include <cmath>
#include "trajectory.h"
#include <sync_read_write2.h>
#include <linear_read_write.h>

#define PI 3.141592

typedef enum{
	start, ready, movetobox, pickup, movetotable, dropit, finish
} States;

States CurrentState;
CTrajectory Trajectory[20]; //size = joint dofpos_goal[0] = target[0];
pthread_mutex_t mutex;

CController::CController()
{		
    _mode = 0 , _mode2 = 0;
	_length_link1 = 0.3 , _length_link2 = 0.3 , _length_link3 = 0.2;
	_now_time4 = 0.0;
	_box =0 , _table =0 ;
	_z = 0.0 , _x = 0.0 , _y = 0.0 ;
	_gamma_1 = 0.0 , _k1_1 = 0.0 , _k2_1 = 0.0 ;
	_cos_q2 = 0.0 , _sin_q2_1 = 0.0 , _sin_q2_2 = 0.0 , _q2_1 = 0.0 , _q2_2 = 0.0 , _k1_1 = 0.0 , _k2_2 = 0.0 , _gamma_2 = 0.0 , _q1_1 = 0.0 , _q1_2 = 0.0 , _q3_1 = 0.0 , _q3_2 = 0.0 ;
	_linear_goal_position = 0;
	_new_mode = 0;
	_cnt = 0;
	for(int i = 0 ; i< 5 ; i ++)
	{
		_q[i] = 0.0 ; 
		_x_goal[i] = 0.0 ;
	}
	for(int i = 0 ; i< 5 ; i ++)
	{
		_dxl_present_position[i] = 0 ; 
		_goal_position[i] = 0 ;
	}
}
CController::~CController()
{
}

double CController::RANGE(double angle) 
{
	while (angle > PI || angle <= -PI) 
	{
		if (angle > PI) 
		{
			angle = angle - 2 * PI;
		}
		else 
		{
			angle = angle + 2 * PI;
		}
	}
	return angle;
}

int CController::getValue()
{
	//도시락 고르기
	int Box_Number;
	std::cout << "\n 도시락 선택 4(↖)3(↗) \n             2(↙)1(↘) :";
	std::cin >> Box_Number;
	return Box_Number;
}

int CController::getValue2()
{
	//놓을 테이블 고르기
	int Table_Number;
	std::cout << "\n 놓을 테이블 선택 1(→),2(←):";
	std::cin >> Table_Number;
	return Table_Number;

}

void CController::inverseKin(double x, double y, double alpha)
{
	_x = x - _length_link3 * cos(alpha);
	_y = y - _length_link3 * sin(alpha);

	_cos_q2 = (pow(_x, 2) + pow(_y, 2) - pow(_length_link1, 2) - pow(_length_link2, 2)) 
		/ (2 * _length_link1 * _length_link2);

	if (abs(_cos_q2) > 1) {
		std::cout << "작동범위를 벗어납니다. " << endl;
		_q[0] = 0.0;// {0.0, 0.0, 0.0 };
		_q[1] = 0.0;
		_q[2] = 0.0;
	}
	else {

		_sin_q2_1 = sqrt(1 - pow(_cos_q2, 2));
		_sin_q2_2 = -sqrt(1 - pow(_cos_q2, 2));

		_q2_1 = atan2(_sin_q2_1, _cos_q2);
		_q2_2 = atan2(_sin_q2_2, _cos_q2);

		double q2[2] = { _q2_1, _q2_2 };
		_k1_1 = _length_link2 * cos(_q2_1) + _length_link1;
		_k1_2 = _length_link2 * cos(_q2_2) + _length_link1;
		//double k1[2] = { k1_1,k1_2 };
		// k1
		_k2_1 = _length_link2 * sin(_q2_1);
		_k2_2 = _length_link2 * sin(_q2_2);
		//double k2[2] = { k2_1, k2_2 };
		// k2
		_gamma_1 = atan2(_k2_1, _k1_1);
		_gamma_2 = atan2(_k2_2, _k1_2);
		
		//double gamma[2] = { gamma_1,gamma_2 };
		_z = atan2(_y, _x);
		_q1_1 = _z - _gamma_1;
		_q1_2 = _z - _gamma_2;
		_q3_1 = alpha - _q1_1 - _q2_1;
		_q3_2 = alpha - _q1_2 - _q2_2;
		double q1[2] = { _q1_1,_q1_2 };
		//q1
		double q3[2] = { _q3_1,_q3_2 };
		//q3
		double q_1[4] = { 0, q1[0],q2[0],q3[0] };
		double q_2[4] = { 0, q1[1],q2[1],q3[1] };
		for (int i = 1; i < 4; i++) {
			q_1[i] = RANGE(q_1[i]);
			q_2[i] = RANGE(q_2[i]);
			if (_rev == true)
			{
				_q[i] = q_2[i];
				//std::cout << q_2[3] << std::endl;
			}
			else
			{
				_q[i] = q_1[i];
				//std::cout << "_reverse" << std::endl;
			}
			// _q[i]
			// 1 이냐 2냐 고를수 있음
		}
	}

}

void CController::get_present_position(int32_t dxl_present_position[])
{
	//std::cout << dxl_present_position[1] << std::endl;

		for(int i = 1 ; i<5 ; i++)
	{
		_dxl_present_position[i] = dxl_present_position[i] ;		
	}
		//std::cout << _dxl_present_position[4] << std::endl;
}

void CController::get_linear_present_position(int linear_present_position)
{
		_dxl_present_position[0] = linear_present_position ;	
}


void CController::Finite_State_Machine( int button, float axes1, float axes2)
{ 	//std::cout <<time << std::endl;
	//std::cout << CurrentState << std::endl;
	//std::cout<< button << axes1 << axes2 << std::endl;
	//std::cout << _new_mode << std::endl;
	if ( button == 1)
	{
		CurrentState = movetotable;
	}
	else if ( axes1 < 0 )
	{
		CurrentState = ready;
	}
	else if ( axes2 < 0 )
	{
		CurrentState  = pickup;
	}
	else
	{
		CurrentState  = start;
	}
		switch (CurrentState)
	{
 		case start: // 시작자세 및 박스번호 준비
				break;
        
	  		case ready: // 물건앞으로 왼위

           		_x_goal[0] = 6000000;
            	_x_goal[1] = 130000;
            	_x_goal[2] = 180000;
            	_x_goal[3] = 120000;
				break;

			case movetobox: // 내려놓기위해 팔 내밀기 왼위
    		
				_x_goal[0] = 1000000; // 
		       	_x_goal[1] = 0;
		       	_x_goal[2] = 0; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
		       	_x_goal[3] = 0;
				break;

			case pickup: // 잡거나 놓거나 오른위
  			
			  	if(	_dxl_present_position[4] > 5)
				  {
					_x_goal[4] = 0;
				  }
				else
				{
    	       		_x_goal[4] = 745;
				}
				break;

			case movetotable: // 기본자세 버튼
			    _x_goal[0] = 0; // 
		       	_x_goal[1] = -65000;
		       	_x_goal[2] = 200000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
		       	_x_goal[3] = 100000;
			break;
	}
}
// 	void CController::Finite_State_Machine( double time)
// {
	// 		switch (CurrentState)
	// {
 	// 	case start: // 시작자세 및 박스번호 준비
  	// 	if (_mode == 0)
    //     {	//std::cout <<time << std::endl;
    //     	_now_time4 = time;            
	// 		for ( int i = 0 ; i< 5 ; i++)
    //         {
    //           Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
	// 		  _goal_position[i] = _dxl_present_position[i];
    //         }
    //     	_mode = 1;
	// 		break;
    //     }
  	// 	else
	// 	{
    //     	if (time >= _now_time4 + 5)
	//       	{
	// 		    _box = getValue(); //박스의 위치 
	// 		    CurrentState = ready; // state를 바꿈
    //         	_mode = 0;				
	// 		    break;
	// 	    }
	// 	      _rev = false;
	// 	      // _x_goal[0] = 0.0; // 
	// 	      // _x_goal[1] = 0.573205;
	// 	      // _x_goal[2] = -0.2; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	      // _x_goal[3] = 0.0 ;
    //        		_x_goal[0] = 5000000; // 
	// 	       	_x_goal[1] = 74000;
	// 	       	_x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	       	_x_goal[3] = 74000;
    //        		_x_goal[4] = 0;
    //       //inverseKin(_x_goal[1] , _x_goal[2] , _x_goal[3] );
	// 			//Trajectory[0].update_time(time);
    //             //Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             //_linear_goal_position = Trajectory[0].position_cubicSpline();
					

    //       	for(int i = 1 ; i<5 ; i++)
    //         {
    //            Trajectory[i].update_time(time);
    //             //target[i] = _q[i] * 303454 / PI ;
    //             //std::cout << check[i] << std::endl;
    //             Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             _goal_position[i] = Trajectory[i].position_cubicSpline();
    //         }
	// 		//std::cout << "test " << _goal_position[4] << std::endl;
	// 		 //Arm.goalposition(_goal_position);
				
	// 			break;
    //     }	      
  	// 	case ready: // 해당 박스 위로 준비자세
   	// 		if (_mode == 0)
    //     	{
    //     		_now_time4 = time;
    //       		for ( int i = 1 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
	// 			break;
    //     	}
	// 	//_table = getValue2();
    // 		else 
    // 		{
	// 			if (time >= _now_time4 + 5)
	// 			{
	// 				CurrentState = movetobox;
    //   				_mode = 0;
	// 				break;
	// 			}
	// 			else
	// 			{	
	// 				// if (_box == 1 || _box == 3)
	// 				// {
	// 				_rev = false;
    //        			_x_goal[0] = 2000000; // 
	// 	       		_x_goal[1] = 30000;
	// 	       		_x_goal[2] = 120000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	       		_x_goal[3] = 78000;
    //        			_x_goal[4] = 745;

	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 				// }

	// 		// 	else if (_box == 2 || _box == 4)
	// 		// 	{
	// 		// 		_rev = true;
    //    		//	    _x_goal[0] = 0.0; // 
	// 		//      _x_goal[1] = 0.0;
	// 		//      _x_goal[2] = 0.0; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		//      _x_goal[3] = 0.0 ;
    //    		//	    _x_goal[4] = 0.0;
	// 		// 		inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //    		//   	for(int i = 1 ; i<5 ; i++)
    //    		//      {
    //    		//      	Trajectory[i].update_time(time);
    //    		//       	target[i] = _q[i] * 303454 / PI ;
    //    		//       	std::cout << check[i] << std::endl;
    //    		//         	Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //    		//         	_goal_position[i] = Trajectory[i].position_cubicSpline();
    //    		//      }
	// 		// 		break;
	// 		// 	}
	// 			}
    // 		}

	// 	case movetobox: // 박스 앞으로 이동
    // 		if (_mode == 0)
    //     	{
    //     		_now_time4 = time;
    //       		for ( int i = 0 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
	// 			break;
    //     	}
	// 		if (time >= _now_time4 + 5)
	// 		{
	// 			CurrentState = pickup;
    // 			_mode = 0;
	// 			break;
	// 		}
	// 		else
	// 		{	//std::cout << "time : " << time << std::endl;
	// 			if (_box == 1) // 1번(우하단 도시락)
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	       		_x_goal[1] = 74000;
	// 	       		_x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	       		_x_goal[3] = 74000;
    //        			_x_goal[4] = 0;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}

	// 			else if (_box == 2) // 2번(좌하단 도시락)
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	    	   	_x_goal[1] = 74000;
	// 		       	_x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		       	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 0;

	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 			else if (_box == 3) // 3번(우상단 도시락)
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	    	   	_x_goal[1] = 74000;
	// 		       	_x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		       	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 0;

    //       			//	inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 			else  // 4번(좌상단 도시락)
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	    	   	_x_goal[1] = 74000;
	// 		       	_x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		       	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 0;
	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 		}

	// 	case pickup: // 박스를 들고 다시 나오기
  	// 		if(_mode == 0)
  	// 		{
    //     		_now_time4 = time;
    //     		for ( int i = 0 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
	// 			break;
  	// 		}
	// 		if (time >= _now_time4 + 10)
	// 		{
	// 			//_table = getValue2();
	// 			CurrentState = movetotable;
	// 			_mode = 0;
    //   			_mode2 = 0;
	// 			break;
	// 		}
	// 		else if (time >= _now_time4 + 5) // 4초까지 기다린후 6초까지(2초간) 나오기
	// 		{       // std::cout << "time :??? " << time << std::endl;
	// 				//std::cout << "nowtime4 " << _now_time4 << std::endl;   
    //   			if(_mode2 == 0)
    //   			{
    //    				for ( int i = 0 ; i< 5 ; i++)
    //        			{
    //           			Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        			}
	// 	  			// std::cout << "nowtime4 " << _now_time4 << std::endl;   
    //        			_mode2 = 1;
	// 				break;
    //   			}
	// 	       	_x_goal[0] = 2000000; // 
	// 	    	_x_goal[1] = 74000;
	// 		    _x_goal[2] = 155000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    _x_goal[3] = 74000;
    // 	       	_x_goal[4] = 0;
				

	// 			Trajectory[0].update_time(time);
    //             Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 10);
    //             _linear_goal_position = Trajectory[0].position_cubicSpline();
	// 			//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
	// 	        for(int i = 1 ; i<5 ; i++)
    //          		{ 
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		//std::cout << check[i] << std::endl;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 10);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //           		}
    //         	break;
	// 		}
	// 		else
	// 		{	//	std::cout << "time : " << time << std::endl;
	// 			_x_goal[0] = 4000000; // 
	// 	    	_x_goal[1] = 30000;
	// 		    _x_goal[2] = 60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    _x_goal[3] = 74000;
    // 	       	_x_goal[4] = 745;

	// 			Trajectory[0].update_time(time);
    //             Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             _linear_goal_position = Trajectory[0].position_cubicSpline();
	// 			//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       		for(int i = 1 ; i<5 ; i++)
    //          	{
    //            		Trajectory[i].update_time(time);
    //         		//target[i] = _q[i] * 303454 / PI ;
    //         	    //std::cout << check[i] << std::endl;
	//                 Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             	_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          	}
	// 			break;
	// 		}

	// 	case movetotable: // 테이블 앞으로 이동
	// 	   if(_mode == 0)
  	// 		{
    //     		_now_time4 = time;
    //     		for ( int i = 0 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
	// 			break;
  	// 		}
	// 		if (time >= _now_time4 + 5)
	// 		{
	// 			CurrentState = dropit;
	// 			_mode = 0;
	// 			break;
	// 		}

	// 		if (_table == 1) // 우측 테이블에 놓는 경우
	// 		{
	// 			if (_box == 1 || _box == 3) // 우측 도시락을 우측 테이블에 놓는 경우
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	    		_x_goal[1] = 30000;
	// 		    	_x_goal[2] = 60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 745;
				
	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 			else // 좌측 도시락을 우측 테이블에 놓는 경우
	// 			{
	// 				_rev = false;
	// 				_x_goal[0] = 1000000; // 
	// 	    		_x_goal[1] = 30000;
	// 		    	_x_goal[2] = 60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 745;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //                     Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 		}
	// 		else // 좌측 테이블에 놓는 경우
	// 		{
	// 			if (_box == 1 || _box == 3) // 우측 도시락을 좌측 테이블에 놓는 경우
	// 			{
	// 				_rev = true;
	// 				_x_goal[0] = 1000000; // 
	// 	    		_x_goal[1] = 30000;
	// 		    	_x_goal[2] = 60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 745;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 			else // 좌측 도시락을 좌측 테이블에 놓는 경우
	// 			{
	// 				_x_goal[0] = 1000000; // 
	// 	    		_x_goal[1] = 30000;
	// 		    	_x_goal[2] = 60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    	_x_goal[3] = 74000;
    // 	       		_x_goal[4] = 745;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 		}

	// 	case dropit: // 내려놓으러 가기 2초동안 이동 , 4초까지 내려놓는거 대기
   	// 		if(_mode == 0)
  	// 		{
	//   			_now_time4 = time;
    //     		for ( int i = 0 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
	// 			break;
  	// 		}
	// 		if (time >= _now_time4 + 5)
	// 		{
	// 			CurrentState = finish;
	// 			_mode = 0;
	// 			break;
	// 		}

	// 		else
	// 		{
	// 		 	_x_goal[0] = 3000000; // 
	// 	    	_x_goal[1] = -30000;
	// 		    _x_goal[2] = -60000; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 		    _x_goal[3] = -74000;
    // 	       	_x_goal[4] = 0;

	// 			Trajectory[0].update_time(time);
    //             Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             _linear_goal_position = Trajectory[0].position_cubicSpline();
	// 			//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
	//         	for(int i = 1 ; i<5 ; i++)
    //          	{
    //            		Trajectory[i].update_time(time);
    //             	//target[i] = _q[i] * 303454 / PI ;
    //             	Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             	_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          	}
	// 			break;
	// 		}

	// 	case finish: // 초기상태로 재설정
   	// 		if(_mode == 0)
  	// 		{
    //     		_now_time4 = time;
    //     		for ( int i = 0 ; i< 5 ; i++)
    //        		{
    //           		Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        		}
    //     		_mode = 1;
  	// 		}
	// 		if (time >= _now_time4 + 10)
	// 		{
	// 			CurrentState = start;
	// 			_mode = 0;
    //   			_mode2 = 0;
	// 			break;
	// 		}
	// 		else
	// 		{
	// 			if (time >= _now_time4 + 5) // 초기자세로 이동
	// 			{
    //   				if(_mode2 == 0)
    // 				{
    //       				for ( int i = 0 ; i< 5 ; i++)
    //    					{
    //     		  			Trajectory[i].reset_initial(time, _dxl_present_position[i], 0);
    //        				}
	//            			_mode2 = 1;
    //   				}
	// 				_rev = false;
	// 		    	_x_goal[0] = -1000000; // 
	// 	       		_x_goal[1] = 0;
	// 	       		_x_goal[2] = 0; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	       		_x_goal[3] = 0 ;
    //        			_x_goal[4] = 745;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 10);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //       			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 10);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 			}
	// 				_x_goal[0] = 0; // 
	// 	       		_x_goal[1] = 0;
	// 	       		_x_goal[2] = 0; // 0.175가 계산상 맞지만 165가 simulation상 맞음 why??
	// 	       		_x_goal[3] = 0;
	//            		_x_goal[4] = 0;

	// 				Trajectory[0].update_time(time);
    //             	Trajectory[0].update_goal(_x_goal[0], 0, _now_time4 + 5);
    //             	_linear_goal_position = Trajectory[0].position_cubicSpline();
	// 				//inverseKin( _x_goal[1] , _x_goal[2] , _x_goal[3] ) ;
    //     			for(int i = 1 ; i<5 ; i++)
    //          		{
    //            			Trajectory[i].update_time(time);
    //             		//target[i] = _q[i] * 303454 / PI ;
    //             		Trajectory[i].update_goal(_x_goal[i], 0, _now_time4 + 5);
    //             		_goal_position[i] = Trajectory[i].position_cubicSpline();
    //          		}
	// 				break;
	// 		}
	// }
//}	

// 원하는 좌표로 이동
// void CController::SetPosition(double target[], double time, double duration)
// {
// 	if (state == 0)
// 	{
// 		for (int i = 0; i < 4; i++)
// 		{
// 			pos_init[i] = pos_des[i];
// 			q_init[i] = q_des[i];
// 			Trajectory[i].reset_initial(time, pos_init[i], 0);
// 			Trajectory[i + 4].reset_initial(time, q_init[i], 0);
// 		}
// 		state = 1;
// 	}

// 	if (time >= duration - 0.002)
// 	{
// 		state = 0;
// 	}

// 	for (int i = 0; i < 4; i++)
// 	{
// 		Trajectory[i].update_time(time);
// 		Trajectory[i].update_goal(target[i], 0, duration);
// 		pos_des[i] = Trajectory[i].position_cubicSpline();
// 		vel_des[i] = Trajectory[i].velocity_cubicSpline();
// 	}

// 	std::cout << q_posdes[1] << std::endl;
// 	std::cout << q_posdes[2] << std::endl;
// 	std::cout << q_posdes[3] << std::endl;

// 	inverseKin(pos_des[1], pos_des[2], pos_des[3]);

// 	/*
// 	if (abs(q[3] - q_init[3]) >= PI)  // end effector가 회전이 최소한인 방향으로 회전하게 하는 if문
// 		{
// 			if (q[3] < 0)
// 			{
// 				q[3] = q[3] + 2 * PI;
// 			}
// 			else
// 			{
// 				q[3] = q[3] - 2 * PI;
// 			}
// 		}
// 	*/
// 	for (int i = 0; i < 4; i++)
// 	{
// 		q_posdes[i] = _q[i];
// 	}
// 	inverseKin(target[1], target[2], target[3]);


// 		for (int N = 0; N < 4; N++)
// 		{
// 			Trajectory[N + 4].update_time(time);
// 			q_goal[N] = q[N];

// 			qdot_goal[N] = 0;
// 			Trajectory[N + 4].update_goal(q_goal[N], qdot_goal[N], duration);
// 			q_des[N] = Trajectory[N + 4].position_cubicSpline();
// 			qdot_des[N] = Trajectory[N + 4].velocity_cubicSpline();
// 			q_des[0] = pos_des[0];
// 			q[0] = pos_des[0];

// 		}

// 		for (int i = 0; i < 4; i++)
// 		{
// 			if (CurrentState == pickup || CurrentState == dropit) // 원하는 지점까지의 position의 trajectory
// 			{
// 				calculate_joint_control_torque(0, i, q_posdes[i], qdot_des[i]);
// 			}
			
// 			else // 원하는 지점까지의 angle의 trajectory
// 			{
// 				calculate_joint_control_torque(0, i, q_des[i], qdot_des[i]);
// 			}
// 		}
		

// 		//cout << CurrentState << endl;
// 	}
