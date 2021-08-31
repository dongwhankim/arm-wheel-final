
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
//#include <iostream>
#include <controller.h>
#include "dynamixel_sdk.h"
#include <sync_read_write2.h>                              // Uses Dynamixel SDK library
#include <linear_read_write.h>
#include <wheel.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#define PERIOD_NS 100000000  // 1Khz
#define SEC_IN_NSEC 1000000000

using namespace std;

double now_time , now_time2;

CController Control;
CRobot_Arm_TR RobotArm;
Clinear linear;
Cwheel wheel;
int tmpcnt = 0;
int tmpcnt2 = 0;
int linear_goal = 0;
int old_linear_goal = 0;
bool linear_play_state = 0;
bool dynamixel_state = 0;
int test = 0;
float axes[8];
int button[8];
int old_x_goal[5];
bool test_state = 1;
float Arr[7];
double linear_vel = 0.0;
double angular_vel = 0.0;
bool mode = 0;

void *dynamixelthread(void *data)
{
  while(1)
  {
    tmpcnt2++;
    RobotArm.RX();
    if(tmpcnt2 == 200)
    {
      linear.read_encoder();
      tmpcnt2 = 0;
    }
    if( dynamixel_state == 1)
    {
      RobotArm.TX();
      dynamixel_state = 0;
    }
    if(linear_play_state == 1)
    {
      linear.goalposition2Uchar(Control._x_goal[0]+1000000);
      linear_play_state = 0;
    }
    usleep(10);
  }
  RobotArm.end();
}

void KeyboardCallback(const std_msgs::Int32::ConstPtr& msg)
{
   //ROS_INFO("Recieved: [%d]", msg->data);
   //test = msg.data;
   test = msg->data;
}

void JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  for(int i = 0 ; i < 8 ; i++)
  {
    axes[i] = joy_msg->axes[i];
  }
    for(int i = 0 ; i < 8 ; i++)
  {
    button[i] = joy_msg->buttons[i];
  }
  if (button[0] == 1)
  {
    if(mode == 1)
    {
      mode = 0;
    }
    else
    {
      mode = 1;
    }
  }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  linear_vel = cmd_vel->linear.x;
  angular_vel = cmd_vel->angular.z;
}

void *ros_com(void *data)
{
 //ros::init(argc, argv, "subscriber");
  while(ros::ok())
  {
  ros::NodeHandle n;
  ros::NodeHandle nh_;
  ros::Subscriber sub2 = nh_.subscribe("joy", 10, JoystickCallback);
  ros::Subscriber sub = n.subscribe("cmd_vel",10,velCallback);
 // ros::Subscriber sub = n.subscribe("/numbers", 10, KeyboardCallback);
  ros::spin();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  pthread_t thread1, thread2;
  struct timespec ts;
  struct timespec ts2;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  clock_gettime(CLOCK_MONOTONIC, &ts2);
  int start_time = ts2.tv_sec;
  ros::NodeHandle n_;
  ros::Publisher pub_ = n_.advertise<std_msgs::Float32MultiArray>("velocity", 1000);
//////////////// initialize RS485
    RobotArm.start();
    RobotArm.RX();
    linear.Open_linear();
    wheel.Open_port();
   linear.goalposition2Uchar(-8000000);
   int cnt_timer = 0;
  cout << "Start RS485 Communication!" << endl << "Wait for 7.5 sec" <<endl<<endl;
   while(1)
   {
       while(ts.tv_nsec >= SEC_IN_NSEC)
         {
             ts.tv_sec++;
             ts.tv_nsec -= SEC_IN_NSEC;
         }
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
         ts.tv_nsec += 100000000;
       if(cnt_timer >= 75) //7.5sec
       {
         cnt_timer =0;
        break;
       }
       cnt_timer++;
   }
    RobotArm.goalposition(RobotArm._dxl_present_position);
    linear.homing();


// //////////////// RS485 communication
  pthread_create(&thread2, NULL, &dynamixelthread,  (void*) &ctime);
  pthread_create(&thread1, NULL, &ros_com, (void*) &ctime);
  while(ros::ok())
  {
     while(ts.tv_nsec >= SEC_IN_NSEC)
         {
             ts.tv_sec++;
             ts.tv_nsec -= SEC_IN_NSEC;
         }

         while(ts2.tv_nsec >= SEC_IN_NSEC)
         {
             ts2.tv_sec++;
             ts2.tv_nsec -= SEC_IN_NSEC;
         }

         old_linear_goal = linear_goal;
       for(int i = 0 ; i<5 ; i++)
       {
         old_x_goal[i] = Control._x_goal[i];
       }

         ts.tv_nsec +=PERIOD_NS;
         clock_gettime(CLOCK_MONOTONIC, &ts2);
         now_time = ts2.tv_sec - start_time;
         now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
         Control.get_present_position(RobotArm._dxl_present_position);
         Control.Finite_State_Machine(button[5],axes[2],axes[5]);
         RobotArm.goalposition(Control._x_goal);
         linear_goal = Control._x_goal[0];

         if(old_linear_goal == linear_goal)
         {
         }
         else
         {
           linear_play_state = 1;
         }

         for (int i = 0 ; i<5 ; i++)
         {
           if(old_x_goal[i] == Control._x_goal[i])
           {
           }
           else
           {
             dynamixel_state = 1;
           }
         }

          if(Control._new_mode == 1)
          {
            linear.homing();
            Control._new_mode = 0;
            std::cout<<Control._new_mode<<std::endl;
          }
	  if(mode == 0)
	  {
	    wheel.JoyStick_msg(axes,button);
	    //std::cout<<" Now : JoyStick Mode / Mode Change : Button 'A' " << std::endl;
	  }
	  else if(mode == 1)
	  {
	    wheel.velocity_target(linear_vel,angular_vel);
	    //std::cout<<" Now : SLAM MODE / Mode Change : Button 'A' " << std::endl;
	  }
          std_msgs::Float32MultiArray velocity;
          velocity.data.clear();
	  velocity.data.push_back(wheel._LeftVelocity_MS);
	  velocity.data.push_back(wheel._RightVelocity_MS);
	  velocity.data.push_back(wheel._LeftPosition);
	  velocity.data.push_back(wheel._RightPosition);
	  pub_.publish(velocity);
  }
  pthread_join(thread1,NULL);
  pthread_join(thread2,NULL);
  return (0);
}

