#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <pthread.h>

int count = 0;

int getNumber() 
{
  int retNum;
  std::cout << "msg : ";
  std::cin >> retNum;
  // 잘못된 입력을 받았을 경우
  if (std::cin.fail())
  {  
    std::cout << "Wrong Number. Retry!" << std::endl; // 에러 메시지 출력
    std::cin.clear(); // 오류스트림을 초기화
    std::cin.ignore(256, '\n'); // 입력버퍼를 비움
    return getNumber(); // 함수를 재호출한다
  }
  return retNum;
}

void *rostest(void *data)
{  
  while(ros::ok())
  {
    count = getNumber();
  }
  pthread_exit(NULL);
}

int main(int argc, char **argv){
  pthread_t thread1 ;
  ros::init(argc, argv, "publisher");
	
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int32>("/numbers", 10);
    
  ros::Rate loop_rate(10);
    
  
  pthread_create(&thread1, NULL, &rostest,  (void*) &ctime);   
  while(ros::ok()){    
    
    std_msgs::Int32 msg;
  
    msg.data = count;
        
    //ROS_INFO("%d", msg.data);
        
    pub.publish(msg);
        
    ros::spinOnce();
        
    loop_rate.sleep();
  }
  pthread_detach(thread1);
  return (0);
}
