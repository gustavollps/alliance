#include "ros/ros.h"
#include "alliance/to_monitor_l.h"

class Monitor_l
{
public:
  Monitor_l(ros::NodeHandle* nh, int robot_num, int task_num);

private:
  ros::NodeHandle* nh_;
  ros::Subscriber alliance_calculation_;
  ros::Timer printTimer_;

  void monitorCallback(alliance::to_monitor_l msg);
  void timerCallBack(const ros::TimerEvent& event);

  void printInfo();

  std::vector<alliance::to_monitor_l> msgs_;

  typedef enum{
    IDLE,
    WANDER,
    BOUNDARY_OVERWATCH,    
    REPORT
  }tasks;

  int robot_num_;
  int task_num_;
};
