#include <ros/ros.h>
#include "../include/alliance/monitor_lib_l.h"

int main(int argc, char **argv)
{
  printf("argc %d\n", argc);
  for (int i = 0; i < argc; i++)
  {
    printf("argv[%d]:  %s\n", argc, argv[i]);
  }

  int robots, tasks;

  if(argc < 3){
    std::cout << "Using default values for robots (1) and taks (4)" << std::endl;
    robots = 3;
    tasks = 4;
  }
  else{
    robots = atoi(argv[1]);
    tasks = atoi(argv[2]);
  }

  ros::init(argc, argv, "Monitor");
  Monitor_l *monitor = new Monitor_l(new ros::NodeHandle,robots,tasks);
  ros::spin();
}
