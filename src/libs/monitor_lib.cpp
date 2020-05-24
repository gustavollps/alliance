#include "../../include/alliance/monitor_lib.h"

/**
 * @brief Monitor::Monitor
 * @param nh
 */
Monitor::Monitor(ros::NodeHandle* nh, int robot_num, int task_num)
{
  nh_ = nh;

  alliance_calculation_ = nh_->subscribe<alliance::to_monitor>("/monitor_bus", 1, &Monitor::monitorCallback, this);
  printTimer_ = nh_->createTimer(ros::Duration(0.2), &Monitor::timerCallBack, this);

  robot_num_ = robot_num;
  task_num_ = task_num;

  msgs_.resize(robot_num);

  for (int i = 0; i < robot_num_; i++)
  {
    msgs_[i].acquiescence.clear();
    msgs_[i].activity_suppression.clear();
    msgs_[i].impatience_reset.clear();
    msgs_[i].impatience.clear();
    msgs_[i].sensory_feedback.clear();
    msgs_[i].motivation.clear();
    msgs_[i].pose_x = 0;
    msgs_[i].pose_y = 0;
    msgs_[i].robot = 255;
    msgs_[i].current_task = 255;
  }

  // system("clc");
}

/**
 * @brief Monitor::monitorCallback
 * @param msg
 */
void Monitor::monitorCallback(alliance::to_monitor msg)
{
  msgs_[msg.robot] = msg;
}

/**
 * @brief Monitor::timerCallBack
 * @param event
 */
void Monitor::timerCallBack(const ros::TimerEvent& event)
{
    printInfo();
}

/**
 * @brief Monitor::printInfo
 */
void Monitor::printInfo()
{
  system("clear");                        // clear screen
  for (int i = 0; i < msgs_.size(); i++)  // for each robot
  {
    std::cout << "Robot " << i << " at: " << msgs_[i].pose_x << "  -  " << msgs_[i].pose_y << "\n"
              << "Current task: ";
    if (msgs_[i].current_task == 0)
      std::cout << "IDLE" << std::endl;
    else if (msgs_[i].current_task == 1)
      std::cout << "WANDER RIGHT" << std::endl;
      else if (msgs_[i].current_task == 2)
      std::cout << "WANDER LEFT" << std::endl;
    else if (msgs_[i].current_task == 3)
      std::cout << "BOUNDARY_OVERWATCH" << std::endl;
    else if (msgs_[i].current_task == 4)
      std::cout << "REPORT" << std::endl;
    else
      std::cout << "UNKNOWN TASK "
                << "(" << msgs_[i].current_task << ")" << std::endl;

    std::cout << "Tasks info: " << std::endl;

    for (int j = 0; j < task_num_; j++)
    {
      if (j == 0)
        std::cout << "IDLE\t";
      else if (j == 1)
        std::cout << "W.RIGHT\t";
      else if (j == 2)
        std::cout << "W.LEFT\t";
      else if (j == 3)
        std::cout << "B.OVER.\t";
      else if (j == 4)
        std::cout << "REPORT\t";

      std::cout << "\tAcquiescence \tMotivation \tActivity Sup."
                << "\tImpatience \tImpatience R. \tSensory Feedback" << std::endl;

      if (!msgs_[i].acquiescence.empty())
        std::cout << "\t\t" << float(msgs_[i].acquiescence[j]) << "\t";
      if (!msgs_[i].motivation.empty())
        std::cout << "\t" << float(msgs_[i].motivation[j]) << "\t";
      if (!msgs_[i].activity_suppression.empty())
        std::cout << "\t" << float(msgs_[i].activity_suppression[j]) << "\t";
      if (!msgs_[i].impatience.empty())
        std::cout << "\t" << float(msgs_[i].impatience[j]) << "\t";
      if (!msgs_[i].impatience_reset.empty())
        std::cout << "\t" << float(msgs_[i].impatience_reset[j]) << "\t";
      if (!msgs_[i].sensory_feedback.empty())
        std::cout << "\t" << float(msgs_[i].sensory_feedback[j]) << "\t";

      std::cout << "\n";
    }

    std::cout << std::endl;
  }
}
