#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "alliance/message.h"
#include "alliance/motivation.h"
#include "alliance/to_monitor.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>
#include "nav_msgs/Odometry.h"
#include <sstream>

int id = 0;
double vel_x;
double vel_omega;
double k = 0.8;
// double alpha;
// int loop = 0;

ros::NodeHandle* nh;
ros::Publisher monitor_pub;
int robot_num;

typedef enum
{
  IDLE,
  WANDER,
  BOUNDARY_OVERWATCH,
  REPORT
} tasks;

tasks the_task = IDLE;

geometry_msgs::Twist vel;

double sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7;
// double sonar0x, sonar1x, sonar2x, sonar3x, sonar4x, sonar5x, sonar6x, sonar7x;
// double sonar0y, sonar1y, sonar2y, sonar3y, sonar4y, sonar5y, sonar6y, sonar7y;

// Distâncias do robô medidas pelo sonar
double vcloser = 1.0;
double closer = 1.3;
double inter = 1.6;
double far = 2.5;
double vfar = 3.0;

// Definição das variáveis envolvidas na arquitetura alliance
// Threshold de ativação de um comportamento
int theta = 2000;
// Variável de entrada que avalia a partir dos sensores se o comportamento é aplicável (true) naquele instante de tempo
int sensory_feedback[4] = { 1, 1, 0, 1 };
// Variável que define a frequência de envio de mensagens por unidade de tempo do robô1, de acordo com sua atividade
float rho1 = 5;
// Variável que define quanto tempo o robô1 pode passar sem receber mensagens de outros robôs, antes de assumir que o
// determinado robô cessou sua atividade
double tau1 = 100.0;
// Variável que define se determinado comportamento deve ser suprimido dado que outro comportamento está ativo
int activity_suppression[4];
// Variável de comunicação recebida de cada robô sobre cada tarefa(comportamento)
int comm[4][3], comm_phi[4][3];

// Variáveis de impaciência do robô
// Variável de tempo que determina quanto tempo o robô1 está disposto a permitir que as mensagens enviadas pelos outros
// robôs afetem a motivação do comportamento
double phi = 360;
// Variável que determina uma variação lenta na impaciência, após o robô1 identificar que outro robô está realizando uma
// tarefa
double delta_slow[4] = { 1.0, 2.5, 5.0, 2.0 };
// Variável que determina uma variação rápida na impaciência, quando não há robô realizando determinada tarefa
double delta_fast[4] = { 1.0, 5.0, 10.0, 3.0 };
// Variável de impaciência
double impatience[4] = { 1.0, 5.0, 10.0, 3.0 };
// Variável de reset de impaciência
int impatience_reset[4] = { 1, 1, 1, 1 };
// int task_counter[4][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Variáveis de aquiescência
// Variável que determina o tempo que o robô1 manterá um comportamento antes de passá-la a outro robô
int psi = 120;  // 2 minutos
// Variável de aquiescência
int acquiescence[4] = { 1, 1, 1, 1 };
// Variável que determina o tempo que o robô1 manterá um comportamento antes tentar realizar outro
double lambda[4] = { 720, 420, 420, 420 };

// Motivação
float m1[4] = { 0.0, 0.0, 0.0, 0.0 };
// Motivação anterior
float m1_old[4] = { 0.0, 0.0, 0.0, 0.0 };

int current_task, my_task;
int current_robot;
int current_behaviour = 0;
int timenow;
int time_old[4][3];
int diff_time[4][3] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int active_time[4] = { 0, 0, 0, 0 };
int j = 0;

geometry_msgs::Pose current_pos;

alliance::message taskmsg;
alliance::motivation motiv;
alliance::to_monitor monitor_msg;

bool msg_filled = false;

// Rotina de cálculo dos ângulos Beta[1, 2, 3, 4] para o behaviour de BOUNDARY OVERWATCH
/*double beta_calc (double delta, double d2, double x1, double y1, double x2, double y2)
{
   double hc, a, beta;

   hc = d2*sin(delta);
   a = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
   beta = (asin(hc/a))*(180.0/M_PI);

   return beta;
}*/

/*double alpha_calc(double delta, double d1, double d2, double x1, double y1, double x2, double y2)
{
   double alpha, threshold, h, a, angle;

   threshold = d2*sin(delta);

   if (sonar5 < sonar0)
   {
    if (threshold <= d1)
    {
    alpha = 90.0 - beta_calc(delta, d2, x1, y1, x2, y2);
    ROS_INFO("Beta1 %0.2f.", beta_calc(delta, d2, x1, y1, x2, y2));
    ROS_INFO("Alpha1: %0.2f.", alpha);
    }
    else if(threshold > d1)
    {
    a = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
    angle = acos((pow(a,2)+pow(d2,2)-pow(d1,2))/(2+a+d2));
    h = d2*sin(angle);
    alpha = -(acos(h/d1))*(180/M_PI);
    ROS_INFO("Alpha2: %0.2f.", alpha);
    }
   }
   else if(sonar5 > sonar0)
   {
    if (threshold <= d1)
    {
    alpha = beta_calc(delta, d2, x1, y1, x2, y2) - 90.0;
    ROS_INFO("Alpha3: %0.2f.", alpha);
    }
    else if(threshold > d1)
    {
    a = sqrt(pow((x1-x2),2)+pow((y1-y2),2));
    angle = acos((pow(a,2)+pow(d2,2)-pow(d1,2))/(2+a+d2));
    h = d2*sin(angle);
    alpha = (acos(h/d1))*(180/M_PI);
    ROS_INFO("Alpha4: %0.2f.", alpha);
    }
   }
   ROS_INFO("Alpha vale %0.3f.", alpha);
   return alpha;
}*/

void task_filler()
{
  taskmsg.task_id = current_behaviour;
  taskmsg.robot_id = id;
}

void report()
{
  // ROS_WARN("REPORT: Tasks WANDER and BOUNDARY OVERWATCH are being done.");
  the_task = REPORT;

  vel_x = 0.1;
  vel_omega = 0.5;

  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;

  task_filler();
}

void boundary_overwatch()
{
  // ROS_WARN("REPORT: Task BOUNDARY OVERWATCH is being done.");
  the_task = BOUNDARY_OVERWATCH;

  // Achando a parede
  if (sonar0 > far and sonar1 > far)
  {
    vel_x = 0.6;
    vel_omega = -0.05;
  }
  // Seguindo a parede
  else if (sonar0 < closer and sonar1 > closer and sonar1 < inter)
  {
    vel_x = 0.2;
    vel_omega = -0.4;
  }
  else if (sonar0 > inter and sonar1 > far)
  {
    vel_x = 0.2;
    vel_omega = 0.4;
  }
  else if (sonar2 < inter and sonar3 < far and sonar1 < far)
  {
    vel_x = 0.0;
    vel_omega = -0.5;
  }
  else if (sonar3 < inter and sonar2 < far and sonar4 < far)
  {
    vel_x = 0.0;
    vel_omega = 0.5;
  }
  else if (sonar0 > far and sonar7 < closer)
  {
    vel_x = 0.1;
    vel_omega = 1.0;
  }
  else if (sonar0 < vcloser and sonar1 < vcloser and sonar2 < vcloser and sonar7 < vcloser)
  {
    vel_x = -0.3;
    vel_omega = -0.15;
  }
  else if (sonar5 > inter and sonar0 > far)
  {
    vel_x = 0.4;
    vel_omega = 0.2;
  }
  else
  {
    vel_x = 0.4;
    vel_omega = 0.0;
  }
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
}

void wander()
// WANDER: robô vaga desviando de obstáculos
{
  // ROS_WARN("REPORT: Task WANDER is being done.");
  the_task = WANDER;

  vel_x = 0.9 * k * ((std::max(sonar2, sonar3)) - (1 / std::max(sonar2, sonar3)));
  vel_omega = k * (std::min((sonar0 / 2), std::min((sonar1 / 2), (sonar2 / 2))) -
                   (std::min((sonar3 / 2), std::min((sonar4 / 2), (sonar5 / 2)))));
  ROS_INFO("Vel_omega %0.2f.", vel_omega);

  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
}

void suppression()
{
  if (my_task == 0 or acquiescence[my_task] == 0)
  {
    ROS_INFO("Suppression in");
    activity_suppression[0] = 1;
    activity_suppression[1] = 1;
    activity_suppression[2] = 1;
    activity_suppression[3] = 1;
    impatience_reset[0] = 1;
    impatience_reset[1] = 1;
    impatience_reset[2] = 1;
    impatience_reset[3] = 1;
  }
  else  // if (current_task != 0)
  {
    for (int i = 1; i <= 3; i++)
    {
      if (my_task == i)
      {
        activity_suppression[i] = 1;
      }
      else
      {
        activity_suppression[i] = 0;
      }
    }
  }

  if (!msg_filled)
  {
    for (int i = 0; i < 4; i++)
    {
      monitor_msg.acquiescence.push_back(acquiescence[i]);
      monitor_msg.impatience.push_back(impatience[i]);
      monitor_msg.impatience_reset.push_back(impatience_reset[i]);
      monitor_msg.motivation.push_back(m1[i]);
      monitor_msg.activity_suppression.push_back(activity_suppression[i]);
      monitor_msg.sensory_feedback.push_back(sensory_feedback[i]);
    }
    monitor_msg.stamp = ros::Time::now();
    monitor_msg.robot = robot_num;
    monitor_msg.current_task = the_task;
    monitor_msg.pose_x = current_pos.position.x;
    monitor_msg.pose_y = current_pos.position.y;
    msg_filled = true;
  }
}

void idle()
{
  // ROS_ERROR("REPORT: Task IDLE is being done.");
  the_task = IDLE;

  vel_x = 0;
  vel_omega = 0;
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;

  suppression();
}

void behaviour_set()
{
  // ROS_INFO("Behaviour_set");
  for (int i = 0; i <= 3; i++)
  {
    if (m1[i] >= theta)
    {
      current_behaviour = i;
      m1_old[i] = theta;
    }
    else
    {
      active_time[i] = 0;
    }
  }
  suppression();
}

void motivation_calc()
{
  // ROS_INFO("Motivation_calc");
  for (int i = 0; i <= 3; i++)
  {
    m1[i] = (m1_old[i] + impatience[i]) * sensory_feedback[i] * activity_suppression[i] * impatience_reset[i] *
            acquiescence[i];
    m1_old[i] = m1[i];
    ROS_INFO("impatience %d %0.2f", i, impatience[i]);
    ROS_INFO("sensory%d %d", i, sensory_feedback[i]);
    ROS_INFO("Suppression%d %d", i, activity_suppression[i]);
    ROS_INFO("Impatience Reset%d %d", i, impatience_reset[i]);
    ROS_INFO("Acquiescence%d %d", i, acquiescence[i]);
    ROS_INFO("m1 old%d %0.2f", i, m1_old[i]);
    ROS_INFO("%0.2f", m1[i]);
  }

  behaviour_set();
}

void acquiescence_calc()
{
  // ROS_INFO("Acquiescence_calc");
  if (((active_time[current_task] > psi) and (comm[current_task][current_robot] == 1)) or
      (active_time[current_task] > lambda[current_task]))
  {
    acquiescence[current_task] = 0;
    activity_suppression[0] = 1;
    activity_suppression[1] = 0;
    activity_suppression[2] = 0;
    activity_suppression[3] = 0;
    impatience[current_task] = delta_slow[current_task];
  }
  else
  {
    acquiescence[current_task] = 1;
  }
  motivation_calc();
}

void impatience_calc()
{
  // ROS_INFO("Impatience_calc");
  if ((comm[current_task][current_robot] == 1) and (comm_phi[current_task][current_robot] == 0))
  {
    impatience[current_task] = delta_slow[current_task];
  }
  else
  {
    impatience[current_task] = delta_fast[current_task];
  }

  acquiescence_calc();
}

void comm_received()
{
  // ROS_INFO("comm_received");
  timenow = ros::Time::now().toSec();
  if (current_robot == id)
  {
    my_task = current_task;
    comm[current_task][current_robot] = 1;
    if (time_old[current_task][current_robot] == 0)
    {
      time_old[current_task][current_robot] = timenow;
      // j = j++;
    }
    diff_time[current_task][current_robot] = timenow - time_old[current_task][current_robot];
    active_time[current_task] = active_time[current_task] + diff_time[current_task][current_robot];
    /*ROS_INFO("Tarefa atual e %d.", my_task);
    ROS_INFO("Robô atual e %d.", current_robot);
    ROS_INFO("Diff_time atual e %d.", diff_time[current_task][current_robot]);
    ROS_INFO("time_old de %d.", time_old[current_task][current_robot]);
    ROS_INFO("Timenow %d.", timenow);
    ROS_INFO("Tempo de %d ativo e %d.", current_task, active_time[current_task]);
    //}*/
    time_old[current_task][current_robot] = timenow;
  }
  else
  {
    // task_counter[current_task][current_robot] = task_counter[current_task][current_robot] + 1;

    if (time_old[current_task][current_robot] == 0)
    {
      time_old[current_task][current_robot] = timenow;
      // j = j++;
    }
    diff_time[current_task][current_robot] = timenow - time_old[current_task][current_robot];
    if (diff_time[current_task][current_robot] < tau1)
    {
      comm[current_task][current_robot] = 1;
      // if (task_counter[current_task][current_robot] == 1)
      //{
      impatience_reset[current_task] = 0;
      //}
    }
    else
    {
      comm[current_task][current_robot] = 0;
      // if (task_counter[current_task][current_robot] > 1 and diff_time[current_task][current_robot] > tau1)
      //{
      impatience_reset[current_task] = 1;
      //}
    }
    if (diff_time[current_task][current_robot] < phi)
    {
      comm_phi[current_task][current_robot] = 0;
    }
    else if (diff_time[current_task][current_robot] > phi)
    {
      comm_phi[current_task][current_robot] = 1;
      impatience_reset[current_task] = 1;
    }

    time_old[current_task][current_robot] = timenow;
  }

  impatience_calc();
}

void TaskCallback(alliance::message taskmsg)
{
  // ROS_INFO("TaskCallback");
  current_task = taskmsg.task_id;
  current_robot = taskmsg.robot_id;

  comm_received();
}

void OdomCallback(nav_msgs::Odometry msg)
{
  current_pos = msg.pose.pose;
}

void SonarCallback(sensor_msgs::PointCloud cloud)
{
  sonar0 = sqrt(cloud.points[0].y * cloud.points[0].y);
  // sonar0x= cloud.points[0].x;
  // sonar0y= cloud.points[0].y;
  sonar1 = sqrt((cloud.points[1].x * cloud.points[1].x) + (cloud.points[1].y * cloud.points[1].y));
  // sonar1x= cloud.points[1].x;
  // sonar1y= cloud.points[1].y;
  sonar2 = sqrt((cloud.points[2].x * cloud.points[2].x) + (cloud.points[2].y * cloud.points[2].y));
  // sonar1x= cloud.points[2].x;
  // sonar1y= cloud.points[2].y;
  sonar3 = sqrt((cloud.points[3].x * cloud.points[3].x) + (cloud.points[3].y * cloud.points[3].y));
  // sonar1x= cloud.points[3].x;
  // sonar1y= cloud.points[3].y;
  sonar4 = sqrt((cloud.points[4].x * cloud.points[4].x) + (cloud.points[4].y * cloud.points[4].y));
  // sonar4x= cloud.points[4].x;
  // sonar4y= cloud.points[4].y;
  sonar5 = sqrt(cloud.points[5].y * cloud.points[5].y);
  // sonar5x= cloud.points[5].x;
  // sonar5y= cloud.points[5].y;
  sonar6 = sqrt((cloud.points[6].x * cloud.points[6].x) + (cloud.points[6].y * cloud.points[6].y));
  // sonar6x= cloud.points[6].x;
  // sonar6y= cloud.points[6].y;
  sonar7 = sqrt((cloud.points[7].x * cloud.points[7].x) + (cloud.points[7].y * cloud.points[7].y));
  // sonar7x= cloud.points[7].x;
  // sonar7y= cloud.points[7].y;
  ROS_INFO("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, sonar6,
           sonar7);
}

void getParams()
{
  int var_helper = 0;

  std::stringstream temp;
  temp << "/Robot" << robot_num << "/sensory_feedback";
  if (!nh->getParam(temp.str(), var_helper))
  {
    sensory_feedback[0] = 1;
    sensory_feedback[1] = 1;
    sensory_feedback[2] = 1;
    sensory_feedback[3] = 1;
    ROS_ERROR("No sensory feedback param for robot %d!", robot_num);
  }
  else
  {
    sensory_feedback[0] = var_helper / 1000;
    sensory_feedback[1] = (var_helper - sensory_feedback[0] * 1000) / 100;
    sensory_feedback[2] = (var_helper - sensory_feedback[1] * 100 - sensory_feedback[0] * 1000) / 10;
    sensory_feedback[3] =
      (var_helper - sensory_feedback[2] * 10 - sensory_feedback[1] * 100 - sensory_feedback[0] * 1000);
  }

  temp.str("");
  temp << "/Robot" << robot_num << "/tau";
  if (!nh->getParam(temp.str(), tau1))
  {
    tau1 = 5;
    ROS_ERROR("No tau param for robot %d!", robot_num);
  }

  temp.str("");
  temp << "/Robot" << robot_num << "/rho";
  if (!nh->getParam(temp.str(), rho1))
  {
    rho1 = 5;
    ROS_ERROR("No rho param for robot %d!", robot_num);
  }

  temp.str("");
  temp << "/Robot" << robot_num << "/id";
  if (!nh->getParam(temp.str(), id))
  {
    ROS_ERROR("No id param for robot %d!", robot_num);
    ros::shutdown();
  }
}

void timerCallBack(const ros::TimerEvent& event)
{
  if (msg_filled)
  {
    monitor_pub.publish(monitor_msg);
    monitor_msg.acquiescence.clear();
    monitor_msg.activity_suppression.clear();
    monitor_msg.motivation.clear();
    monitor_msg.impatience_reset.clear();
    monitor_msg.impatience.clear();
    monitor_msg.sensory_feedback.clear();
    msg_filled = false;
  }
}

int main(int argc, char** argv)
{
  printf("argc %d\n", argc);
  for (int i = 0; i < argc; i++)
  {
    printf("argv[%d]:  %s\n", argc, argv[i]);
  }

  robot_num = atoi(argv[1]);
  std::cout << "atoi :" << robot_num << std::endl;

  std::stringstream node_name;
  node_name << "alliance_bot" << robot_num;
  ros::init(argc, argv, node_name.str());
  nh = new ros::NodeHandle();

  getParams();
  ros::Publisher task_pub = nh->advertise<alliance::message>("TaskAllocation", 5);

  std::stringstream robot_name;
  robot_name << "/p3dx_" << robot_num;

  std::stringstream str_t;
  str_t << robot_name.str() << "/cmd_vel";
  ros::Publisher cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(str_t.str(), 1);

  monitor_pub = nh->advertise<alliance::to_monitor>("monitor_bus", 1);
  ros::Timer monitor_timer = nh->createTimer(ros::Duration(0.1), timerCallBack);

  str_t.str(std::string());
  str_t << "Motivation_robot" << robot_num;
  ros::Publisher motivation_pub = nh->advertise<alliance::motivation>(str_t.str(), 1);

  ros::Subscriber tasksub = nh->subscribe<alliance::message>("TaskAllocation", 5, TaskCallback);

  str_t.str(std::string());
  str_t << robot_name.str() << "/pose";
  ros::Subscriber odomsub = nh->subscribe<nav_msgs::Odometry>(str_t.str(), 10, OdomCallback);

  str_t.str(std::string());
  str_t << robot_name.str() << "/sonar";
  ros::Subscriber sonarsub = nh->subscribe<sensor_msgs::PointCloud>(str_t.str(), 10, SonarCallback);

  ros::Rate loop_rate(rho1);
  while (ros::ok())
  {
    if (current_behaviour == 0)
    {
      idle();
      cmd_vel_pub.publish(vel);

      task_filler();
      task_pub.publish(taskmsg);

      rho1 = 10;
    }
    if (current_behaviour == 1)
    {
      wander();
      cmd_vel_pub.publish(vel);

      task_filler();
      task_pub.publish(taskmsg);

      rho1 = 2;
    }
    if (current_behaviour == 2)
    {
      boundary_overwatch();
      cmd_vel_pub.publish(vel);

      task_filler();
      task_pub.publish(taskmsg);

      rho1 = 2;
    }
    if (current_behaviour == 3)
    {
      report();
      cmd_vel_pub.publish(vel);

      task_filler();
      task_pub.publish(taskmsg);

      rho1 = 0.5;
    }

    motiv.task_id = 0;
    motiv.motivation = m1[0];

    motivation_pub.publish(motiv);

    motiv.task_id = 1;
    motiv.motivation = m1[1];

    motivation_pub.publish(motiv);

    motiv.task_id = 2;
    motiv.motivation = m1[2];

    motivation_pub.publish(motiv);

    motiv.task_id = 3;
    motiv.motivation = m1[3];

    motivation_pub.publish(motiv);

    /*for (int i = 0; i <=3; i++)
    {
   motiv.task_id = i;
   motiv.motivation = m1[i];

   motivation_pub.publish(motiv);
    }*/

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
