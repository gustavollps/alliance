#!/usr/bin/env python

import sys
import rospy
from enum import Enum

from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from alliance_class import Alliance
from alliance.msg import motivation
from alliance.msg import message
from alliance.msg import to_monitor

from callBack import callBacks
from utils import loopRate
from utils import debug
import tasks
import random
import time
import math
import copy

robot_id = 0
if len(sys.argv) > 1:
    robot_id = sys.argv[1]

time.sleep(random.random())


class Tasks(Enum):
    IDLE = 0
    WANDER = 1
    BOUNDARY_OVERWATCH = 2
    REPORT = 3


rospy.init_node("robot_0" + str(robot_id))

# tau -> time without msg to reset impatience
# psi -> aquiescence
# phi
robot_00 = Alliance(robot_id=robot_id,
                    theta=2000, rho=7, tau=150, psi=180, phi=360,
                    task_num=5, robot_num=3,
                    sensory_feedback=[1, 1, 1, 1, 1],
                    delta_slow=[1.0, 2.5, 2.5, 5.0, 2.0],
                    delta_fast=[1.0, 5.0, 5.0, 10.0, 3.0],
                    impatience=[1.0, 5.0, 5.0, 10.0, 3.0],
                    lambda_task=[720, 420, 420, 420, 420],
                    acquiescence=[1, 1, 1, 1, 1])


def taskCallBack(msg):
    time_now = rospy.get_time()
    robot_00.comm_received(time_now, msg.task_id, msg.robot_id)


def missionCallBack(msg):
    robot_00.sensory_feedback[int(msg.data)] = 0
    # print("Task Concluded: " + str(msg.data))


def task_filler():
    task_msg.task_id = robot_00.current_behaviour
    task_msg.robot_id = robot_00.robot_id


def timerCallBack(event):
    global pub_vel
    global pub_mon
    global msg_filled
    global dist_wander
    global old_odom
    global pub_mission
    global wander_goal
    global regions
    global region
    global old_region
    global report_time

    msg = publish_monitor()
    pub_mon.publish(msg)
    pub_mon_single.publish(msg)
    msg_filled = False
    robot_00.update()
    cmd_vel = Twist()
    cmd_vel.angular.z = 0
    cmd_vel.linear.x = 0

    if robot_00.current_behaviour == 0:
        # print('Idle')
        cmd_vel = tasks.idle(cmd_vel)
        robot_00.suppression()
        robot_00.rho = 10
        dist_wander = 0
        regions = 0
        region = 0
        old_region = 0
        report_time = 0
        cmd_vel.angular.z = 0
        cmd_vel.linear.x = 0

    elif robot_00.current_behaviour == 1:
        # print('Wander')
        cmd_vel = tasks.wander_right(cmd_vel, call_backs.sonar, call_backs.odom)
        robot_00.rho = 2
        dist_wander = dist_wander + \
                      math.sqrt((call_backs.odom.pose.pose.position.x - old_odom.pose.pose.position.x) ** 2 +
                                (call_backs.odom.pose.pose.position.y - old_odom.pose.pose.position.y) ** 2)
        old_odom = call_backs.odom
        if dist_wander > wander_goal:
            pub_mission.publish(1)
            robot_00.current_behaviour = 0

    elif robot_00.current_behaviour == 2:
        # print('Wander')
        cmd_vel = tasks.wander_left(cmd_vel, call_backs.sonar, call_backs.odom)
        robot_00.rho = 2
        dist_wander = dist_wander + \
                      math.sqrt((call_backs.odom.pose.pose.position.x - old_odom.pose.pose.position.x) ** 2 +
                                (call_backs.odom.pose.pose.position.y - old_odom.pose.pose.position.y) ** 2)
        old_odom = call_backs.odom
        if dist_wander > wander_goal:
            pub_mission.publish(2)
            robot_00.current_behaviour = 0

    elif robot_00.current_behaviour == 3:
        # print('Boundary_Overwatch')
        cmd_vel, done = tasks.boundary_overwatch(cmd_vel, call_backs.sonar, call_backs.odom)
        robot_00.rho = 2
        if done:
            pub_mission.publish(3)
            robot_00.current_behaviour = 0

    elif robot_00.current_behaviour == 4:
        # print('Report')
        cmd_vel = tasks.report(cmd_vel, call_backs.sonar, call_backs.odom)
        robot_00.rho = 0.5
        report_time = report_time + 0.02
        if report_time > 30:
            pub_mission.publish(4)
            robot_00.current_behaviour = 0

    pub_vel.publish(cmd_vel)


def getParams():
    temp = "/Robot" + str(robot_id) + "/sensory_feedback"

    robot_00.sensory_feedback = rospy.get_param(temp, [1, 1, 1, 1, 1])

    temp = "/Robot" + str(robot_id) + "/tau"
    robot_00.tau = rospy.get_param(temp, 5)

    temp = "/Robot" + str(robot_id) + "/rho"
    robot_00.rho = rospy.get_param(temp, 5)

    temp = "/Robot" + str(robot_id) + "/id"
    robot_00.robot_id = rospy.get_param(temp, robot_id)

    temp = "/Robot" + str(robot_id) + "/delta"
    delta_list = rospy.get_param(temp, 5)
    delta_fast = copy.copy(delta_list)
    delta_slow = [i*2 for i in delta_list]
    impatience = copy.copy(delta_slow)
    robot_00.delta_fast = delta_fast
    robot_00.delta_slow = delta_slow
    robot_00.impatience = impatience


def publish_monitor():
    global msg_filled
    global odom
    global pub_mon
    monitor_msg = to_monitor()
    if not msg_filled:
        monitor_msg.acquiescence = robot_00.acquiescence
        monitor_msg.impatience = robot_00.impatience
        monitor_msg.impatience_reset = robot_00.impatience_reset
        monitor_msg.motivation = robot_00.motivation
        monitor_msg.activity_suppression = robot_00.activity_suppression
        monitor_msg.sensory_feedback = robot_00.sensory_feedback

        monitor_msg.stamp = rospy.get_rostime()
        monitor_msg.robot = robot_00.robot_id
        monitor_msg.current_task = robot_00.my_task
        monitor_msg.pose_x = odom.pose.pose.position.x
        monitor_msg.pose_y = odom.pose.pose.position.y
        msg_filled = True
    return monitor_msg


msg_filled = False
sim_time = 0
odom = Odometry()
old_odom = Odometry()
sonar = PointCloud()
task_msg = message()
motivation_msg = motivation()


getParams()
call_backs = callBacks(robot_00)

sub_task = rospy.Subscriber("/task_allocation", message, taskCallBack)
sub_time = rospy.Subscriber("/p3dx_" + str(robot_id) + "/simTime", Float32, call_backs.timeCallBack)
sub_odom = rospy.Subscriber("/p3dx_" + str(robot_id) + "/pose", Odometry, call_backs.odomCallBack)
sub_sonar = rospy.Subscriber("/p3dx_" + str(robot_id) + "/sonar", PointCloud, call_backs.sonarCallBack)

pub_vel = rospy.Publisher("/p3dx_" + str(robot_id) + "/cmd_vel", Twist, queue_size=10)
pub_mot = rospy.Publisher("/p3dx_" + str(robot_id) + "/motivation", motivation, queue_size=10)
pub_task = rospy.Publisher("/task_allocation", message, queue_size=10)
pub_mission = rospy.Publisher("/mission", Float32, queue_size=10)
sub_mission = rospy.Subscriber("/mission", Float32, missionCallBack)
pub_mon = rospy.Publisher("monitor_bus", to_monitor, queue_size=10)
pub_mon_single = rospy.Publisher("monitor_bus" + str(robot_id), to_monitor, queue_size=10)
timer = rospy.Timer(rospy.Duration(0.02), timerCallBack)

rate = loopRate(rospy.get_time())

# wander
dist_wander = 0
wander_goal = 50

# report
report_time = 0

while not rospy.is_shutdown():

    if robot_00.current_behaviour == 0:
        # print('Idle')
        robot_00.suppression()
        robot_00.rho = 10

    elif robot_00.current_behaviour == 1:
        # print('Wander Right')
        robot_00.rho = 2

    elif robot_00.current_behaviour == 2:
        # print('Wander Left')
        robot_00.rho = 2

    elif robot_00.current_behaviour == 3:
        # print('Boundary_Overwatch')
        robot_00.rho = 2

    elif robot_00.current_behaviour == 4:
        # print('Report')
        robot_00.rho = 0.5

    # debug(robot_00)
    # print(regions)
    task_filler()
    pub_task.publish(task_msg)
    rate.sleep(robot_00.rho)
