import rospy

from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from alliance.msg import motivation
from alliance.msg import message
from alliance.msg import to_monitor


class point:
    x = 0
    y = 0


class callBacks:
    robot = 0
    sim_time = 0
    odom = Odometry()
    sonar = PointCloud()

    def __init__(self, robot):
        self.robot = robot
        pt1 = point()

        for i in range(16):
            self.sonar.points.append(pt1)
            self.sonar.points[i].x = 2
            self.sonar.points[i].y = 2

    def timeCallBack(self, msg):
        self.sim_time = msg.data

    def odomCallBack(self, msg):
        self.odom = msg

    def sonarCallBack(self, msg):
        self.sonar = msg
