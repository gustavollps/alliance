import rospy
import time


def debug(robot):
    print('Motivation: \t' + str(robot.motivation))
    print('Active Time: \t' + str(robot.active_time))
    print('Suppression: \t' + str(robot.activity_suppression))
    print('Sen Feedback: \t' + str(robot.sensory_feedback))
    print('Impat. Reset: \t' + str(robot.impatience_reset))
    print('Acquiescence: \t' + str(robot.acquiescence))


class loopRate:
    last_loop = 0

    def __init__(self, last_loop):
        self.last_loop = last_loop

    def sleep(self, rate):
        time_now = rospy.get_time()
        T = float(1.0) / float(rate)
        T_rest = float(T) - float(time_now - self.last_loop)
        if T_rest > 0.0:
            time.sleep(T_rest)

        self.last_loop = self.last_loop + T
