import math
import random
from tf.transformations import euler_from_quaternion

turn = False
boundary_state = 0
avoiding = False
cicle = 0
wander_state = 0
points_list = [[3.75, 9.25],
               [3.75, 6.5],
               [1.5, 6.5],
               [1.5, 4.25],
               [3.75, 4.25],
               [3.75, -9],
               [-2.5, -9],
               [-2.5, -7.75],
               [-4, -7.5],
               [-4, 9.25],
               [-1.25, 9.25],
               [-1.25, 6.75],
               [1, 6.75],
               [1, 9.25]]


def idle(cmd_vel):
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    return cmd_vel


def wander(cmd_vel, sonar):
    minimum_vel = 0.5
    if sonar.points[3].x > 2:
        sonar.points[3].x = 2
    if sonar.points[4].x > 2:
        sonar.points[4].x = 2

    if sonar.points[2].x > 2:
        sonar.points[2].x = 2
    if sonar.points[5].x > 2:
        sonar.points[5].x = 2

    if sonar.points[1].x > 2:
        sonar.points[1].x = 2
    if sonar.points[6].x > 2:
        sonar.points[6].x = 2

    if sonar.points[3].x < 1 or sonar.points[4].x < 1:
        cmd_vel.linear.x = min(sonar.points[2].x, sonar.points[3].x, sonar.points[4].x, sonar.points[5].x) * 0.3
        cmd_vel.angular.z = (sonar.points[3].x - sonar.points[4].x) * \
                            (3 - min(sonar.points[3].x, sonar.points[4].x))
        if cmd_vel.angular.z < 0:
            if cmd_vel.angular.z > -minimum_vel * 2:
                cmd_vel.angular.z = -minimum_vel * 2
        if cmd_vel.angular.z > 0:
            if cmd_vel.angular.z < minimum_vel * 2:
                cmd_vel.angular.z = minimum_vel * 2

    elif sonar.points[1].x < 2 or sonar.points[6].x < 2:

        cmd_vel.linear.x = min(sonar.points[1].x, sonar.points[3].x, sonar.points[4].x, sonar.points[1].x) * 0.6
        cmd_vel.angular.z = (sonar.points[1].x - sonar.points[6].x) * 0.5
        if cmd_vel.angular.z < 0:
            if cmd_vel.angular.z > -minimum_vel * 0.5:
                cmd_vel.angular.z = -minimum_vel * 0.5
        if cmd_vel.angular.z > 0:
            if cmd_vel.angular.z < minimum_vel * 0.5:
                cmd_vel.angular.z = minimum_vel * 0.5

    else:
        cmd_vel.linear.x = min(sonar.points[1].x, sonar.points[3].x, sonar.points[4].x, sonar.points[6].x) * 0.6
        if sonar.points[3].x > 2:
            sonar.points[3].x = 2
        if sonar.points[4].x > 2:
            sonar.points[4].x = 2

        cmd_vel.angular.z = (sonar.points[3].x - sonar.points[4].x)
        if cmd_vel.angular.z < 0:
            if cmd_vel.angular.z > -minimum_vel:
                cmd_vel.angular.z = -minimum_vel
        if cmd_vel.angular.z > 0:
            if cmd_vel.angular.z < minimum_vel:
                cmd_vel.angular.z = minimum_vel
        # cmd_vel.angular.z = -0.25 + random.random()/2

    if sonar.points[2].x < 0.5 or sonar.points[3].x < 0.5 or \
            sonar.points[4].x < 0.5 or sonar.points[5].x < 0.5:
        cmd_vel.angular.z = 2
        cmd_vel.linear.x = 0

    # print(cmd_vel.angular.z)
    return cmd_vel


def wander_right(cmd_vel, sonar, odom):
    if odom.pose.pose.position.y > 0 and \
            sonar.points[3].x > 1 and sonar.points[4].x > 1 and \
            sonar.points[1].x > 0.7 and sonar.points[6].x > 0.7:
        # return to area
        quaternion = [odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w]
        odom_angle = euler_from_quaternion(quaternion)

        kp = 1.5
        cmd_vel.angular.z = (-math.pi / 2 - odom_angle[2]) * kp
        cmd_vel.linear.x = 0.5

    else:
        cmd_vel = wander(cmd_vel, sonar)

    return cmd_vel


def wander_left(cmd_vel, sonar, odom):
    if odom.pose.pose.position.y < 0 and \
            sonar.points[3].x > 1 and sonar.points[4].x > 1 and \
            sonar.points[1].x > 0.7 and sonar.points[6].x > 0.7:
        # return to area
        quaternion = [odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w]
        odom_angle = euler_from_quaternion(quaternion)

        kp = 1.5

        cmd_vel.angular.z = (math.pi / 2 - odom_angle[2]) * kp
        cmd_vel.linear.x = 0.5

    else:
        cmd_vel = wander(cmd_vel, sonar)

    return cmd_vel


def boundary_overwatch(cmd_vel, sonar, odom):
    global boundary_state
    global cicle
    global avoiding

    dist = 50
    done = False
    if not avoiding:
        if boundary_state == 0:
            for i in range(len(points_list)):
                if dist_calc(odom, points_list[i]) < dist:
                    dist = dist_calc(odom, points_list[i])
                    boundary_state = i + 1
                    if boundary_state > len(points_list) - 1:
                        boundary_state = 1
        else:
            cmd_vel.angular.z, dist, error_angle = facePoint(points_list[boundary_state - 1], odom)
            if error_angle > 15 * math.pi / 180:
                cmd_vel.linear.x = abs(math.pi / 2 - error_angle) / math.pi
            else:
                cmd_vel.linear.x = dist
            if dist_calc(odom, points_list[boundary_state - 1]) < 0.5:
                cmd_vel.linear.x = dist_calc(odom, points_list[boundary_state - 1]) + 0.3

            if dist_calc(odom, points_list[boundary_state - 1]) < 0.2:
                boundary_state += 1
                if boundary_state > len(points_list):
                    boundary_state = 1
                cicle += 1
                if cicle == len(points_list) + 1:
                    done = True

        if sonar.points[3].x < 1.5 or sonar.points[4].x < 1.5 or \
                sonar.points[2].x < 1 or sonar.points[5].x < 1:
            cmd_vel.linear.x = 0.3
        if sonar.points[3].x < 0.6 or sonar.points[4].x < 0.6 \
                or sonar.points[5].x < 0.6 or sonar.points[2].x < 0.6:
            avoiding = True

        # else:
        #        if sonar.points[3].x < 0.7 or sonar.points[4].x < 0.7 or \
        #                sonar.points[2].x < 0.7 or sonar.points[5].x < 0.7 or \
        #                sonar.points[0].y < 0.5 or sonar.points[7].x > -0.5 or \
        #                sonar.points[10].x < -0.7 or sonar.points[13].x < -0.7:
        #            cmd_vel = wander(cmd_vel, sonar)
    else:
        cmd_vel = wander(cmd_vel, sonar)
        if sonar.points[3].x < 0.7 or sonar.points[4].x < 0.7 or \
                sonar.points[2].x < 0.7 or sonar.points[5].x < 0.7 or \
                sonar.points[0].y < 0.5 or sonar.points[7].y > -0.5 or \
                sonar.points[10].x > -0.7 or sonar.points[13].x > -0.7:
            avoiding = True
        else:
            avoiding = False

    if cmd_vel.linear.x > 1.5:
        cmd_vel.linear.x = 1.5

    return cmd_vel, done


def report(cmd_vel, sonar, odom):
    if sonar.points[3].x > 1.2 and sonar.points[4].x > 1.2 \
            and sonar.points[5].x > 0.8 and sonar.points[2].x > 0.8:
        cmd_vel.angular.z, cmd_vel.linear.x, _ = facePoint([2, -7.5], odom)
        if cmd_vel.linear.x > 1:
            cmd_vel.linear.x = 1
    else:
        cmd_vel = wander(cmd_vel, sonar)

    if dist_calc(odom, [2, -7.5]) < 0.3:
        cmd_vel.angular.z = 3
        cmd_vel.linear.x = 0
    return cmd_vel


I = 0
error = 0
old_error = 0


def facePoint(point, odom):
    kp = 1.2
    ki = 0.0001
    kd = 0.3
    global I
    global error
    global old_error

    dx = -odom.pose.pose.position.x + point[0]
    dy = -odom.pose.pose.position.y + point[1]
    angle = math.atan2(dy, dx)

    quaternion = [odom.pose.pose.orientation.x,
                  odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z,
                  odom.pose.pose.orientation.w]
    odom_angle = euler_from_quaternion(quaternion)

    d_angle = angle - odom_angle[2]
    if abs(d_angle) > math.pi:
        if odom_angle[2] > 0:
            angle = angle + 2 * math.pi
        else:
            angle = angle - 2 * math.pi

    error = (angle - odom_angle[2])
    P = kp * error
    I = I + error * ki
    D = (error - old_error) * kd

    PID = P + I + D
    old_error = error
    # print(str(PID) + '\t' + str(error) + '\t' + str(P) + '\t' + str(I) + '\t' + str(D))
    # print(str(angle) + "\t" + str(odom_angle[2]))
    return PID, math.sqrt(dx ** 2 + dy ** 2), error


def dist_calc(odom, point):
    dx = -odom.pose.pose.position.x + point[0]
    dy = -odom.pose.pose.position.y + point[1]
    return math.sqrt(dx ** 2 + dy ** 2)
