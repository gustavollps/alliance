import rospy


class Alliance:
    robot_id = 0  # robot id

    theta = 0  # threshold for behaviour activation
    sensory_feedback = []  # len = task_num -> sensory capacity for a task
    rho = 0  # msg
    tau = 0
    activity_suppression = []
    comm = []
    comm_phi = []
    phi = 0
    delta_slow = []  # len = task_num -> slow impatience increment for a active task
    delta_fast = []  # len = task_num -> fast impatience increment for a inactive task
    impatience = []  # len = task_num -> impatience for task
    impatience_reset = []
    psi = 0  # time interval until a robot quit the behaviour
    acquiescence = []  # len = task_num -> represents the robot self aware capacity for a task
    lambda_task = []  # time one should wait to suppress a task
    motivation = []
    motivation_old = []

    time_now = 0
    time_old = []
    time_dif = []
    active_time = []

    robot_num = 0  # number of robots on the process
    task_num = 0  # number of tasks

    current_task = 0
    my_task = 0

    current_robot = 0
    current_behaviour = 0

    def __init__(self, theta, sensory_feedback, rho, tau, psi, phi, task_num, robot_num, robot_id,
                 delta_slow, delta_fast, impatience, lambda_task, acquiescence):

        self.robot_id = robot_id

        self.theta = theta
        self.task_num = task_num

        self.tau = tau
        self.rho = rho
        self.psi = psi
        self.phi = phi
        self.delta_fast = delta_fast
        self.delta_slow = delta_slow

        self.impatience = impatience
        self.sensory_feedback = sensory_feedback

        self.activity_suppression = [0] * task_num
        self.lambda_task = lambda_task
        self.robot_num = robot_num

        self.motivation = [0] * task_num
        self.motivation_old = [0] * task_num

        self.acquiescence = acquiescence
        self.impatience_reset = [0] * task_num

        self.active_time = [0] * task_num

        for i in range(task_num):
            self.comm.append([])
            self.comm_phi.append([])
            self.time_old.append([])
            self.time_dif.append([])

            for j in range(robot_num):
                self.comm[i].append(0)
                self.time_old[i].append(0)
                self.time_dif[i].append(0)
                self.comm_phi[i].append(0)

    def comm_received(self, time, current_task, current_robot):
        timenow = time
        self.current_task = current_task
        self.current_robot = current_robot
        if current_robot == self.robot_id:
            self.my_task = current_task
            self.comm[current_task][current_robot] = 1
            if self.time_old[current_task][current_robot] == 0:
                self.time_old[current_task][current_robot] = timenow

            self.time_dif[current_task][current_robot] = timenow - self.time_old[current_task][current_robot]
            self.active_time[current_task] = self.active_time[current_task] + self.time_dif[current_task][current_robot]
            # ROS_INFO("Tarefa atual e %d.", my_task)
            # ROS_INFO("Robo atual e %d.", current_robot)
            # ROS_INFO("Diff_time atual e %d.", diff_time[current_task][current_robot]);
            # ROS_INFO("time_old de %d.", time_old[current_task][current_robot]);
            # ROS_INFO("Timenow %d.", timenow);
            # ROS_INFO("Tempo de %d ativo e %d.", current_task, active_time[current_task]);

            self.time_old[current_task][current_robot] = timenow
        else:
            if self.time_old[current_task][current_robot] == 0:
                self.time_old[current_task][current_robot] = timenow

            self.time_dif[current_task][current_robot] = timenow - self.time_old[current_task][current_robot]

            if self.time_dif[current_task][current_robot] < self.tau:
                self.comm[current_task][current_robot] = 1
                self.impatience_reset[current_task] = 0
                if self.current_behaviour == current_task:
                    self.current_behaviour = 0
            else:
                self.comm[current_task][current_robot] = 0
                self.impatience_reset[current_task] = 1

            if self.time_dif[current_task][current_robot] < self.phi:
                self.comm_phi[current_task][current_robot] = 0

            elif self.time_dif[current_task][current_robot] > self.phi:
                self.comm_phi[current_task][current_robot] = 1
                self.impatience_reset[current_task] = 1

            self.time_old[current_task][current_robot] = timenow

        self.impatience_calc()

    def impatience_calc(self):
        if self.comm[self.current_task][self.current_robot] == 1 and \
                self.comm_phi[self.current_task][self.current_robot] == 0:
            self.impatience[self.current_task] = self.delta_slow[self.current_task]
        else:
            self.impatience[self.current_task] = self.delta_fast[self.current_task]

        self.acquiescence_calc()

    def acquiescence_calc(self):
        if (self.active_time[self.current_task] > self.psi and self.comm[self.current_task][self.current_robot] == 1) \
                or self.active_time[self.current_task] > self.lambda_task[self.current_task]:
            rospy.logerr('RESET' + str(self.robot_id))
            self.acquiescence[self.current_task] = 0
            self.activity_suppression[0] = 1
            for i in range(1, self.task_num):
                self.activity_suppression[i] = 0

            self.impatience[self.current_task] = self.delta_slow[self.current_task]

        else:
            self.acquiescence[self.current_task] = 1

        # self.motivation_calc()

    def motivation_calc(self):
        for i in range(self.task_num):
            self.motivation[i] = (self.motivation_old[i] + self.impatience[i]) * \
                                 self.sensory_feedback[i] * self.activity_suppression[i] * \
                                 self.impatience_reset[i] * self.acquiescence[i]
            self.motivation_old[i] = self.motivation[i]

            # rospy.loginfo("MSG: " + str(self.current_task) +
            #              "\t" + str(self.current_robot) +
            #              "\t" + str(self.current_behaviour) + "\t" + str(self.robot_id) + "-------------------"
            #              + "\nimpatience: " + str(i) + " %.2f" % self.impatience[i]
            #              + "\nsensory: " + str(i) + " %.2f" % self.sensory_feedback[i]
            #              + "\nSuppression: " + str(i) + " %.2f" % self.activity_suppression[i]
            #              + "\nImpatience reset: " + str(i) + " %.2f" % self.impatience_reset[i]
            #              + "\nAcquiescence: " + str(i) + " %.2f" % self.acquiescence[i]
            #              + "\nMotivation: " + str(i) + " %.2f" % self.motivation[i])

        self.behaviour_set()

    def behaviour_set(self):
        tasks = 0
        for i in range(self.task_num):
            if self.motivation[i] >= self.theta:
                tasks = tasks + 1
                #    rospy.logwarn("MSG: Task" + str(self.current_task) +
                #              "\t" + str(self.current_robot) +
                #              "\t" + str(self.current_behaviour) + "\t" + str(self.robot_id) + "-------------------"
                #              + "\nimpatience: " + str(i) + " %.2f" % self.impatience[i]
                #              + "\nsensory: " + str(i) + " %.2f" % self.sensory_feedback[i]
                #              + "\nSuppression: " + str(i) + " %.2f" % self.activity_suppression[i]
                #              + "\nImpatience reset: " + str(i) + " %.2f" % self.impatience_reset[i]
                #              + "\nAcquiescence: " + str(i) + " %.2f" % self.acquiescence[i]
                #              + "\nMotivation: " + str(i) + " %.2f" % self.motivation[i])
                self.current_behaviour = i
                self.motivation_old[i] = self.theta
            else:
                self.active_time[i] = 0

        if tasks == 0:
            self.current_behaviour = 0

        self.suppression()

    def suppression(self):
        if self.my_task == 0 or self.acquiescence[self.my_task] == 0:

            # ROS_INFO("Suppression in");
            for i in range(self.task_num):
                self.activity_suppression[i] = 1
                self.impatience_reset[i] = 1

        else:
            for i in range(1, self.task_num):
                if self.my_task == i:
                    self.activity_suppression[i] = 1
                else:
                    self.activity_suppression[i] = 0

    def suppress(self):
        for i in range(self.task_num - 1):
            if self.my_task == i + 1:
                self.activity_suppression[i + 1] = 1
            else:
                self.activity_suppression[i + 1] = 0

    def update(self):
        self.motivation_calc()
