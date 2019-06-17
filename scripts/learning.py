#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
import numpy as np
from os.path import join
from cv_bridge import CvBridge
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cs_golf.srv import RateIteration, RateIterationResponse
from cs_golf.srv import Plan, PlanResponse
from sensor_msgs.msg import Image
from cs_golf.persistence import dicttostate

class Learning(object):
    NUM_VALUES = 100   # Number of discrete values from each exploration space (angle + speed)
    DURATION_RANGE = [0.15, 0.2]    # Minimum and maxium durations of motions ; speed indexes are mapping within this range
                                    # Acceleration and deceleration of 25% are added to this duration 

    def __init__(self):
        self.bridge = CvBridge()
        self.smoke_motion_id = 0
        self.scores = np.ones((self.NUM_VALUES, self.NUM_VALUES))  # scores[speed_i][angle_i] = mark of this trajectory between [0., 1.]
        self.num_votes = 0  # Number of votes taken into account in self.scores so far
        self.pending_iteration_parameters = [-1, -1, -1]  # iteration + speed + angle parameters indexes to be scored
        self.x = np.linspace(0, 1, self.NUM_VALUES)   # Basic linear space for speed and angle dimensions
        self.rospack = rospkg.RosPack()
        rospy.Service('golf/learning/plan', Plan, self._cb_plan)
        rospy.Service('golf/learning/rate', RateIteration, self._cb_rate)
        self.scores_pub = rospy.Publisher("golf/learning/scores", Image, queue_size=1)

        with open(join(self.rospack.get_path("cs_golf"), "config/motions.json")) as f:
            self.motions = json.load(f)

        with open(join(self.rospack.get_path("cs_golf"), "config/optimal.json")) as f:
            self.optimal = json.load(f)
       
        rospy.loginfo("Learning node is ready to plan!")

    def _make_shooting_trajectory(self, json_traj, duration):
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = [str(j) for j in self.motions["joints"]]
        num_points = len(json_traj)
        time = 0
        time_step = float(duration)/num_points
        # Acceleration-deceleration profile
        profile = []
        num_points_acceleration = int(num_points/4)
        for i in range(num_points_acceleration):
            profile.append(4. - 3*(float(i)/num_points_acceleration))
        for i in range(num_points - 2*num_points_acceleration):
            profile.append(1.)
        for i in range(num_points_acceleration):
            profile.append(1. + 3*float(i)/num_points_acceleration)
        time = 0
        for i_point, point in enumerate(json_traj):
            time += profile[i_point] * time_step
            jtp = JointTrajectoryPoint()
            jtp.positions = point
            jtp.time_from_start = rospy.Duration.from_sec(time)
            rt.joint_trajectory.points.append(jtp)
        return rt

    def _pick_trajectories_params(self, iteration):
        """
        Generates parameters of trajectories:
          * Angle (motion ID)
          * Speed (mapped to trajectory duration in seconds later on)
        
        It is influenced by 3 components:
          * User's grade given to the last iteration between 0 and 10
          * Randomness
          * Natural attraction to the optimal parameters
        """
        mark_ratio = float(iteration)/self.optimal["num_iterations_convergence"]
        # TODO: Should we force convergence to a right value?
        optimal_ratio = 0 # 2./self.optimal["num_iterations_convergence"]  
        rand_ratio = 1. - mark_ratio - optimal_ratio

        # Highest mark component
        highest_mark_speed_i, highest_mark_angle_i =  np.unravel_index(np.argmax(self.scores), self.scores.shape)

        # Random component
        random_speed_i = np.random.randint(0, self.NUM_VALUES)
        random_angle_i = np.random.randint(0, self.NUM_VALUES)

        # TODO: Should we force convergence to a right value?
        optimal_speed_i = self.optimal["optimal_speed_i"]
        optimal_angle_i = self.optimal["optimal_angle_i"]

        # Merging random and highest mark components
        selected_speed_i = int(random_speed_i*rand_ratio + highest_mark_speed_i*mark_ratio + optimal_speed_i*optimal_ratio)
        selected_angle_i = int(random_angle_i*rand_ratio + highest_mark_angle_i*mark_ratio + optimal_angle_i*optimal_ratio)

        return selected_speed_i, selected_angle_i

    def add_score(self, mark, value_speed, value_angle):
        def gauss(x, mu, sigma):
            return np.exp(-(x-mu)*(x-mu)/(2*sigma*sigma))
        
        SIGMA = 0.1   # Impact of a single mark on neighbours
        speeds = np.tile((mark)*gauss(self.x, value_speed, SIGMA), [self.NUM_VALUES,1])
        angles = np.tile((mark)*gauss(self.x, value_angle, SIGMA).reshape((self.NUM_VALUES, 1)), self.NUM_VALUES)
        new_frame = np.multiply(speeds, angles)
        self.scores = (self.num_votes*self.scores + new_frame)/(self.num_votes+1)
        self.num_votes += 1

    def _cb_plan(self, req):
        iteration = rospy.get_param("golf/iteration")
        i_motion, i_speed = self._pick_trajectories_params(iteration)

        # If smoke mode is active, overide the planned trajectory
        if rospy.get_param('golf/smoke', False):
            i_motion = self.smoke_motion_id
            rospy.logwarn("Planning the smoke trajectory #{}".format(i_motion))
            self.smoke_motion_id = (self.smoke_motion_id + 1) % len(self.motions["trajectories"])
        
        duration = i_speed*(self.DURATION_RANGE[1] - self.DURATION_RANGE[0])/self.NUM_VALUES + self.DURATION_RANGE[0]   # Linear mapping between indexes and min/max trajecotry durations
        angle = self.motions["trajectories"][i_motion]["angle"]
        traj = self._make_shooting_trajectory(self.motions["trajectories"][i_motion]["points"], duration)
        rospy.loginfo("Generated a trajectory of {} sec with angle {} (motion index {})".format(traj.joint_trajectory.points[-1].time_from_start.to_sec(), angle, i_motion))
        self.pending_iteration_parameters[0] = iteration
        self.pending_iteration_parameters[1] = i_speed     # Speed index
        self.pending_iteration_parameters[2] = i_motion    # Motion's ID = its angle (index)    
        return PlanResponse(trajectory=traj)

    def _cb_rate(self, req):
        speed_i = self.pending_iteration_parameters[1]
        angle_i = self.pending_iteration_parameters[2]
        iteration = self.pending_iteration_parameters[0]
        success = False
        try:
            scored_iteration = int(req.iteration)
            int_grade = int(req.grade)
        except:
            scored_iteration = -1
            int_grade = -1
        else:
            if speed_i >= 0 and angle_i >= 0 and iteration >= 0 and iteration == scored_iteration:
                if 0 <= int_grade <= 10:
                    float_grade = int_grade/10.
                    rospy.logwarn("Recording mark {} for speed index {} and angle index {}".format(float_grade, speed_i, angle_i))
                    self.add_score(float_grade, float(speed_i)/self.NUM_VALUES, float(angle_i)/self.NUM_VALUES)
                    success = True
                    self.pending_iteration_parameters = [-1, -1, -1]
                else:
                    rospy.logerr("Invalid integer grade not in [0, 10]: {}".format(int_grade))
            else:
                rospy.logerr("Unexpected score for iteration {}".format(scored_iteration))
            response = RateIterationResponse(success=success)
        return response

    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            #image = self.bridge.cv2_to_imgmsg(self.scores, encoding="8UC1")
            #self.scores_pub.publish(image)
            import matplotlib.pyplot as plt
            plt.imshow(self.scores, cmap='hot', interpolation='nearest')
            plt.show()
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('cs_golf_learning')
    learning = Learning()
    learning.run()
