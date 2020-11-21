#!/usr/bin/env python3

import math
import json
import random

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

from utils import *


class LocalGoalCreator:
    def __init__(self):
        rospy.init_node('LocalGoalCreator', anonymous=True)

        # param
        self.HZ = rospy.get_param("~HZ", 10)
        self.WORLD_FRAME = rospy.get_param("~WORLD_FRAME", 'map')
        self.ROBOT_FRAME = rospy.get_param("~ROBOT_FRAME", 'base_link')
        self.GOAL_DIS_TOLERANCE = rospy.get_param("~GOAL_DIS_TOLERANCE", 0.3)
        self.GOAL_YAW_TOLERANCE = rospy.get_param("~GOAL_YAW_TOLERANCE", 1.0)
        self.TIMEOUT = rospy.get_param("~TIMEOUT", 180)
        self.USE_WAYPOINTS = rospy.get_param("~USE_WAYPOINTS")
        if self.USE_WAYPOINTS:
            WAYPOINTS_PATH = rospy.get_param("~WAYPOINTS_PATH")
            with open(WAYPOINTS_PATH) as f:
                waypoints_data = json.load(f)
            self.waypoints = []
            for wp in waypoints_data["WAYPOINTS"]:
                self.waypoints.append([wp["x"], wp["y"], wp["yaw"]])
            self.idx = 0
            self.goal = self.create_goal(self.idx)
        else:
            # subscriber
            target_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callabck)
            self.goal = None

        # publisher
        self.local_goal_pub = rospy.Publisher('/local_goal', PoseStamped, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.start_time = rospy.Time.now()

    def goal_callabck(self, data):
        self.goal = data
        print("next goal: ")
        print(self.goal)
        self.start_time = rospy.Time.now()

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            if self.goal is not None:
                # compute local_goal
                try:
                    robot2map = self.tfBuffer.lookup_transform(self.ROBOT_FRAME, self.WORLD_FRAME, rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    continue
                local_goal = tf2_geometry_msgs.do_transform_pose(self.goal, robot2map)
                # check goal
                time = rospy.Time.now().to_sec() - self.start_time.to_sec()
                timeout = time>self.TIMEOUT
                dis = math.sqrt(local_goal.pose.position.x**2 + local_goal.pose.position.y**2)
                yaw = abs(yaw_from_quaternion(local_goal.pose.orientation))
                arrived_goal = dis < self.GOAL_DIS_TOLERANCE and yaw < self.GOAL_YAW_TOLERANCE
                if arrived_goal or timeout:
                    if timeout:
                        print("==== timeout ====")
                    if arrived_goal:
                        print("==== goal!!! ====")
                    print("time: ", time)
                    if self.USE_WAYPOINTS:
                        self.idx = self.idx+1 if self.idx<len(self.waypoints)-1 else 0
                        self.goal = self.create_goal(self.idx)
                        print("next goal: [%d] " % self.idx)
                        print(self.goal)
                        self.start_time = rospy.Time.now()
                local_goal.header.stamp = rospy.Time.now();
                local_goal.header.frame_id = self.ROBOT_FRAME;
                self.local_goal_pub.publish(local_goal)
            r.sleep()

    def create_goal(self, idx):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.stamp = self.WORLD_FRAME
        goal.pose.position.x = self.waypoints[idx][0]
        goal.pose.position.y = self.waypoints[idx][1]
        goal.pose.orientation = quaternion_from_yaw(self.waypoints[idx][2])
        return goal

if __name__ == '__main__':
    local_goal_creator = LocalGoalCreator()
    try:
        local_goal_creator.process()
    except rospy.ROSInterruptException:
        pass
