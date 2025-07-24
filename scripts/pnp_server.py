#!/usr/bin/env python
import sys
import copy
import numpy as np
import tf.transformations as tran
import tf2_ros
import rospy
import actionlib
import moveit_commander
from pick_and_place.msg import MoveArmAction, MoveArmResult, MoveArmFeedback
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Empty, Float64
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class PnPServer:
    def __init__(self):
        # Initiate the server
        self.pnp_server = actionlib.SimpleActionServer('pnp_server', MoveArmAction, execute_cb=self.pick_cb, auto_start=False)
        self.pnp_server.start()
        rospy.loginfo("Starting PickandPlace server")
        self.group_arm_torso = moveit_commander.MoveGroupCommander("arm_torso")   

        self.play_motion_client = actionlib.SimpleActionClient('play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server()

        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy(
        	'/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected!")


    def abort(self, msg):
        rospy.logerr(msg)
        self.pnp_server.set_aborted(MoveArmResult(msg))


    def succeed(self, msg):
        rospy.loginfo(msg)
        self.pnp_server.set_succeeded(MoveArmResult(msg))


    def feedback(self, msg):
        rospy.loginfo(msg)
        self.pnp_server.publish_feedback(MoveArmFeedback(msg))


    def move_tcp_to_pose(self, target_pose):
        result = MoveArmResult()
        result.res = True
        fb_msg = MoveArmFeedback()

        self.group_arm_torso.set_planner_id("SBLkConfigDefault")
        self.group_arm_torso.set_pose_reference_frame("base_footprint")
        self.group_arm_torso.set_pose_target(target_pose)

        self.group_arm_torso.set_start_state_to_current_state()
        self.group_arm_torso.set_max_velocity_scaling_factor(1.0)
        self.group_arm_torso.set_planning_time(5.0)
        
        plan_success = self.group_arm_torso.plan()
        if not plan_success:
            fb_msg.fb = "No plan found"
            self.pnp_server.publish_feedback(fb_msg)
            rospy.loginfo("Failed to find the plan.")
            return False
        
        # Execute the plan
        fb_msg.fb = "Plan found"
        self.pnp_server.publish_feedback(fb_msg)
        start_time = rospy.Time.now()
        exec_result = self.group_arm_torso.go(wait=True)
        rospy.loginfo("Motion duration: %s seconds", (rospy.Time.now() - start_time).to_sec())
   
        if not exec_result:
            # Execution failed, publish feedback and set aborted
            fb_msg.fb = "Execution failed"
            self.pnp_server.publish_feedback(fb_msg)
            result.res = False
        else:
            # Execution succeeded, publish feedback and set succeeded
            fb_msg.fb = "Execution succeeded"
            self.pnp_server.publish_feedback(fb_msg)
            result.res = True
        return result.res 

    def perform_motion(self, motion):
        rospy.loginfo("Performing: " + motion)
        motion_goal = PlayMotionGoal()
        motion_goal.motion_name = motion
        self.play_motion_client.send_goal(motion_goal)

        # Wait for the action to complete (or set a timeout)
        if not self.play_motion_client.wait_for_result(rospy.Duration(15.0)):
            rospy.logwarn(motion + " action did not complete in the expected time.")
            self.play_motion_client.cancel_goal()
        else:
            rospy.loginfo(motion + " action completed successfully.")


    def pick_cb(self, goal):
        rospy.loginfo("Pick and Place server receive goal \n {}".format(goal.target_pose))

        self.perform_motion("inspect_surroundings")

        result = MoveArmResult()
        result.res = True
        dist_btw_entry_and_goal = 0.04
        
        # Open Gripper
        self.perform_motion("open")

        # Move to entry point
        entry_pose = PoseStamped()
        entry_pose = goal.target_pose.pose
        rospy.loginfo("Gripper going to entry point")
        result.res = self.move_tcp_to_pose(entry_pose)

        # Move linearly to the pick point
        travelled_dist = 0
        fb_msg = MoveArmFeedback()
        if result.res:
            rospy.loginfo("Start moving tcp forward")
            while travelled_dist <dist_btw_entry_and_goal and result.res:
                fb_msg.fb = "Travelled distance = {}".format(travelled_dist)
                self.pnp_server.publish_feedback(fb_msg)             
                curr_pose = self.group_arm_torso.get_current_pose().pose
                next_pose = curr_pose
                next_pose.position.x = next_pose.position.x + 0.01   
                result.res = self.move_tcp_to_pose(next_pose)
                travelled_dist = travelled_dist + 0.01  

        # Close gripper
        self.perform_motion("close_half")

        #Prepare the raise and place point
        raise_pose = PoseStamped()
        raise_pose = copy.deepcopy(goal.target_pose)
        raise_pose.header.frame_id = "base_footprint"
        raise_pose.pose.position.z = raise_pose.pose.position.z + 0.2

        place_pose = PoseStamped()
        place_pose = copy.deepcopy(goal.target_pose)
        place_pose.header.frame_id = "base_footprint"
        place_pose.pose.position.x = place_pose.pose.position.x+ 0.05
        place_pose.pose.position.y = place_pose.pose.position.y - 0.4

        # Raise
        rospy.loginfo("RAISE THE OBJECT")
        rospy.loginfo("Raise pose: \n{}".format(raise_pose))
        result.res = self.move_tcp_to_pose(raise_pose)
        #Place
        rospy.loginfo("PLACE THE OBJECT TO THE SIDE")
        rospy.loginfo("PLACE pose: \n{}".format(place_pose))
        result.res = self.move_tcp_to_pose(place_pose)

        # Open the 
        self.perform_motion("open")
        if result.res:
            self.pnp_server.set_succeeded(result)
        else:
            self.pnp_server.set_aborted(result)
        

if __name__ == '__main__':
    rospy.init_node('pnp_server')
    server = PnPServer()
    rospy.spin()
