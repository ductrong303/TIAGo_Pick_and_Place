#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose
import tf.transformations as transformations
import tf2_ros
from pick_and_place.msg import MoveArmAction, MoveArmResult, MoveArmFeedback, MoveArmGoal


class TiagoRob():
    def __init__(self):
        self.pnp_client = actionlib.SimpleActionClient('pnp_server', MoveArmAction)
        rospy.loginfo("TiagoRob init")

    def find_object_with_aruco(self):
        rospy.loginfo("Start looking for Aruco marker")
        aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        rospy.loginfo("Marker found")
        transform_ok = False
        tfBuffer = tf2_ros.Buffer()
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = tfBuffer.lookup_transform("base_footprint", 
	    								   aruco_pose.header.frame_id,
	    								   rospy.Time(0))
                trans_pose = do_transform_pose(aruco_pose, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:  
                rospy.logwarn(
	    				"Exception on transforming point... trying again \n(" +
	    				str(e) + ")")
            rospy.sleep(0.01)
        rospy.loginfo("Setting cube pose based on ArUco detection")
        
        dist_btw_entry_and_goal = 0.04
        entry_point = PoseStamped()
        entry_point.header.frame_id = trans_pose.header.frame_id
        entry_point.pose.position = trans_pose.pose.position
        entry_point.pose.position.x = entry_point.pose.position.x - dist_btw_entry_and_goal
        quaternion = transformations.quaternion_from_euler(3.14/2, 0, 0)
        entry_point.pose.orientation.x = quaternion[0] 
        entry_point.pose.orientation.y = quaternion[1] 
        entry_point.pose.orientation.z = quaternion[2] 
        entry_point.pose.orientation.w = quaternion[3] 

        return entry_point


    def pnp_goal_response_cb(self):
        rospy.loginfo('Server accepted the goal')

    def pnp_feedback_cb(self, feedback):
        # This function is called whenever the action server sends feedback
        rospy.loginfo('Feedback from pnp server: %s' % str(feedback.fb))


    def start_pnp_process(self):
        # Entry point information extracte from the ArUco marker
        #entry_pt = self.find_object_with_aruco()

        # Artificial entry point coordinates and orientations
        entry_point = PoseStamped()
        entry_point.header.frame_id = "base_footprint"
        entry_point.pose.position.x = 0.4
        entry_point.pose.position.y = 0.15
        entry_point.pose.position.z = 0.85
        quaternion = transformations.quaternion_from_euler(3.14/2, 0, 0)
        entry_point.pose.orientation.x = quaternion[0] 
        entry_point.pose.orientation.y = quaternion[1] 
        entry_point.pose.orientation.z = quaternion[2] 
        entry_point.pose.orientation.w = quaternion[3] 

        # Send the entry point information as a goal to server
        self.pnp_client.wait_for_server()
        goal = MoveArmGoal()
        goal.target_pose = entry_point
        rospy.loginfo("Sending the follow goal {}".format(goal.target_pose))
        self.pnp_client.send_goal(goal, feedback_cb= self.pnp_feedback_cb, active_cb = self.pnp_goal_response_cb)

        # Wait for the result from server
        timeout = self.pnp_client.wait_for_result()
        if not timeout:
            rospy.loginfo("PickandPlace server not available")
            return False

        # Assess the received result
        result = self.pnp_client.get_result()
        rospy.loginfo("PnP is successful: " + str(result.res))
        rospy.loginfo("Final state is " + str(self.pnp_client.get_state()))

        if result and self.pnp_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("PnP action is performed successfully")
        else:
            rospy.loginfo("PnP action failed")
            return False
        return True


if __name__ == '__main__':
    rospy.init_node('main_node')
    my_rob = TiagoRob()
    my_rob.start_pnp_process()
    rospy.spin()