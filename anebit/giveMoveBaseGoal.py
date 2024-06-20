#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import sys

def send_goal(x, y):
    # Create an action client called 'move_base' with action definition file 'MoveBaseAction'
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Defines the goal position and a neutral orientation (any yaw is acceptable)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # Neutral orientation

    # Sends the goal to the action server.
    client.send_goal(goal)
    wait = client.wait_for_result()
    
    # Checks if the result was successful.
    if not wait:
        rospy.logerr("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_goal_sender')
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        result = send_goal(x, y)  # Modify x, y as needed
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

