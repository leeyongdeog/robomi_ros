import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Waypoints 설정
waypoints = [
    [(13.415, -5.186, 0.0), (0.0, 0.0, -0.361, 0.932)],
    [(17.182, -5.613, 0.0), (0.0, 0.0, 0.972, -0.234)],
    [(17.136, -9.036, 0.0), (0.0, 0.0, 0.916, 0.399)],
    [(13.570, -8.771, 0.0), (0.0, 0.0, 0.304, 0.952)],
    [(15.498, -7.280, 0.0), (0.0, 0.0, 0.202, 0.965)],
    [(15.498, -7.264, 0.0), (0.0, 0.0, -0.317, 0.948)],
    [(15.498, -7.264, 0.0), (0.0, 0.0, 0.917, -0.396)],
    [(15.498, -7.264, 0.0), (0.0, 0.0, 0.930, 0.366)]
]

class Patrol:

    def __init__(self):
        rospy.init_node('my_patrol')
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

    def run(self):
        for pose in waypoints:
            goal = self.goal_pose(pose)
            self.move_client.send_goal(goal)
            self.move_client.wait_for_result()
            time.sleep(1)

    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

if __name__ == "__main__":
    patrol = Patrol()
    patrol.run()
