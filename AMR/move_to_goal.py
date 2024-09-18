#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import tf.transformations as tft
import time

class MoveBaseControl:
    def __init__(self):
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber('user_destination', String, self.destination_callback)
        self.done_publisher = rospy.Publisher('move_base_done', String, queue_size=10)
        self.moving = False

    def move_to_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        yaw_radians = math.radians(yaw)
        quaternion = tft.quaternion_from_euler(0, 0, yaw_radians)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("Sending goal to x=%f, y=%f, yaw=%f degrees", x, y, yaw)
        self.client.send_goal(goal, done_cb=self.done_callback)
        self.moving = True
        rospy.loginfo("Robot is moving...")

    def done_callback(self, state, result):
        rospy.loginfo("Callback state: %d", state)  # 상태 로깅 추가
        if state == 3:
            rospy.loginfo("Reached goal!")  # 도달 로깅 강화
            self.done_publisher.publish("done")
        elif state == 2:
            rospy.loginfo("Goal preempted, stopping...")
            self.stop_robot()
            self.done_publisher.publish("preempted")
        elif state == 4:
            rospy.loginfo("Goal aborted, stopping...")
            self.stop_robot()
            self.done_publisher.publish("aborted")
        self.moving = False

    def stop_robot(self):
        # This function should send a command to stop the robot
        # Example: Publish a zero velocity Twist message to the robot's cmd_vel topic
        stop_command = Twist()
        stop_command.linear.x = 0
        stop_command.angular.z = 0
        # Assuming a publisher exists for cmd_vel, it would look something like this:
        # self.cmd_vel_publisher.publish(stop_command)
        rospy.loginfo("Robot has been stopped.")

    def destination_callback(self, data):
        if data.data == 's':  
            if self.moving:
                rospy.loginfo("Stopping the robot...")
                self.client.cancel_goal()  
            else:
                rospy.loginfo("Robot is not moving.")
        else:
            if not self.moving:  # 이 부분에서 로봇이 이동 중이지 않을 때만 명령을 받도록 함
                self.moving = True  # 명령을 받으면 바로 이동 중 상태로 변경
                if data.data == '1':
                    self.move_to_goal(3.23, 4.06, -87.53) 
                elif data.data == '2':
                    self.move_to_goal(0.05, 1.30, -0.80) 
                elif data.data == '3':
                    self.move_to_goal(5.94, 5.03, -90)
                elif data.data == '4':
                    self.move_to_goal(0, 0,  0)
                else:
                    rospy.loginfo("Invalid destination!")
                    self.moving = False  # 잘못된 명령

def main():
    rospy.init_node('move_to_goal', anonymous=True)
    move_base_control = MoveBaseControl()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
