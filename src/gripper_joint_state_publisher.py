#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input

class RobotiqGripperStatePublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('robotiq_gripper_state_publisher', anonymous=True)

        # 创建一个发布器
        self.pub = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)

        # 订阅Robotiq手爪的输入状态
        rospy.Subscriber('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.joint_state_publisher)

    def joint_state_publisher(self, data):
        """Publish the gripper joint state in ROS."""
        # 计算对应的joint angle范围从0到0.8
        position = (data.gPR / 255.0) * 0.8
        
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['robotiq_85_left_knuckle_joint']
        joint_state.position = [position]

        # 发布关节状态
        self.pub.publish(joint_state)

def main():
    # 创建类实例
    gripper_publisher = RobotiqGripperStatePublisher()

    # 保持节点运行直到被关闭
    rospy.spin()

if __name__ == '__main__':
    main()
