#!/usr/bin/env python3
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk

class GripperControl:
    def __init__(self, master):
        self.master = master
        self.master.title("Gripper Control")

        # ROS initialization
        rospy.init_node('gripper_control_node', anonymous=True)
        self.publisher = rospy.Publisher('/gripper_controller/follow_joint_trajectory/goal', 
                                         FollowJointTrajectoryActionGoal, queue_size=10)

        # Styling the slider
        style = ttk.Style()
        style.theme_use('clam')  # Use the 'clam' theme as the base for customization
        style.configure("TScale", sliderlength=40, background="#333", foreground="#fff", troughcolor="#555", bordercolor="#666", lightcolor="#777", darkcolor="#444")

        # GUI setup
        self.slider_value = tk.DoubleVar()
        ttk.Label(master, text="Gripper Position:").pack()
        self.slider = ttk.Scale(master, from_=0.0, to=0.8, orient='horizontal', 
                                variable=self.slider_value, length=300,  # Make the slider longer
                                command=self.update_value, style="TScale")
        self.slider.pack(pady=20)  # Add some padding around the slider

        # Button to send the command
        self.send_button = ttk.Button(master, text="Send Command", command=self.send_command)
        self.send_button.pack(pady=10)  # Add padding below the button

    def update_value(self, event):
        # This function is called whenever the slider is moved
        value = self.slider_value.get()
        rospy.loginfo("Slider updated to: %f", value)

    def send_command(self):
        # Prepare the message
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"] 
        point = JointTrajectoryPoint()
        point.positions = [self.slider_value.get()]
        point.time_from_start = rospy.Duration(1.0)
        msg.goal.trajectory.points.append(point)
        
        # Publish the message
        self.publisher.publish(msg)
        rospy.loginfo("Sent gripper command: %f", self.slider_value.get())

def main():
    root = tk.Tk()
    app = GripperControl(root)
    root.mainloop()

if __name__ == '__main__':
    main()