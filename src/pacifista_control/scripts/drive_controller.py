#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import pandas as pd
from effective_radii import *


class WheelController:
    def __init__(self):
        rospy.init_node('wheel_controller')
        # Read the contact points and ICRs from the CSV files
        contact_points = pd.read_csv('/home/paradox/OELP_WS/src/pacifista_control/scripts/contact_points_1000.csv').values
        center_of_wheel = np.array([0, 0, 0])
        self.effective_radii = calculate_effective_radii(contact_points, center_of_wheel)
        self.wheel_base = 0.3
        self.rate = rospy.Rate(10)  # 10 Hz
        # Publishers for each wheel velocity command
        self.pub1 = rospy.Publisher('/pacifista/Rev1/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/pacifista/Rev2/command', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/pacifista/Rev3/command', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/pacifista/Rev4/command', Float64, queue_size=10)

        # Subscribers for commanded robot velocity and joint states
        rospy.Subscriber('/pacifista/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/pacifista/joint_states', JointState, self.joint_states_callback)

        self.cmd_linear_velocity = 0.0
        self.cmd_angular_velocity = 0.0
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
    
    def calculate_effective_radii(contact_points, icrs):
        return np.linalg.norm(contact_points - icrs, axis=1)

    def radius_function(angle, radii, num_points=5):
        angle_normalized = angle % (2 * np.pi / num_points)
        idx = int(angle_normalized * len(radii) / (2 * np.pi / num_points))
        return radii[idx]

    def cmd_vel_callback(self, msg):
        self.cmd_linear_velocity = msg.linear.x  # Assuming forward motion is along the x-axis
        self.cmd_angular_velocity = msg.angular.z  # Assuming rotation is around the z-axis

    def joint_states_callback(self, msg):
        # Assuming the joint angles are in the order [Rev1, Rev2, Rev3, Rev4]
        self.joint_angles = msg.position

    def calculate_and_publish_wheel_velocities(self):
        # Calculate the effective radii of the wheels based on the joint angles
        effective_radii = [radius_function(angle, self.effective_radii) for angle in self.joint_angles]
        print(effective_radii)
         # Assuming a skid-steering (differential drive) robot, calculate the wheel velocities
        left_wheel_velocity_f = (self.cmd_linear_velocity - (self.wheel_base / 2.0) * self.cmd_angular_velocity) / effective_radii[1]
        left_wheel_velocity_b = (self.cmd_linear_velocity - (self.wheel_base / 2.0) * self.cmd_angular_velocity) / effective_radii[3]
        right_wheel_velocity_f = (self.cmd_linear_velocity + (self.wheel_base / 2.0) * self.cmd_angular_velocity) / effective_radii[0]
        right_wheel_velocity_b = (self.cmd_linear_velocity + (self.wheel_base / 2.0) * self.cmd_angular_velocity) / effective_radii[2]


        # Publish the wheel velocities
        self.pub1.publish(right_wheel_velocity_f)
        self.pub2.publish(left_wheel_velocity_f)
        self.pub3.publish(right_wheel_velocity_b)
        self.pub4.publish(left_wheel_velocity_b)

def main():
    
    wc = WheelController()

    while not rospy.is_shutdown():
        wc.calculate_and_publish_wheel_velocities()
        wc.rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

