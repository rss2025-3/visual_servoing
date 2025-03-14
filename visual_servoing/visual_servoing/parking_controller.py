#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
import math

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.acceptable_angle = 10 * math.pi / 180
        self.relative_x = 0
        self.relative_y = 0

        self.L = 0.25
        self.L_ah = 0.33
        self.speed = 1.0
        self.backwards = False

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        eta = math.atan(self.relative_y / self.relative_x)
        delta = math.atan(2 * self.L * math.sin(eta) / self.L_ah)

        current_time = self.get_clock().now()
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.header.stamp = current_time.to_msg()
        
        if self.backwards is False:
            if self.relative_x < self.parking_distance and abs(eta) < self.acceptable_angle:
                # we're parked
                drive_cmd.drive.speed = 0.0
                drive_cmd.drive.steering_angle = 0.0
            elif self.relative_x < self.parking_distance:
                # we're too close
                drive_cmd.drive.speed = -1 * self.speed
                drive_cmd.drive.steering_angle = 0.0
                self.backwards = True
            else:
                drive_cmd.drive.speed = 1 * self.speed
                drive_cmd.drive.steering_angle = delta
        else:
            if self.relative_x > 2 * self.parking_distance:
                drive_cmd.drive.speed = 1 * self.speed
                drive_cmd.drive.steering_angle = delta
                self.backwards = False
            else:
                drive_cmd.drive.speed = -1 * self.speed
                drive_cmd.drive.steering_angle = 0.0



        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        error_msg.x_error = float(self.relative_x)
        error_msg.y_error = float(self.relative_y)
        error_msg.distance_error = float(math.sqrt(self.relative_x ** 2 + self.relative_y ** 2))

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()