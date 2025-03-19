import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_about_axis

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

import serial
from math import sin, cos, pi


class VelocityController(Node):
    """
    A ROS interface for the HomeplateRobot
    """

    def __init__(self):
        super().__init__("hpr_interface")
        # Create serial communication to Pico
        self.pico_msngr = serial.Serial("/dev/ttyACM0", 115200)
        self.listen_pico_msg_timer = self.create_timer(0.015, self.listen_pico_msg)
        # Create target velocity subscriber
        self.targ_vel_subr = self.create_subscription(
            topic="cmd_vel",
            msg_type=Twist,
            callback=self.set_vel,
            qos_profile=1,
        )
        # Create odometry publisher
        self.odom_pubr = self.create_publisher(
            topic="odom",
            msg_type=Odometry,
            qos_profile=1,
        )
        self.publish_odom_timer = self.create_timer(0.02, self.publish_odom)
        # Create tf broadcaster
        self.odom_base_broadcaster = TransformBroadcaster(self)
        # variables
        self.lin_vel = 0.0  # in base_link frame
        self.ang_vel = 0.0  # and they are real vels
        self.x = 0.0  # in odom frame
        self.y = 0.0
        self.th = 0.0
        self.prev_ts = self.get_clock().now()
        self.curr_ts = self.get_clock().now()
        # constants
        self.GROUND_CLEARANCE = 0.0375
        self.get_logger().info("HomeR's driver node brought up.")

    def listen_pico_msg(self):
        if self.pico_msngr.inWaiting() > 0:
            vels = (
                self.pico_msngr.readline().decode("utf-8").rstrip().split(",")
            )  # actual linear and angular vel
            if len(vels) == 2:
                self.lin_vel = float(vels[0])
                self.ang_vel = float(vels[1])
        self.get_logger().debug(
            f"HomeR's real velocity\nlinear: {self.lin_vel}, angular: {self.ang_vel}"
        )

    def set_vel(self, msg):
        targ_lin = msg.linear.x
        targ_ang = msg.angular.z
        self.pico_msngr.write(f"{targ_lin},{targ_ang}\n".encode("utf-8"))
        self.get_logger().debug(
            f"Set HomeR's target velocity\nlinear: {targ_lin}, angular: {targ_ang}"
        )

    def publish_odom(self):
        self.curr_ts = self.get_clock().now()
        dt = (self.curr_ts - self.prev_ts).nanoseconds * 1e-9
        dx = self.lin_vel * cos(self.th) * dt
        dy = self.lin_vel * sin(self.th) * dt
        dth = self.ang_vel * dt
        self.x += dx
        self.y += dy
        self.th += dth
        if self.th > pi:
            self.th -= 2 * pi
        elif self.th < -pi:
            self.th += 2 * pi
        quat = quaternion_about_axis(self.th, (0, 0, 1))
        self.prev_ts = self.curr_ts
        # publish odom to base_link transform
        odom_base_trans = TransformStamped()
        odom_base_trans.header.stamp = self.curr_ts.to_msg()
        odom_base_trans.header.frame_id = "odom"
        odom_base_trans.child_frame_id = "base_link"
        odom_base_trans.transform.translation.x = self.x
        odom_base_trans.transform.translation.y = self.y
        odom_base_trans.transform.translation.z = self.GROUND_CLEARANCE
        odom_base_trans.transform.rotation.x = quat[0]
        odom_base_trans.transform.rotation.y = quat[1]
        odom_base_trans.transform.rotation.z = quat[2]
        odom_base_trans.transform.rotation.w = quat[3]
        self.odom_base_broadcaster.sendTransform(odom_base_trans)
        # publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.curr_ts.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.GROUND_CLEARANCE
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.lin_vel
        odom_msg.twist.twist.angular.z = self.ang_vel
        self.odom_pubr.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
