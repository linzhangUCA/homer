import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_about_axis
import serial
from math import sin, cos, pi


class VelocityController(Node):
    """
    A ROS interface for the HomeplateRobot
    """

    def __init__(self):
        super().__init__("hpr_interface")
        # Create serial communication
        self.pico_msngr = serial.Serial("/dev/ttyACM0", 115200)
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
        self.odom_pub_timer = self.create_timer(0.02, self.odom_pub_cb)
        # Create transformations
        self.base_foorprint_broadcaster = StaticTransformBroadcaster(self)
        self.odom_base_broadcaster = TransformBroadcaster(self)
        self.base_lidar_broadcaster = StaticTransformBroadcaster(self)
        # variables
        self.lin_vel = 0.0  # in base_link frame
        self.ang_vel = 0.0
        self.x = 0.0  # in odom frame
        self.y = 0.0
        self.th = 0.0
        self.prev_ts = self.get_clock().now()
        self.curr_ts = self.get_clock().now()
        # constants
        self.GROUND_CLEARANCE = 0.0375  # wheel radius

    def set_vel(self, msg):
        target_lin = msg.linear.x
        target_ang = msg.angular.z
        # target_vel_str = f"{target_lin},{target_ang}\n"
        # self.messenger.write(bytes(target_vel_str.encode("utf-8")))
        self.pico_msngr.write(f"{target_lin}, {target_ang}\n".encode("utf-8"))

    def odom_pub_cb(self):
        if self.pico_msngr.inWaiting() > 0:
            vels = (
                self.pico_msngr.readline().decode("utf-8").rstrip().split(",")
            )  # actual linear and angular vel
            if len(vels) == 2:
                self.lin_vel = float(vels[0])
                self.ang_vel = float(vels[1])
            # print(self.lin_vel, self.ang_vel)
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
        # publish base_link to base_footprint transform
        base_footprint_trans = TransformStamped()
        base_footprint_trans.header.stamp = self.curr_ts.to_msg()
        base_footprint_trans.header.frame_id = "base_link"
        base_footprint_trans.child_frame_id = "base_footprint"
        base_footprint_trans.transform.translation.x = 0.0
        base_footprint_trans.transform.translation.y = 0.0
        base_footprint_trans.transform.translation.z = -0.0375
        base_footprint_trans.transform.rotation.x = 0.0
        base_footprint_trans.transform.rotation.y = 0.0
        base_footprint_trans.transform.rotation.z = 0.0
        base_footprint_trans.transform.rotation.w = 1.0
        self.base_lidar_broadcaster.sendTransform(base_footprint_trans)
        # publish base_link to lidar_link transform
        base_lidar_trans = TransformStamped()
        base_lidar_trans.header.stamp = self.curr_ts.to_msg()
        base_lidar_trans.header.frame_id = "base_link"
        base_lidar_trans.child_frame_id = "lidar_link"
        base_lidar_trans.transform.translation.x = 0.0
        base_lidar_trans.transform.translation.y = 0.0
        base_lidar_trans.transform.translation.z = 0.093
        lidar_quat = quaternion_about_axis(pi, (0, 0, 1))
        base_lidar_trans.transform.rotation.x = lidar_quat[0]
        base_lidar_trans.transform.rotation.y = lidar_quat[1]
        base_lidar_trans.transform.rotation.z = lidar_quat[2]
        base_lidar_trans.transform.rotation.w = lidar_quat[3]
        self.base_lidar_broadcaster.sendTransform(base_lidar_trans)
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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
