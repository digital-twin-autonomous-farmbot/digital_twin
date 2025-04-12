from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30) # 30 Hz

        # robot state
        left_wheel_angle = 0.0
        right_wheel_angle = 0.0
        tinc = degree
        angle = 0.0
        height = 0.0
        hinc = 0.005

        

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "base_link"
        odom_trans.child_frame_id = "axis"
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                # update the joint state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = [
                    'left_f_wheel_joint', 'left_b_wheel_joint',
                    'right_f_wheel_joint', 'right_b_wheel_joint'
                ]
                joint_state.position = [left_wheel_angle, left_wheel_angle, right_wheel_angle, right_wheel_angle]

                # update the transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 2.0 * cos(angle)
                odom_trans.transform.translation.y = 2.0 * sin(angle)
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = euler_to_quaternion(0, 0, angle + pi/2)

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # create new robot state
                left_wheel_angle += tinc
                right_wheel_angle += tinc
                if left_wheel_angle > pi:  
                    left_wheel_angle = -pi
                if right_wheel_angle > pi:  
                    right_wheel_angle = -pi
                angle += degree / 4
                
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
