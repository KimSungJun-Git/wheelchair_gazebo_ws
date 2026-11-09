#!/usr/bin/env python3
# imu_relay_node.py
# stella_ahrs_node의 빈 header.frame_id를 'imu_link'로 보정하여
# EKF 등의 TF 변환 호환성을 맞추기 위해 /imu/data_fixed로 재발행하는 노드.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuRelayNode(Node):
    def __init__(self):
        super().__init__('imu_relay_node')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_fixed')
        self.declare_parameter('target_frame_id', 'imu_link')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value

        # stella_ahrs_node가 RELIABLE로 발행하므로 매칭되는 QoS 사용
        qos_in = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_out = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(Imu, input_topic, self.imu_callback, qos_in)
        self.pub = self.create_publisher(Imu, output_topic, qos_out)
        self.get_logger().info(f'IMU Relay started: {input_topic} -> {output_topic} (frame_id: {self.target_frame_id})')

    def imu_callback(self, msg: Imu):
        msg.header.frame_id = self.target_frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
