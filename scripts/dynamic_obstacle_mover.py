#!/usr/bin/env python3
import argparse
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MultiDynamicObstacleMover(Node):
    """
    Gazebo 내 복수의 동적 장애물(Planar Move 플러그인 탑재 모델)을
    주기적으로 왕복 이동시키는 제어 노드.
    단일 토픽 및 쉼표 구분 복수 토픽 지원.
    """

    def __init__(self, topic_names, speed=0.3, duration=3.5, axis='y', rate=20.0):
        super().__init__('dynamic_obstacle_mover')

        self.speed = float(speed)
        self.move_duration = float(duration)
        self.axis = str(axis).lower()
        self.publish_rate = float(rate)

        # 토픽 목록 파싱 (쉼표 구분)
        if isinstance(topic_names, str):
            self.topics = [t.strip() for t in topic_names.split(',') if t.strip()]
        else:
            self.topics = list(topic_names)

        # 각 토픽별 퍼블리셔 및 개별 상태 (교차 이동을 위해 번갈아 반대 방향 설정)
        self.publishers_list = []
        self.directions = []
        for i, topic in enumerate(self.topics):
            pub = self.create_publisher(Twist, topic, 10)
            self.publishers_list.append(pub)
            # 짝수번째는 +1, 홀수번째는 -1로 교차 이동
            initial_dir = 1.0 if (i % 2 == 0) else -1.0
            self.directions.append(initial_dir)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ticks_per_cycle = int(self.move_duration * self.publish_rate)
        self.tick_count = 0

        self.get_logger().info(
            f"다중 동적 장애물 제어 노드 시작: topics={self.topics}, "
            f"speed={self.speed}m/s, duration={self.move_duration}s, axis='{self.axis}'"
        )

    def timer_callback(self):
        for i, pub in enumerate(self.publishers_list):
            msg = Twist()
            current_velocity = self.speed * self.directions[i]
            if self.axis == 'x':
                msg.linear.x = current_velocity
                msg.linear.y = 0.0
            else:
                msg.linear.x = 0.0
                msg.linear.y = current_velocity
            msg.angular.z = 0.0
            pub.publish(msg)

        self.tick_count += 1
        if self.tick_count >= self.ticks_per_cycle:
            for i in range(len(self.directions)):
                self.directions[i] *= -1.0
            self.tick_count = 0
            self.get_logger().debug("동적 장애물 이동 방향 반전")

    def stop_all(self):
        """종료 시 속도 0 전송"""
        msg = Twist()
        for pub in self.publishers_list:
            pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Dynamic Obstacle Mover Node')
    parser.add_argument('--topic', default='/dynamic_obstacle/cmd_vel',
                        help='cmd_vel topic name (comma-separated for multiple)')
    parser.add_argument('--speed', type=float, default=0.3, help='Linear velocity (m/s)')
    parser.add_argument('--duration', type=float, default=3.5, help='Duration in seconds per direction')
    parser.add_argument('--axis', default='y', choices=['x', 'y'], help='Movement axis (x or y)')
    parser.add_argument('--rate', type=float, default=20.0, help='Publish rate in Hz')

    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MultiDynamicObstacleMover(
        topic_names=parsed_args.topic,
        speed=parsed_args.speed,
        duration=parsed_args.duration,
        axis=parsed_args.axis,
        rate=parsed_args.rate
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
