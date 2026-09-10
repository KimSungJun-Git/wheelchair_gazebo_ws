#!/usr/bin/env python3
"""
coord_picker.py
RViz2의 'Publish Point' 도구로 맵 위를 클릭했을 때
발행되는 /clicked_point 토픽을 수신하여
정적/동적 장애물 설정에 바로 쓸 수 있는 좌표를 출력해주는 노드.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class CoordinatePicker(Node):
    def __init__(self):
        super().__init__('coord_picker')
        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        self.count = 0
        self.get_logger().info('=' * 60)
        self.get_logger().info('🧭 RViz2 좌표 피커 노드가 시작되었습니다!')
        self.get_logger().info('▶ RViz2 상단 툴바에서 [Publish Point] 버튼을 누른 후,')
        self.get_logger().info('▶ 맵 위 원하는 위치를 클릭하면 좌표가 터미널에 출력됩니다.')
        self.get_logger().info('=' * 60)

    def point_callback(self, msg: PointStamped):
        self.count += 1
        x = round(msg.point.x, 2)
        y = round(msg.point.y, 2)
        frame = msg.header.frame_id

        print(f"\n📍 [RViz 클릭 #{self.count}] (Frame: {frame}):")
        print(f"   ▶ 좌표:  X = {x:6.2f} m,  Y = {y:6.2f} m")
        print(f"   ▶ 정적 장애물 YAML 형식:  x: {x:.2f}, y: {y:.2f}")
        print(f"   ▶ 동적 웨이포인트 YAML 형식:  - [{x:.2f}, {y:.2f}]")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
