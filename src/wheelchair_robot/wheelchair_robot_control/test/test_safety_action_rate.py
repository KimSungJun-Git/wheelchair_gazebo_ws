#!/usr/bin/env python3
"""/safety_action이 상태 전이에만 발행되고, 반복은 heartbeat 1건으로 줄어드는지 검증."""
import rclpy
from geometry_msgs.msg import Twist

from wheelchair_robot_control.safety_stop_node import SafetyStopNode


def main():
    rclpy.init()
    node = SafetyStopNode()
    published = []
    node.action_pub.publish = lambda msg: published.append(msg)   # 발행 가로채기

    cmd = Twist()
    cmd.linear.x = 0.2

    # 1) 정상 주행 명령 200회(=Nav2 20Hz로 10초) → 상태는 계속 allowed
    for _ in range(200):
        node.nav_cmd_callback(cmd)
    assert len(published) == 1, f'allowed 반복이 그대로 샌다: {len(published)}건'

    # 2) 전방 장애물 진입 → 즉시 발행 (heartbeat를 기다리지 않는다)
    node.dist_front = 0.05
    node.nav_cmd_callback(cmd)
    assert len(published) == 2, '상태 전이가 즉시 발행되지 않았다'
    assert 'modified' in published[-1].data

    # 3) 같은 위험이 지속되는 동안은 추가 발행 없음
    for _ in range(200):
        node.nav_cmd_callback(cmd)
    assert len(published) == 2, f'지속 상태가 도배된다: {len(published)}건'

    # 4) heartbeat는 지속 구간에도 1건씩 시간축을 남긴다
    node.publish_action_heartbeat()
    assert len(published) == 3, 'heartbeat가 안 나온다'

    # 5) 명령이 끊기면 idle로 기록돼 정지 구간도 로그에 남는다
    node.last_cmd_time = None
    node.publish_action_heartbeat()
    assert 'idle' in published[-1].data, '정지 구간이 기록되지 않는다'

    print(f'OK — 명령 401회 → 발행 {len(published)}건 '
          f'({100 * (1 - len(published) / 401):.1f}% 감소)')
    node.destroy_node()

    check_cmd_watchdog()
    rclpy.shutdown()


def check_cmd_watchdog():
    """auto 모드에서 /cmd_vel_safe가 끊기면 마지막 속도가 아니라 정지가 나가야 한다."""
    from wheelchair_robot_control.mode_switch_node import ModeSwitchNode

    node = ModeSwitchNode()
    sent = []
    node.cmd_pub.publish = lambda msg: sent.append(msg)
    node.mode = 'auto'

    moving = Twist()
    moving.linear.x = 0.3
    node.nav_cmd_callback(moving)

    node.control_loop()
    assert sent[-1].linear.x == 0.3, '정상 구간에서 명령이 안 나간다'

    node.last_nav_time -= 10.0          # 명령이 10초간 끊긴 상황
    node.control_loop()
    assert sent[-1].linear.x == 0.0, '명령이 끊겼는데 마지막 속도가 계속 나간다'

    print('OK — /cmd_vel_safe 끊김 시 정지 확인')
    node.destroy_node()


if __name__ == '__main__':
    main()
