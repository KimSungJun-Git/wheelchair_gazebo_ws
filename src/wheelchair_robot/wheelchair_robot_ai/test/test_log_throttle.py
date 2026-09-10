#!/usr/bin/env python3
"""과거 주행로그를 리플레이해 throttle이 edge를 하나도 잃지 않는지 검증."""
import glob
import json
import os
import tempfile

import rclpy
from std_msgs.msg import String

from wheelchair_robot_ai import log_collector_node as lc

LOG_DIR = os.path.expanduser('~/wheelchair_ws/driving_data')


def replay(path, tmp, throttle_sec):
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            if o.get('_event_marker') or o.get('_session_marker'):
                continue
            if o.get('timestamp'):
                rows.append(o)

    node = lc.LogCollectorNode()
    assert node.save_dir == tmp, '테스트가 실제 driving_data를 건드리고 있다'
    node.throttle_sec = throttle_sec

    real_time = lc.time.time
    try:
        for o in rows:
            lc.time.time = lambda ts=o['timestamp']: ts
            node.log_callback(String(data=json.dumps(
                {k: o.get(k) for k in ('source', 'action', 'reason')})))
    finally:
        lc.time.time = real_time
        node.destroy_node()

    return rows, list(node.log_buffer)


def edges(rows):
    """상태(action, reason) 전이 시퀀스"""
    out, prev = [], object()
    for r in rows:
        key = (r.get('action'), r.get('reason'))
        if key != prev:
            out.append(key)
            prev = key
    return out


def main():
    tmp = tempfile.mkdtemp()
    # 노드가 __init__에서 바로 세션 파일을 열기 때문에 오버라이드를 init에 실어 보낸다
    rclpy.init(args=['--ros-args', '-p', f'save_dir:={tmp}'])
    try:
        files = sorted(glob.glob(f'{LOG_DIR}/*.json'), key=os.path.getsize, reverse=True)
        if not files:
            # 갓 만든 워크스페이스에는 주행로그가 없다 — 실패가 아니라 건너뛴다
            print(f'SKIP — 검증할 주행로그가 없다: {LOG_DIR}')
            return

        for path in files[:3]:
            rows, kept = replay(path, tmp, throttle_sec=1.0)
            assert rows, path

            # 1) 상태 전이는 단 하나도 사라지지 않는다
            assert edges(kept) == edges(rows), f'edge 손실: {os.path.basename(path)}'

            # 2) 남은 로그는 throttle 간격보다 촘촘하지 않다 (같은 상태 구간에서)
            prev_key, prev_ts = object(), None
            for r in kept:
                key = (r.get('action'), r.get('reason'))
                if key == prev_key:
                    assert r['timestamp'] - prev_ts >= 1.0 - 1e-9, 'throttle 미적용'
                prev_key, prev_ts = key, r['timestamp']

            print(f'{os.path.basename(path)}: {len(rows)} → {len(kept)}건 '
                  f'({100 * (1 - len(kept) / len(rows)):.1f}% 감소), '
                  f'전이 {len(edges(rows))}개 전부 보존')
    finally:
        rclpy.shutdown()
    print('OK')


if __name__ == '__main__':
    main()
