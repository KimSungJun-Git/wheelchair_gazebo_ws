#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import yaml

def generate_map():
    ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    sdf_path = os.path.join(ws_dir, 'models/Local_0904/model.sdf')
    world_path = os.path.join(ws_dir, 'world/Local_0904.world')

    res = 0.05
    origin_x, origin_y = -13.0, -13.0
    width_m, height_m = 39.0, 26.0
    width_px = int(round(width_m / res))   # 780
    height_px = int(round(height_m / res)) # 520

    # 254: Free space (흰색)
    img = np.ones((height_px, width_px), dtype=np.uint8) * 254

    def world_to_pixel(x, y):
        px = int(round((x - origin_x) / res))
        py = height_px - 1 - int(round((y - origin_y) / res))
        return px, py

    def draw_box(x, y, yaw, dx, dy):
        corners = np.array([
            [-dx/2, -dy/2],
            [ dx/2, -dy/2],
            [ dx/2,  dy/2],
            [-dx/2,  dy/2]
        ])
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        rot = corners @ R.T + np.array([x, y])
        pts = np.array([world_to_pixel(pt[0], pt[1]) for pt in rot], dtype=np.int32)
        cv2.fillPoly(img, [pts], 0)

    # 1. model.sdf 파싱 (21개 벽체)
    print("▶ model.sdf 벽체 파싱 중...")
    tree = ET.parse(sdf_path)
    wall_count = 0
    for link in tree.getroot().findall('.//link'):
        pose_elem = link.find('pose')
        box_elem = link.find('.//geometry/box/size')
        if pose_elem is not None and box_elem is not None:
            pose = [float(v) for v in pose_elem.text.strip().split()]
            box = [float(v) for v in box_elem.text.strip().split()]
            draw_box(pose[0], pose[1], pose[5], box[0], box[1])
            wall_count += 1
    print(f"  - 총 {wall_count}개 벽체 렌더링 완료")

    # 정적 장애물 제외 - 순수 벽체만 맵에 반영
    print("▶ 정적 장애물 제외 완료 (벽체 전용 맵)")

    # 대상 경로 목록
    target_dirs = [
        os.path.join(ws_dir, 'src/wheelchair_robot/wheelchair_robot_navigation2/map'),
        os.path.join(ws_dir, 'models/Local_0904'),
        os.path.join(ws_dir, 'world'),
    ]

    map_yaml_data = {
        'image': 'Local_0904.pgm',
        'resolution': res,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    for d in target_dirs:
        os.makedirs(d, exist_ok=True)
        pgm_path = os.path.join(d, 'Local_0904.pgm')
        yaml_path = os.path.join(d, 'Local_0904.yaml')
        cv2.imwrite(pgm_path, img)
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(map_yaml_data, f, default_flow_style=False, sort_keys=False)
        print(f"✅ 저장 완료: {pgm_path} & {yaml_path}")

    # 마스크 파일도 Local_0904 크기에 맞춰 생성 (기본: 장애물 없는 깨끗한 마스크)
    nav_map_dir = os.path.join(ws_dir, 'src/wheelchair_robot/wheelchair_robot_navigation2/map')
    
    # keepout_mask: 254 (Free = 진입 가능)
    keepout_img = np.ones((height_px, width_px), dtype=np.uint8) * 254
    cv2.imwrite(os.path.join(nav_map_dir, 'keepout_mask.pgm'), keepout_img)
    with open(os.path.join(nav_map_dir, 'keepout_mask.yaml'), 'w', encoding='utf-8') as f:
        yaml.dump({
            'image': 'keepout_mask.pgm',
            'mode': 'trinary',
            'resolution': res,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }, f, default_flow_style=False, sort_keys=False)
    print(f"✅ keepout_mask 동기화 완료: {height_px}x{width_px}")

    # speed_mask: 100 (100% 속도 허용)
    speed_img = np.ones((height_px, width_px), dtype=np.uint8) * 100
    cv2.imwrite(os.path.join(nav_map_dir, 'speed_mask.pgm'), speed_img)
    with open(os.path.join(nav_map_dir, 'speed_mask.yaml'), 'w', encoding='utf-8') as f:
        yaml.dump({
            'image': 'speed_mask.pgm',
            'mode': 'scale',
            'resolution': res,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }, f, default_flow_style=False, sort_keys=False)
    print(f"✅ speed_mask 동기화 완료: {height_px}x{width_px}")

if __name__ == '__main__':
    generate_map()
