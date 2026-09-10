#!/usr/bin/env python3
import os
import numpy as np
import cv2
from PIL import Image, ImageDraw


def generate_test_zone_2d_map(output_dirs, map_name="test_zone_map", res=0.05):
    """
    직선 하강형 점진적 확장 5-Zone 맵 (14m × 30m)
    Nav2용 Ground Truth 2D Occupancy Grid Map (PGM & YAML) 생성

    복도: 4m → 6m → 10m → 14m(Open) → 6m(Recovery)
    """
    origin_x = -2.0
    origin_y = -2.0
    map_len_x = 18.0   # covers -2 to 16
    map_len_y = 34.0   # covers -2 to 32

    width_px = int(round(map_len_x / res))   # 360 px
    height_px = int(round(map_len_y / res))  # 680 px

    img = Image.new("L", (width_px, height_px), color=254)
    draw = ImageDraw.Draw(img)

    def m_to_px(x, y):
        px = int(round((x - origin_x) / res))
        py = height_px - int(round((y - origin_y) / res))
        return px, py

    def add_box(cx, cy, dx, dy):
        px_min, py_max = m_to_px(cx - dx / 2.0, cy - dy / 2.0)
        px_max, py_min = m_to_px(cx + dx / 2.0, cy + dy / 2.0)
        draw.rectangle([min(px_min, px_max), min(py_min, py_max),
                        max(px_min, px_max), max(py_min, py_max)], fill=0)

    def add_cyl(cx, cy, r):
        px_min, py_max = m_to_px(cx - r, cy - r)
        px_max, py_min = m_to_px(cx + r, cy + r)
        draw.ellipse([min(px_min, px_max), min(py_min, py_max),
                      max(px_min, px_max), max(py_min, py_max)], fill=0)

    # === 폐곡선 외곽 벽체 20개 (시계 방향) ===
    # 1. north_wall
    add_box(7.0, 30.0, 4.2, 0.2)
    # 2. z12_east_wall
    add_box(9.0, 26.0, 0.2, 8.2)
    # 3. z3s1_east_cap
    add_box(9.5, 22.0, 1.2, 0.2)
    # 4. z3s1_east_wall
    add_box(10.0, 20.5, 0.2, 3.2)
    # 5. z3s2_east_cap
    add_box(11.0, 19.0, 2.2, 0.2)
    # 6. z3s2_east_wall
    add_box(12.0, 17.5, 0.2, 3.2)
    # 7. z4_east_cap
    add_box(13.0, 16.0, 2.2, 0.2)
    # 8. z4_east_wall
    add_box(14.0, 10.0, 0.2, 12.2)
    # 9. z5_east_block
    add_box(12.0, 4.0, 4.2, 0.2)
    # 10. z5_east_wall
    add_box(10.0, 2.0, 0.2, 4.2)
    # 11. south_wall
    add_box(7.0, 0.0, 6.2, 0.2)
    # 12. z5_west_wall
    add_box(4.0, 2.0, 0.2, 4.2)
    # 13. z5_west_block
    add_box(2.0, 4.0, 4.2, 0.2)
    # 14. z4_west_wall
    add_box(0.0, 10.0, 0.2, 12.2)
    # 15. z4_west_cap
    add_box(1.0, 16.0, 2.2, 0.2)
    # 16. z3s2_west_wall
    add_box(2.0, 17.5, 0.2, 3.2)
    # 17. z3s2_west_cap
    add_box(3.0, 19.0, 2.2, 0.2)
    # 18. z3s1_west_wall
    add_box(4.0, 20.5, 0.2, 3.2)
    # 19. z3s1_west_cap
    add_box(4.5, 22.0, 1.2, 0.2)
    # 20. z12_west_wall
    add_box(5.0, 26.0, 0.2, 8.2)

    # === Zone 5 랜드마크 ===
    add_cyl(5.5, 3.2, 0.25)     # z5_landmark_cyl_nw
    add_box(8.5, 3.2, 0.5, 0.5) # z5_landmark_box_ne
    add_box(5.5, 1.0, 0.5, 0.5) # z5_landmark_box_sw
    add_cyl(8.5, 1.0, 0.25)     # z5_landmark_cyl_se

    img_np = np.array(img)

    yaml_content = f"""image: {map_name}.pgm
resolution: {res}
origin: [{origin_x:.2f}, {origin_y:.2f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

    for target_dir in output_dirs:
        os.makedirs(target_dir, exist_ok=True)
        pgm_path = os.path.join(target_dir, f"{map_name}.pgm")
        yaml_path = os.path.join(target_dir, f"{map_name}.yaml")

        cv2.imwrite(pgm_path, img_np)
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(yaml_content)
        print(f"✅ 2D 맵 생성 완료: {pgm_path}, {yaml_path}")


if __name__ == "__main__":
    targets = [
        "/home/kim/wheelchair_gazebo_ws/world",
        "/home/kim/wheelchair_gazebo_ws/src/wheelchair_robot/wheelchair_robot_navigation2/map",
        "/home/kim/wheelchair_gazebo_ws/models/test_zone_map",
        "/home/kim/model_editor_models"
    ]
    generate_test_zone_2d_map(targets)
