#!/usr/bin/env python3
"""
click_map_coords.py
Nav2 2D 맵(PGM & YAML)을 열어 마우스 클릭으로 실시간 미터 좌표(X, Y)를 확인하고
복사할 수 있는 편리한 좌표 추출 도구.
"""
import os
import sys
import yaml
import cv2
import numpy as np


class MapCoordinatePicker:
    def __init__(self, yaml_path):
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"YAML 파일을 찾을 수 없습니다: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as f:
            self.map_info = yaml.safe_load(f)

        self.res = float(self.map_info.get("resolution", 0.05))
        self.origin_x = float(self.map_info["origin"][0])
        self.origin_y = float(self.map_info["origin"][1])

        map_dir = os.path.dirname(yaml_path)
        image_name = self.map_info.get("image", "Local_0904.pgm")
        self.image_path = os.path.join(map_dir, image_name)

        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"PGM 이미지 파일을 찾을 수 없습니다: {self.image_path}")

        self.img_raw = cv2.imread(self.image_path)
        self.img_h, self.img_w = self.img_raw.shape[:2]
        self.display_img = self.img_raw.copy()

        self.clicked_points = []
        self.curr_mouse_m = (0.0, 0.0)

    def px_to_m(self, px, py):
        """이미지 픽셀 (px, py) -> Gazebo/Nav2 월드 좌표계 (X, Y) 미터 변환"""
        xm = self.origin_x + px * self.res
        ym = self.origin_y + (self.img_h - 1 - py) * self.res
        return round(xm, 2), round(ym, 2)

    def mouse_callback(self, event, x, y, flags, param):
        self.curr_mouse_m = self.px_to_m(x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            xm, ym = self.curr_mouse_m
            self.clicked_points.append((xm, ym, x, y))
            idx = len(self.clicked_points)

            print(f"\n📍 [Point #{idx}] 선택됨:")
            print(f"   ▶ X: {xm:6.2f} m,  Y: {ym:6.2f} m")
            print(f"   ▶ 정적 장애물 복사용:  x: {xm:.2f}, y: {ym:.2f}")
            print(f"   ▶ 동적 웨이포인트 복사용:  - [{xm:.2f}, {ym:.2f}]")

            # 화면에 점과 번호 표시
            cv2.circle(self.display_img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(
                self.display_img,
                f"#{idx} ({xm:.1f},{ym:.1f})",
                (x + 8, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 200),
                1,
                cv2.LINE_AA,
            )

            # 이전 점이 있으면 선으로 연결
            if len(self.clicked_points) >= 2:
                prev_x, prev_y = self.clicked_points[-2][2:4]
                cv2.line(self.display_img, (prev_x, prev_y), (x, y), (255, 0, 0), 1)
                dist = np.hypot(xm - self.clicked_points[-2][0], ym - self.clicked_points[-2][1])
                print(f"   📏 이전 점과의 거리: {dist:.2f} m")

    def run(self):
        window_name = "Nav2 Map Coordinate Picker - Local_0904"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, min(1200, self.img_w * 2), min(800, self.img_h * 2))
        cv2.setMouseCallback(window_name, self.mouse_callback)

        print("=" * 65)
        print("  🧭 Nav2 Map Coordinate Picker 실행 중")
        print("  - 마우스 좌클릭: 원하는 위치 좌표 추출 (터미널에 출력)")
        print("  - 'c' 키: 클릭한 점 초기화")
        print("  - 'q' 또는 ESC 키: 종료")
        print("=" * 65)

        while True:
            temp_view = self.display_img.copy()
            # 하단 상태 표시줄
            status_text = f"Mouse: X = {self.curr_mouse_m[0]:6.2f} m, Y = {self.curr_mouse_m[1]:6.2f} m  |  Points: {len(self.clicked_points)}"
            cv2.putText(
                temp_view,
                status_text,
                (15, self.img_h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (50, 50, 220),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(window_name, temp_view)
            key = cv2.waitKey(20) & 0xFF
            if key == 27 or key == ord("q"):
                break
            elif key == ord("c"):
                self.clicked_points.clear()
                self.display_img = self.img_raw.copy()
                print("\n🔄 모든 점이 초기화되었습니다.")

        cv2.destroyAllWindows()


def main():
    default_yaml = "/home/kim/wheelchair_gazebo_ws/world/Local_0904.yaml"
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml
    picker = MapCoordinatePicker(yaml_path)
    picker.run()


if __name__ == "__main__":
    main()
