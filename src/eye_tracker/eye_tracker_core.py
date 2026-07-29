"""眼球追踪核心算法模块。

职责分层：
  1) Normalizer        — 归一化器（基于极值向量将 3D 注视向量映射到 [-1, 1]²）
  2) GazeVectorTracker — 瞳孔检测 + 视线向量计算 + 归一化 + 调试绘制
"""

import cv2
import math
import random
import time
import yaml
import os
import logging
import numpy as np
from typing import Optional, List, Dict, Tuple

logger = logging.getLogger(__name__)

# V4L2 四字符码整数值（直接设置 CAP_PROP_FOURCC 使用）
_V4L2_FOURCC_MAP = {
    "YUYV": 0x56595559,
    "MJPG": 0x47504A4D,
    "NV12": 0x3231564E,
    "H264": 0x34363248,
    "BGR3": 0x33524742,
    "RGB3": 0x33424752,
}


# ============================================================
# Normalizer：基于极值向量的正交投影归一化
# ============================================================

class Normalizer:
    """将 3D 注视向量（gaze_rotated）映射为屏幕坐标 [-1, 1]² 的归一化器。

    标定文件（extreme_vectors.yaml）需包含四个极值点：
      {side}_inner, {side}_outer, {side}_up, {side}_down

    无此文件时 normalize() 返回 eye_x / eye_y 均为 None，
    下游调用者须自行降级处理。
    """

    def __init__(self, extreme_file: str = "extreme_vectors.yaml"):
        self._extreme_file = os.path.join(os.path.dirname(__file__), extreme_file)
        self._extremes: Dict[str, List[float]] = {}
        self._load_extremes()

    def _load_extremes(self) -> None:
        try:
            with open(self._extreme_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self._extremes = {k: list(v) for k, v in data.items()}
            logger.info("已加载 %d 个极值向量", len(self._extremes))
        except FileNotFoundError:
            self._extremes = {}
            logger.info("极值向量文件不存在，归一化将返回 None")

    def reload(self) -> None:
        """重新加载极值文件。主进程保存新极值后调用此方法刷新缓存。"""
        self._load_extremes()

    def set_extreme(self, side: str, direction: str, vector: List[float]) -> None:
        key = f"{side}_{direction}"
        self._extremes[key] = vector
        try:
            with open(self._extreme_file, 'w', encoding='utf-8') as f:
                yaml.dump(self._extremes, f, allow_unicode=True)
        except Exception as e:
            logger.error(f"保存极值向量失败: {e}")

    def clear_extremes(self, side: str) -> None:
        """清除指定眼（left/right）的所有极值向量，并从 YAML 文件中移除。"""
        keys_to_remove = [k for k in self._extremes if k.startswith(f"{side}_")]
        for k in keys_to_remove:
            del self._extremes[k]
        try:
            with open(self._extreme_file, 'w', encoding='utf-8') as f:
                yaml.dump(self._extremes, f, allow_unicode=True)
            logger.info("已清除 %s 眼 %d 个极值向量", side, len(keys_to_remove))
        except Exception as e:
            logger.error(f"保存清除后的极值向量文件失败: {e}")
        # 也清除受影响的侧键以便 reload 能获取最新状态
    def normalize(self, side: str, gaze_rotated: List[float]) -> Dict[str, Optional[float]]:
        result = {"eye_x": None, "eye_y": None, "missing": []}
        if not gaze_rotated or len(gaze_rotated) != 3:
            return result
        inner_key = f"{side}_inner"
        outer_key = f"{side}_outer"
        up_key = f"{side}_up"
        down_key = f"{side}_down"
        expected = [inner_key, outer_key, up_key, down_key]
        missing = [k.split("_", 1)[1] for k in expected if k not in self._extremes]
        if missing:
            result["missing"] = missing
            return result
        eye_x, eye_y = self._orthogonal_project(
            gaze_rotated,
            self._extremes[inner_key], self._extremes[outer_key],
            self._extremes[up_key], self._extremes[down_key],
        )
        result["eye_x"] = eye_x
        result["eye_y"] = eye_y
        return result

    @staticmethod
    def _orthogonal_project(current, inner, outer, up, down):
        cx = inner[0] + outer[0] + up[0] + down[0]
        cy = inner[1] + outer[1] + up[1] + down[1]
        cz = inner[2] + outer[2] + up[2] + down[2]
        c_len = math.sqrt(cx * cx + cy * cy + cz * cz)
        if c_len < 1e-12:
            return None, None
        cx /= c_len; cy /= c_len; cz /= c_len

        raw_ex = [inner[0] - outer[0], inner[1] - outer[1], inner[2] - outer[2]]
        d_ex = raw_ex[0] * cx + raw_ex[1] * cy + raw_ex[2] * cz
        ex = [raw_ex[0] - d_ex * cx, raw_ex[1] - d_ex * cy, raw_ex[2] - d_ex * cz]
        ex_len = math.sqrt(ex[0]**2 + ex[1]**2 + ex[2]**2)
        if ex_len < 1e-12:
            return None, None
        ex = [ex[0] / ex_len, ex[1] / ex_len, ex[2] / ex_len]

        ey = [cy * ex[2] - cz * ex[1], cz * ex[0] - cx * ex[2], cx * ex[1] - cy * ex[0]]
        ey_len = math.sqrt(ey[0]**2 + ey[1]**2 + ey[2]**2)
        if ey_len < 1e-12:
            return None, None
        ey = [ey[0] / ey_len, ey[1] / ey_len, ey[2] / ey_len]

        def dot(a, b):
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        outer_x, inner_x = dot(outer, ex), dot(inner, ex)
        down_y, up_y = dot(down, ey), dot(up, ey)
        cur_x, cur_y = dot(current, ex), dot(current, ey)

        def clamp_map(val, lo, hi):
            if hi - lo < 1e-12:
                return 0.0
            val = max(lo, min(val, hi))
            return 2.0 * (val - lo) / (hi - lo) - 1.0

        return clamp_map(cur_x, outer_x, inner_x), clamp_map(cur_y, down_y, up_y)


# ============================================================
# GazeVectorTracker：瞳孔检测 + 视线向量计算
# ============================================================

class GazeVectorTracker:
    """瞳孔检测 → 视线方向计算 → 归一化 → 结果回传。

    通过 multiprocessing.Queue 与主进程通信：
      - result_queue: 每帧产出 {side, gaze_rotated, eye_x, eye_y, confidence}
      - command_queue: 接收锁定/解锁定/headless 切换/极值重载命令

    可在 headless 模式下运行（无 X11 环境），不创建 OpenCV 窗口。
    """

    def __init__(
        self,
        cam_index: int = 0,
        flip: bool = False,
        crop: Optional[List[int]] = None,
        side: str = "left",
        frame_width: int = 640,
        frame_height: int = 480,
        frame_rate: int = 30,
        extreme_file: str = "extreme_vectors.yaml",
        use_recommended_resolution: bool = True,
        dark_search_roi_scale: float = 0.70,
        fourcc_str: str = "",
        brightness: float = 0.0,
        contrast: float = 1.0,
    ):
        self.cam_index = cam_index
        self.flip = flip
        self.crop = crop if crop is not None else [0, 0, frame_width, frame_height]
        self.side = side
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_rate = frame_rate
        self._fourcc_str = fourcc_str
        self._use_recommended_resolution = use_recommended_resolution
        self._dark_search_roi_scale = max(0.1, min(1.0, dark_search_roi_scale))
        self._brightness = brightness
        self._contrast = contrast

        # ---- 归一化器 ----
        self._normalizer = Normalizer(extreme_file)

        # ---- 追踪状态 ----
        self.ray_lines: list = []
        self.model_centers: list = []
        self.min_model_centers = 30
        self.max_rays = 100
        self.prev_model_center_avg = (self.frame_width // 2, self.frame_height // 2)
        self.max_observed_distance = 0
        self.last_sphere_radius_ellipse = None
        self.pupil_confidence_threshold = 0.85
        self.pupil_confidence_threshold_sphere = 0.65
        self.intersection_ray_count = 4
        self.minimum_intersection_angle_degrees = 8
        self.last_tracking_result = None
        self.stored_intersections: list = []

        # ---- 锁定状态 ----
        self.sphere_radius_locked = False
        self.locked_sphere_radius = 0
        self.eye_center_locked = False
        self.locked_eye_center = (self.frame_width // 2, self.frame_height // 2)

        # ---- 运行状态 ----
        self.cap = None
        self.running = False
        self._headless = False
        self.result_queue = None
        self._win_name = None
        self._last_search_ellipse = None  # for debug overlay: (cx, cy, rx, ry)

    # ==================== 锁定函数 ====================

    def lock_sphere_radius(self):
        if self.max_observed_distance > 0:
            self.sphere_radius_locked = True
            self.locked_sphere_radius = self.max_observed_distance
            print(f"[{self.side}] 眼球半径已锁定: {self.locked_sphere_radius:.1f}")
        else:
            print(f"[{self.side}] 眼球半径仍为0，无法锁定")

    def unlock_sphere_radius(self):
        self.sphere_radius_locked = False
        print(f"[{self.side}] 眼球半径已解锁")

    def lock_eye_center(self):
        if self.prev_model_center_avg[0] != self.frame_width // 2:
            self.eye_center_locked = True
            self.locked_eye_center = self.prev_model_center_avg
            print(f"[{self.side}] 眼球中心已锁定: {self.locked_eye_center}")
        else:
            print(f"[{self.side}] 眼球中心尚未稳定，无法锁定")

    def unlock_eye_center(self):
        self.eye_center_locked = False
        print(f"[{self.side}] 眼球中心已解锁")

    # ==================== 核心主循环 ====================

    def start_tracking(
        self,
        command_queue=None,
        result_queue=None,
        headless: bool = False,
    ):
        """打开摄像机并进入主循环（阻塞）。

        Parameters
        ----------
        command_queue : multiprocessing.Queue | None
            支持命令: lock_radius, unlock_radius, lock_center, unlock_center,
                      headless_on, headless_off,
                      ("save_extreme", direction_str, [x,y,z])
        result_queue : multiprocessing.Queue | None
            每帧回传: {side, gaze_rotated, eye_x, eye_y, confidence}
        headless : bool
            True 时跳过所有 OpenCV GUI 操作。
        """
        self._headless = headless
        self._reset_tracking_state()

        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            self.cap = cv2.VideoCapture(self.cam_index, backend)
            if self.cap.isOpened():
                break

        if not self.cap.isOpened():
            print(f"错误：无法打开摄像机 (索引 {self.cam_index})")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        if self._fourcc_str and self._fourcc_str in _V4L2_FOURCC_MAP:
            self.cap.set(cv2.CAP_PROP_FOURCC, _V4L2_FOURCC_MAP[self._fourcc_str])

        self.running = True
        print(f"眼球追踪已启动 (cam={self.cam_index}, side={self.side}, "
              f"headless={self._headless})")

        self.result_queue = result_queue

        win_name = f"Eye Tracker - {self.side.upper()} Eye"
        self._win_name = win_name
        if not self._headless:
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.waitKey(1)

        frame_fail_count = 0
        max_fail = 10

        while self.running:
            # ---- 处理命令队列 ----
            if command_queue is not None:
                while not command_queue.empty():
                    try:
                        cmd = command_queue.get_nowait()
                        if cmd == "lock_radius":
                            self.lock_sphere_radius()
                        elif cmd == "unlock_radius":
                            self.unlock_sphere_radius()
                        elif cmd == "lock_center":
                            self.lock_eye_center()
                        elif cmd == "unlock_center":
                            self.unlock_eye_center()
                        elif cmd == "headless_on":
                            self._switch_headless_on()
                        elif cmd == "headless_off":
                            self._switch_headless_off()
                        elif cmd == "reload_extremes":
                            self._normalizer.reload()
                        elif cmd == "clear_extremes":
                            self._normalizer.clear_extremes(self.side)
                        elif isinstance(cmd, tuple) and cmd[0] == "save_extreme":
                            direction, vector = cmd[1], cmd[2]
                            self._normalizer.set_extreme(self.side, direction, vector)
                        elif isinstance(cmd, tuple) and cmd[0] == "set_search_roi_scale":
                            self._dark_search_roi_scale = max(0.1, min(1.0, float(cmd[1])))
                            logger.info(f"[{self.side}] 搜索区域比例: {self._dark_search_roi_scale:.2f}")
                        elif isinstance(cmd, tuple) and cmd[0] == "restart_capture":
                            # 动态重启相机 (w, h, fps, fourcc_str)
                            w, h, fps, fcc = cmd[1], cmd[2], cmd[3], cmd[4]
                            self._restart_camera(w, h, fps, fcc)
                        elif isinstance(cmd, tuple) and cmd[0] == "set_postprocess":
                            # 动态更新明度/对比度
                            self._brightness = float(cmd[1])
                            self._contrast = float(cmd[2])
                            logger.info(f"[{self.side}] 后处理: 明度={self._brightness:.0f}, 对比度={self._contrast:.1f}")
                    except Exception:
                        pass

            ret, frame = self.cap.read()
            if not ret:
                frame_fail_count += 1
                if frame_fail_count >= max_fail:
                    print(f"警告：连续 {max_fail} 次无法读取帧，退出")
                    break
                if self._headless:
                    time.sleep(0.05)
                else:
                    cv2.waitKey(50)
                continue
            frame_fail_count = 0

            if self.flip:
                frame = cv2.flip(frame, 0)

            x1, y1, x2, y2 = self.crop
            if y2 > frame.shape[0] or x2 > frame.shape[1]:
                print(f"警告：crop {self.crop} 超出帧尺寸 {frame.shape}")
                continue
            frame = frame[y1:y2, x1:x2]

            self._process_frame(frame)

            if not self._headless:
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    print("按下退出键，停止追踪。")
                    break
                try:
                    if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                        print(f"[{self.side}] OpenCV 窗口丢失，自动切换为 headless 模式")
                        self._switch_headless_on()
                except cv2.error:
                    print(f"[{self.side}] X11 异常，自动切换为 headless 模式")
                    self._switch_headless_on()

        self._cleanup()

    def _switch_headless_on(self):
        if self._headless:
            return
        try:
            cv2.destroyAllWindows()
            for _ in range(20):
                if cv2.waitKey(1) < 0:
                    break
        except cv2.error:
            pass
        self._headless = True
        print(f"[{self.side}] 已切换到 headless 模式")

    def _switch_headless_off(self):
        if not self._headless:
            return
        self._headless = False
        print(f"[{self.side}] 已退出 headless 模式")
        try:
            if self._win_name:
                cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
                cv2.waitKey(1)
        except cv2.error as e:
            print(f"[{self.side}] 无法重建 OpenCV 窗口: {e}")
            self._headless = True

    def stop(self):
        self.running = False
        self._cleanup()

    def _cleanup(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if not self._headless:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    def _restart_camera(self, w: int, h: int, fps: int, fourcc_str: str):
        """动态重启相机，切换分辨率/帧率/像素格式。"""
        logger.info(f"[{self.side}] 重启相机: {w}x{h} @{fps}fps fourcc={fourcc_str}")
        old_cap = self.cap
        self.cap = None

        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            new_cap = cv2.VideoCapture(self.cam_index, backend)
            if new_cap.isOpened():
                break
        else:
            logger.error(f"[{self.side}] 重启相机失败，保留旧相机")
            self.cap = old_cap
            return

        new_cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        new_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        new_cap.set(cv2.CAP_PROP_FPS, fps)
        if fourcc_str and fourcc_str in _V4L2_FOURCC_MAP:
            new_cap.set(cv2.CAP_PROP_FOURCC, _V4L2_FOURCC_MAP[fourcc_str])

        # 验证实际生效的尺寸
        actual_w = int(new_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(new_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_w != w or actual_h != h:
            logger.warning(f"[{self.side}] 期望 {w}x{h}，实际 {actual_w}x{actual_h}")

        # 按比例缩放 crop（原 crop 基于旧分辨率）
        old_w = self.frame_width
        old_h = self.frame_height
        if old_w > 0 and old_h > 0:
            self.crop = [
                self.crop[0] * w // old_w,
                self.crop[1] * h // old_h,
                self.crop[2] * w // old_w,
                self.crop[3] * h // old_h,
            ]

        self.frame_width = w
        self.frame_height = h
        self.frame_rate = fps
        self._fourcc_str = fourcc_str
        self._reset_tracking_state()

        if old_cap is not None:
            old_cap.release()
        self.cap = new_cap
        logger.info(f"[{self.side}] 相机重启完成: {w}x{h} @{fps}fps")

    def get_last_tracking_result(self):
        return self.last_tracking_result

    # ==================== 单帧处理 ====================

    def _process_frame(self, frame):
        if self._use_recommended_resolution:
            frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        else:
            h, w = frame.shape[:2]
            if h > w:
                new_w, new_h = 480, int(480 * h / w)
            else:
                new_w, new_h = 640, int(640 * h / w)
            frame = cv2.resize(frame, (new_w, new_h))
            self.frame_width = new_w
            self.frame_height = new_h
        # 后处理：明度/对比度调整（影响最终瞳孔检测效果）
        if self._contrast != 1.0 or self._brightness != 0.0:
            frame = cv2.convertScaleAbs(frame, alpha=self._contrast, beta=self._brightness)
        # 一次性灰度转换，后续无需重复
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        darkest_point = self._get_darkest_area(frame, gray_frame)
        if darkest_point is None:
            self.last_tracking_result = None
            return
        darkest_pixel_value = gray_frame[darkest_point[1], darkest_point[0]]

        thresholded_strict = self._apply_binary_threshold(
            gray_frame, darkest_pixel_value, 5
        )
        thresholded_strict = self._mask_outside_square(
            thresholded_strict, darkest_point, 250
        )

        thresholded_medium = self._apply_binary_threshold(
            gray_frame, darkest_pixel_value, 15
        )
        thresholded_medium = self._mask_outside_square(
            thresholded_medium, darkest_point, 250
        )

        thresholded_relaxed = self._apply_binary_threshold(
            gray_frame, darkest_pixel_value, 25
        )
        thresholded_relaxed = self._mask_outside_square(
            thresholded_relaxed, darkest_point, 250
        )

        self._process_frames(
            thresholded_strict,
            thresholded_medium,
            thresholded_relaxed,
            frame,
        )

    # ==================== 多阈值融合与瞳孔拟合 ====================

    def _process_frames(
        self,
        thresholded_strict,
        thresholded_medium,
        thresholded_relaxed,
        frame,
    ):
        kernel = np.ones((5, 5), np.uint8)
        image_array = [thresholded_relaxed, thresholded_medium, thresholded_strict]

        final_rotated_rect = None
        final_contours = []
        goodness = 0
        best_ratio_under_ellipse = 0
        best_center_x, best_center_y = None, None

        for i in range(3):
            dilated = cv2.dilate(image_array[i], kernel, iterations=2)
            contours, _ = cv2.findContours(
                dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            reduced = self._filter_contours_by_area_and_return_largest(
                contours, 1000, 3
            )

            if len(reduced) > 0 and len(reduced[0]) > 5:
                # 先 fitEllipse 一次，后续函数复用结果
                ellipse = cv2.fitEllipse(reduced[0])
                center_x, center_y = map(int, ellipse[0])

                current_goodness = self._check_ellipse_goodness(dilated, reduced[0], ellipse)
                total_pixels = self._check_contour_pixels(reduced[0], dilated.shape, ellipse)

                final_goodness = (
                    current_goodness[0]
                    * total_pixels[0]
                    * total_pixels[0]
                    * total_pixels[1]
                )

                if final_goodness > 0 and final_goodness > goodness:
                    goodness = final_goodness
                    best_ratio_under_ellipse = total_pixels[1]
                    final_contours = reduced
                    best_center_x = center_x
                    best_center_y = center_y

        center_x = best_center_x
        center_y = best_center_y

        final_contours = [self._optimize_contours_by_angle(final_contours)]

        if (
            final_contours
            and not isinstance(final_contours[0], list)
            and len(final_contours[0]) > 5
        ):
            ellipse = cv2.fitEllipse(final_contours[0])
            final_rotated_rect = ellipse

            if best_ratio_under_ellipse >= self.pupil_confidence_threshold:
                self.ray_lines.append(final_rotated_rect)
                if len(self.ray_lines) > self.max_rays:
                    self.ray_lines = self.ray_lines[-self.max_rays:]

        if self.eye_center_locked:
            model_center_average = self.locked_eye_center
        else:
            model_center_average = (self.frame_width // 2, self.frame_height // 2)

            model_center = self._compute_average_intersection(
                frame,
                self.ray_lines,
                self.intersection_ray_count,
                1500,
                self.minimum_intersection_angle_degrees,
            )
            if model_center is not None and model_center != (0, 0):
                model_center_average = self._update_and_average_point(
                    self.model_centers, model_center, 200
                )

            if model_center_average[0] == self.frame_width // 2:
                model_center_average = self.prev_model_center_avg
            if model_center_average[0] != 0:
                self.prev_model_center_avg = model_center_average

        if center_x is None or center_y is None:
            self.last_tracking_result = None
            return

        self._update_eye_sphere_radius(
            model_center_average,
            final_rotated_rect,
            best_ratio_under_ellipse,
        )

        self.last_tracking_result = {
            "pupil_ellipse": {
                "center": (
                    [float(final_rotated_rect[0][0]), float(final_rotated_rect[0][1])]
                    if final_rotated_rect is not None
                    else None
                ),
                "axes": (
                    [float(final_rotated_rect[1][0]), float(final_rotated_rect[1][1])]
                    if final_rotated_rect is not None
                    else None
                ),
                "angle_degrees": (
                    float(final_rotated_rect[2])
                    if final_rotated_rect is not None
                    else None
                ),
            }
            if final_rotated_rect is not None
            else None,
            "eye_center": [int(model_center_average[0]), int(model_center_average[1])],
            "sphere_radius": float(self.max_observed_distance),
        }

        center_3d, gaze_rotated, norm_result = self._compute_gaze_vector(
            center_x, center_y,
            model_center_average[0], model_center_average[1],
            best_ratio_under_ellipse,
        )

        if not self._headless:
            self._draw_debug_overlay(
                frame,
                model_center_average,
                final_rotated_rect,
                center_x, center_y,
                center_3d, gaze_rotated,
                best_ratio_under_ellipse,
                norm_result,
            )

    # ==================== 调试绘制 ====================

    def _draw_debug_overlay(
        self,
        frame,
        model_center_average,
        final_rotated_rect,
        center_x, center_y,
        center_3d, gaze_rotated,
        best_ratio_under_ellipse,
        norm_result,
    ):
        # ---- 绘制椭圆搜索区域 ----
        if self._last_search_ellipse is not None:
            cx_e, cy_e, rx_e, ry_e = self._last_search_ellipse
            cv2.ellipse(
                frame,
                (cx_e, cy_e),
                (rx_e, ry_e),
                0, 0, 360,
                (0, 200, 0), 2,
            )

        cv2.circle(
            frame,
            model_center_average,
            int(self.max_observed_distance),
            (255, 50, 50),
            2,
        )
        cv2.circle(frame, model_center_average, 8, (255, 255, 0), -1)

        if final_rotated_rect is not None and center_x is not None and center_y is not None:
            cv2.line(
                frame,
                model_center_average,
                (center_x, center_y),
                (255, 150, 50),
                2,
            )

        if final_rotated_rect is not None:
            cv2.ellipse(frame, final_rotated_rect, (20, 255, 255), 2)

        if final_rotated_rect is not None and center_x is not None and center_y is not None:
            dx = center_x - model_center_average[0]
            dy = center_y - model_center_average[1]
            extended_x = int(model_center_average[0] + 2 * dx)
            extended_y = int(model_center_average[1] + 2 * dy)
            cv2.line(
                frame,
                (center_x, center_y),
                (extended_x, extended_y),
                (200, 255, 0),
                3,
            )

        if center_3d is not None and gaze_rotated is not None:
            origin_text = (
                f"Origin: ({center_3d[0]:.2f}, "
                f"{center_3d[1]:.2f}, {center_3d[2]:.2f})"
            )
            dir_text = (
                f"Direction: ({gaze_rotated[0]:.2f}, "
                f"{gaze_rotated[1]:.2f}, {gaze_rotated[2]:.2f})"
            )

            text_origin_shadow = (12, frame.shape[0] - 38)
            text_dir_shadow = (12, frame.shape[0] - 13)
            text_origin = (10, frame.shape[0] - 40)
            text_dir = (10, frame.shape[0] - 15)

            cv2.putText(
                frame, origin_text, text_origin_shadow,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
            )
            cv2.putText(
                frame, dir_text, text_dir_shadow,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
            )
            cv2.putText(
                frame, origin_text, text_origin,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
            )
            cv2.putText(
                frame, dir_text, text_dir,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
            )

        # ---- 归一化状态 HUD（左下角） ----
        eye_x = norm_result.get("eye_x")
        eye_y = norm_result.get("eye_y")
        missing = norm_result.get("missing", [])

        if missing:
            msg = f"Insufficient extreme vectors! ({', '.join(missing)}) is missing."
            cv2.putText(
                frame, msg, (10, frame.shape[0] - 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
            )
            cv2.putText(
                frame, msg, (10, frame.shape[0] - 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2,
            )
        elif eye_x is not None and eye_y is not None:
            norm_text = f"Eye X: {eye_x:+.3f}   Eye Y: {eye_y:+.3f}"
            cv2.putText(
                frame, norm_text, (10, frame.shape[0] - 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
            )
            cv2.putText(
                frame, norm_text, (10, frame.shape[0] - 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
            )

        ratio_text = f"{best_ratio_under_ellipse * 100:.2f}%"
        cv2.putText(
            frame, ratio_text, (12, 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4,
        )
        cv2.putText(
            frame, ratio_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
        )

        cv2.imshow(f"Eye Tracker - {self.side.upper()} Eye", frame)

    # ==================== 视线向量计算与归一化 ====================

    def _compute_gaze_vector(
        self, x, y, center_x, center_y, confidence_ratio=0.0
    ):
        """根据瞳孔屏幕坐标和眼球中心计算 3D 视线方向。

        结果（含归一化后的 eye_x/eye_y）通过 result_queue 回传。
        """
        viewport_width = self.frame_width
        viewport_height = self.frame_height

        fov_y_deg = 45.0
        aspect_ratio = viewport_width / viewport_height
        far_clip = 100.0

        camera_position = np.array([0.0, 0.0, 3.0])

        fov_y_rad = np.radians(fov_y_deg)
        half_height_far = np.tan(fov_y_rad / 2) * far_clip
        half_width_far = half_height_far * aspect_ratio

        ndc_x = (2.0 * x) / viewport_width - 1.0
        ndc_y = 1.0 - (2.0 * y) / viewport_height

        far_x = ndc_x * half_width_far
        far_y = ndc_y * half_height_far
        far_z = camera_position[2] - far_clip
        far_point = np.array([far_x, far_y, far_z])

        ray_origin = camera_position
        ray_direction = far_point - camera_position
        ray_direction /= np.linalg.norm(ray_direction)
        ray_direction = -ray_direction

        inner_radius = 1.0 / 1.05
        sphere_offset_x = (center_x / viewport_width) * 2.0 - 1.0
        sphere_offset_y = 1.0 - (center_y / viewport_height) * 2.0
        sphere_center = np.array([sphere_offset_x * 1.5, sphere_offset_y * 1.5, 0.0])

        origin = ray_origin
        direction = -ray_direction
        L = origin - sphere_center

        a = np.dot(direction, direction)
        b = 2 * np.dot(direction, L)
        c = np.dot(L, L) - inner_radius ** 2

        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            t = -np.dot(direction, L) / np.dot(direction, direction)
            intersection_point = origin + t * direction
            intersection_local = intersection_point - sphere_center
            target_direction = intersection_local / np.linalg.norm(intersection_local)
        else:
            sqrt_disc = np.sqrt(discriminant)
            t1 = (-b - sqrt_disc) / (2 * a)
            t2 = (-b + sqrt_disc) / (2 * a)

            t = None
            if t1 > 0 and t2 > 0:
                t = min(t1, t2)
            elif t1 > 0:
                t = t1
            elif t2 > 0:
                t = t2
            if t is None:
                return None, None

            intersection_point = origin + t * direction
            intersection_local = intersection_point - sphere_center
            target_direction = intersection_local / np.linalg.norm(intersection_local)

        circle_local_center = np.array([0.0, 0.0, inner_radius])
        circle_local_center /= np.linalg.norm(circle_local_center)

        rotation_axis = np.cross(circle_local_center, target_direction)
        rotation_axis_norm = np.linalg.norm(rotation_axis)
        if rotation_axis_norm < 1e-6:
            gaze_rotated = circle_local_center
        else:
            rotation_axis /= rotation_axis_norm
            dot = np.dot(circle_local_center, target_direction)
            dot = np.clip(dot, -1.0, 1.0)
            angle_rad = np.arccos(dot)

            c_rot = np.cos(angle_rad)
            s_rot = np.sin(angle_rad)
            t_rot = 1 - c_rot
            x_a, y_a, z_a = rotation_axis

            rotation_matrix = np.array([
                [t_rot * x_a * x_a + c_rot,
                 t_rot * x_a * y_a - s_rot * z_a,
                 t_rot * x_a * z_a + s_rot * y_a],
                [t_rot * x_a * y_a + s_rot * z_a,
                 t_rot * y_a * y_a + c_rot,
                 t_rot * y_a * z_a - s_rot * x_a],
                [t_rot * x_a * z_a - s_rot * y_a,
                 t_rot * y_a * z_a + s_rot * x_a,
                 t_rot * z_a * z_a + c_rot],
            ])

            gaze_local = np.array([0.0, 0.0, inner_radius])
            gaze_rotated = rotation_matrix @ gaze_local
            gaze_rotated /= np.linalg.norm(gaze_rotated)

        # ---- 归一化 ----
        gaze_list = [float(v) for v in gaze_rotated]
        norm_result = self._normalizer.normalize(self.side, gaze_list)

        # ---- 回传 ----
        if self.result_queue is not None:
            try:
                self.result_queue.put_nowait({
                    "side": self.side,
                    "gaze_rotated": gaze_list,
                    "eye_x": norm_result["eye_x"],
                    "eye_y": norm_result["eye_y"],
                    "confidence": confidence_ratio,
                })
            except Exception:
                pass

        return sphere_center, gaze_rotated, norm_result

    # ==================== 阈值与遮罩 ====================

    @staticmethod
    def _apply_binary_threshold(image, darkest_pixel_value, added_threshold):
        threshold = darkest_pixel_value + added_threshold
        _, thresholded = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
        return thresholded

    def _get_darkest_area(self, image, gray_frame=None):
        """在椭圆 ROI 内搜索最暗区域（向量化版本）。
        
        用 numpy 批量采样替代四层纯 Python 循环。
        椭圆中心 = 画面中心，长轴 = 宽度/2 * scale，短轴 = 高度/2 * scale。
        返回 (暗点坐标)。
        """
        h, w = image.shape[:2]
        cx_roi, cy_roi = w // 2, h // 2
        rx = int((w / 2) * self._dark_search_roi_scale)
        ry = int((h / 2) * self._dark_search_roi_scale)
        self._last_search_ellipse = (cx_roi, cy_roi, rx, ry)

        if gray_frame is None:
            gray_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ignore_bounds, image_skip_size = 20, 10
        search_area, internal_skip_size = 20, 5

        ys = np.arange(ignore_bounds, h - ignore_bounds, image_skip_size)
        xs = np.arange(ignore_bounds, w - ignore_bounds, image_skip_size)
        if len(ys) == 0 or len(xs) == 0:
            return None

        grid_y, grid_x = np.meshgrid(ys, xs, indexing='ij')
        sy = grid_y + search_area // 2  # 采样点中心 y
        sx = grid_x + search_area // 2  # 采样点中心 x

        # ---- 椭圆过滤 ----
        if rx > 0 and ry > 0:
            in_ellipse = ((sx - cx_roi) / rx) ** 2 + ((sy - cy_roi) / ry) ** 2 <= 1.0
            if not np.any(in_ellipse):
                return None
            sy_v, sx_v = sy[in_ellipse], sx[in_ellipse]
        else:
            sy_v, sx_v = sy.ravel(), sx.ravel()

        n_points = len(sy_v)
        if n_points == 0:
            return None

        # ---- 构建采样偏移 (0,5,10,15) × (0,5,10,15) = 16 个点 ----
        y_offsets = np.arange(0, search_area, internal_skip_size)
        x_offsets = np.arange(0, search_area, internal_skip_size)
        yo, xo = np.meshgrid(y_offsets, x_offsets, indexing='ij')
        yo, xo = yo.ravel(), xo.ravel()  # 每个长度 16

        # ---- 一次性提取所有像素值 (n_points, 16) ----
        all_y = sy_v[:, None] + yo[None, :]
        all_x = sx_v[:, None] + xo[None, :]

        # 边界裁剪
        np.clip(all_y, 0, h - 1, out=all_y)
        np.clip(all_x, 0, w - 1, out=all_x)

        pixels = gray_frame[all_y.astype(np.int32), all_x.astype(np.int32)].astype(np.int32)
        sums = np.sum(pixels, axis=1)

        # ---- 找最小和 ----
        min_idx = np.argmin(sums)
        return (int(sx_v[min_idx]), int(sy_v[min_idx]))

    @staticmethod
    def _mask_outside_square(image, center, size):
        x, y = center
        half = size // 2
        mask = np.zeros_like(image)
        top_left_x = max(0, x - half)
        top_left_y = max(0, y - half)
        bottom_right_x = min(image.shape[1], x + half)
        bottom_right_y = min(image.shape[0], y + half)
        mask[top_left_y:bottom_right_y, top_left_x:bottom_right_x] = 255
        return cv2.bitwise_and(image, mask)

    # ==================== 轮廓处理 ====================

    @staticmethod
    def _filter_contours_by_area_and_return_largest(
        contours, pixel_thresh, ratio_thresh
    ):
        max_area = 0
        largest_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= pixel_thresh:
                x, y, w, h = cv2.boundingRect(contour)
                length_to_width_ratio = max(w / h, h / w)
                if length_to_width_ratio <= ratio_thresh:
                    if area > max_area:
                        max_area = area
                        largest_contour = contour

        return [largest_contour] if largest_contour is not None else []

    @staticmethod
    def _optimize_contours_by_angle(contours):
        """向量化版本：用 numpy 批量操作替代 Python 逐点循环。"""
        if len(contours) < 1:
            return contours

        all_contours = np.concatenate(contours[0], axis=0)  # (N, 2)
        n = len(all_contours)
        if n < 3:
            return contours

        spacing = max(1, int(n / 25))
        centroid = np.mean(all_contours, axis=0)  # (2,)

        # ---- 批量构建 prev/next/current 向量 ----
        # prev_idx[i] = i - spacing, next_idx[i] = i + spacing（环形）
        idx = np.arange(n)
        prev_idx = (idx - spacing) % n
        next_idx = (idx + spacing) % n

        prev_pts = all_contours[prev_idx]  # (N, 2)
        curr_pts = all_contours[idx]       # (N, 2)
        next_pts = all_contours[next_idx]  # (N, 2)

        vec1 = prev_pts - curr_pts         # (N, 2)
        vec2 = next_pts - curr_pts         # (N, 2)
        vec_to_centroid = centroid - curr_pts  # (N, 2)

        # ---- 归一化向量 ----
        n1 = np.linalg.norm(vec_to_centroid, axis=1)  # (N,)
        n2 = np.linalg.norm(vec1 + vec2, axis=1)       # (N,)

        valid = (n1 > 1e-6) & (n2 > 1e-6)
        if not np.any(valid):
            return np.array([], dtype=np.int32).reshape((-1, 1, 2))

        v_dir = np.zeros_like(curr_pts)
        v_dir[valid] = vec_to_centroid[valid] / n1[valid, None]
        v_tangent = np.zeros_like(curr_pts)
        v_tangent[valid] = (vec1[valid] + vec2[valid]) / n2[valid, None]

        # ---- 点积筛选 ----
        cos_threshold = np.cos(np.radians(60))
        dot_products = np.sum(v_dir * v_tangent, axis=1)  # (N,)
        keep = valid & (dot_products >= cos_threshold)

        if not np.any(keep):
            return np.array([], dtype=np.int32).reshape((-1, 1, 2))

        return all_contours[keep].reshape((-1, 1, 2)).astype(np.int32)

    # ==================== 椭圆质量检测 ====================

    @staticmethod
    def _check_ellipse_goodness(binary_image, contour, ellipse=None):
        if len(contour) < 5:
            return [0, 0, 0]

        if ellipse is None:
            ellipse = cv2.fitEllipse(contour)

        # 计算包含椭圆的最小 ROI，大幅减少 mask 创建开销
        h, w = binary_image.shape
        cx, cy = int(ellipse[0][0]), int(ellipse[0][1])
        axis_major = int(max(ellipse[1]) / 2) + 5  # +5 安全边距
        x1 = max(0, cx - axis_major)
        y1 = max(0, cy - axis_major)
        x2 = min(w, cx + axis_major)
        y2 = min(h, cy + axis_major)

        roi_bin = binary_image[y1:y2, x1:x2]
        if roi_bin.size == 0:
            return [0, 0, 0]

        mask = np.zeros(roi_bin.shape, dtype=np.uint8)
        # 调整椭圆参数到 ROI 坐标
        ellipse_roi = (
            (ellipse[0][0] - x1, ellipse[0][1] - y1),
            ellipse[1],
            ellipse[2],
        )
        cv2.ellipse(mask, ellipse_roi, (255), -1)

        ellipse_area = np.sum(mask == 255)
        if ellipse_area == 0:
            return [0, 0, 0]

        covered_pixels = np.sum((roi_bin == 255) & (mask == 255))
        goodness = [0, 0, 0]
        goodness[0] = covered_pixels / ellipse_area
        goodness[2] = min(
            ellipse[1][1] / ellipse[1][0], ellipse[1][0] / ellipse[1][1]
        )

        return goodness

    @staticmethod
    def _check_contour_pixels(contour, image_shape, ellipse=None):
        if len(contour) < 5:
            return [0, 0]

        if ellipse is None:
            ellipse = cv2.fitEllipse(contour)

        # 计算包含轮廓和椭圆的最小 ROI
        h, w = image_shape
        cx, cy = int(ellipse[0][0]), int(ellipse[0][1])
        axis_major = int(max(ellipse[1]) / 2) + 15  # +15 安全边距（含10px厚轨迹）
        x1 = max(0, cx - axis_major)
        y1 = max(0, cy - axis_major)
        x2 = min(w, cx + axis_major)
        y2 = min(h, cy + axis_major)

        roi_w, roi_h = x2 - x1, y2 - y1
        if roi_w <= 0 or roi_h <= 0:
            return [0, 0]

        # ---- 在 ROI 内创建 mask ----
        contour_mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        if len(contour) > 0:
            # 将轮廓坐标偏移到 ROI 空间
            shifted_contour = [(pt[0][0] - x1, pt[0][1] - y1) for pt in contour]
            shifted_contour = np.array([shifted_contour], dtype=np.int32)
            cv2.drawContours(contour_mask, shifted_contour, -1, (255), 1)

        ellipse_mask_thick = np.zeros((roi_h, roi_w), dtype=np.uint8)
        ellipse_mask_thin = np.zeros((roi_h, roi_w), dtype=np.uint8)
        # 调整椭圆参数到 ROI 坐标
        ellipse_roi = (
            (ellipse[0][0] - x1, ellipse[0][1] - y1),
            ellipse[1],
            ellipse[2],
        )
        cv2.ellipse(ellipse_mask_thick, ellipse_roi, (255), 10)
        cv2.ellipse(ellipse_mask_thin, ellipse_roi, (255), 4)

        overlap_thick = cv2.bitwise_and(contour_mask, ellipse_mask_thick)
        overlap_thin = cv2.bitwise_and(contour_mask, ellipse_mask_thin)

        absolute_pixel_total_thick = np.sum(overlap_thick > 0)
        absolute_pixel_total_thin = np.sum(overlap_thin > 0)

        total_border_pixels = np.sum(contour_mask > 0)
        ratio_under_ellipse = (
            absolute_pixel_total_thin / total_border_pixels
            if total_border_pixels > 0
            else 0
        )

        return [absolute_pixel_total_thick, ratio_under_ellipse]

    # ==================== 眼球中心与半径估计 ====================

    @staticmethod
    def _distance_to_pupil_outer_edge(eye_center, pupil_ellipse):
        pupil_center, axes, angle_degrees = pupil_ellipse
        direction_x = pupil_center[0] - eye_center[0]
        direction_y = pupil_center[1] - eye_center[1]
        center_distance = math.hypot(direction_x, direction_y)

        semi_axis_x = axes[0] / 2
        semi_axis_y = axes[1] / 2
        if center_distance == 0 or semi_axis_x <= 0 or semi_axis_y <= 0:
            return None

        unit_x = direction_x / center_distance
        unit_y = direction_y / center_distance
        angle_radians = math.radians(angle_degrees)
        cosine = math.cos(angle_radians)
        sine = math.sin(angle_radians)

        local_x = cosine * unit_x + sine * unit_y
        local_y = -sine * unit_x + cosine * unit_y
        edge_offset = 1 / math.sqrt(
            (local_x / semi_axis_x) ** 2 + (local_y / semi_axis_y) ** 2
        )

        return center_distance + edge_offset

    def _update_eye_sphere_radius(
        self, eye_center, current_pupil_ellipse, current_pupil_confidence
    ):
        if self.sphere_radius_locked:
            self.max_observed_distance = self.locked_sphere_radius
            return

        if self.last_sphere_radius_ellipse is not None:
            anchored_distance = self._distance_to_pupil_outer_edge(
                eye_center, self.last_sphere_radius_ellipse
            )
            if anchored_distance is not None:
                self.max_observed_distance = anchored_distance

        if (
            current_pupil_ellipse is not None
            and current_pupil_confidence >= self.pupil_confidence_threshold_sphere
            and len(self.model_centers) >= self.min_model_centers
        ):
            current_distance = self._distance_to_pupil_outer_edge(
                eye_center, current_pupil_ellipse
            )
            if (
                current_distance is not None
                and (
                    self.last_sphere_radius_ellipse is None
                    or current_distance > self.max_observed_distance
                )
            ):
                self.max_observed_distance = current_distance
                self.last_sphere_radius_ellipse = current_pupil_ellipse

    # ==================== 射线交集与眼球中心估计 ====================

    @staticmethod
    def _angle_diff(a, b):
        diff = abs(a - b) % 180
        return min(diff, 180 - diff)

    @staticmethod
    def _find_line_intersection(ellipse1, ellipse2):
        (cx1, cy1), (_, minor_axis1), angle1 = ellipse1
        (cx2, cy2), (_, minor_axis2), angle2 = ellipse2

        angle1_rad = np.deg2rad(angle1)
        angle2_rad = np.deg2rad(angle2)

        dx1 = (minor_axis1 / 2) * np.cos(angle1_rad)
        dy1 = (minor_axis1 / 2) * np.sin(angle1_rad)
        dx2 = (minor_axis2 / 2) * np.cos(angle2_rad)
        dy2 = (minor_axis2 / 2) * np.sin(angle2_rad)

        A = np.array([[dx1, -dx2], [dy1, -dy2]])
        B = np.array([cx2 - cx1, cy2 - cy1])

        if np.linalg.det(A) == 0:
            return None

        t1, t2 = np.linalg.solve(A, B)
        intersection_x = cx1 + t1 * dx1
        intersection_y = cy1 + t1 * dy1

        return (int(intersection_x), int(intersection_y))

    def _compute_average_intersection(
        self, frame, ray_lines, number_lines, total_lines, minimum_angle_degrees
    ):
        pixel_limit = 30
        angle_threshold = 5

        if len(ray_lines) < 2 or number_lines < 2:
            return (0, 0)

        height, width = frame.shape[:2]
        selected_lines = random.sample(ray_lines, min(number_lines, len(ray_lines)))

        intersections = []
        for i in range(len(selected_lines) - 1):
            line1 = selected_lines[i]
            line2 = selected_lines[i + 1]

            if self._angle_diff(line1[2], line2[2]) >= minimum_angle_degrees:
                intersection = self._find_line_intersection(line1, line2)
                if (
                    intersection
                    and (0 <= intersection[0] < width)
                    and (0 <= intersection[1] < height)
                ):
                    intersections.append(intersection)

        if not intersections:
            return (0, 0)

        accept = True
        if len(intersections) >= 2:
            for i in range(len(intersections)):
                for j in range(i + 1, len(intersections)):
                    dx = intersections[i][0] - intersections[j][0]
                    dy = intersections[i][1] - intersections[j][1]
                    if (dx * dx + dy * dy) ** 0.5 > pixel_limit:
                        accept = False
                        break
                    angle_i = selected_lines[i][2]
                    angle_j = selected_lines[j][2]
                    if self._angle_diff(angle_i, angle_j) < angle_threshold:
                        accept = False
                        break
                if not accept:
                    break

        if accept:
            self.stored_intersections.extend(intersections)

        if len(self.stored_intersections) > total_lines:
            self.stored_intersections = self._prune_intersections(
                self.stored_intersections, total_lines
            )

        if not self.stored_intersections:
            return (0, 0)

        avg_x = np.mean([pt[0] for pt in self.stored_intersections])
        avg_y = np.mean([pt[1] for pt in self.stored_intersections])

        if np.isnan(avg_x) or np.isnan(avg_y):
            return (0, 0)

        return (int(avg_x), int(avg_y))

    @staticmethod
    def _prune_intersections(intersections, maximum_intersections):
        if len(intersections) <= maximum_intersections:
            return intersections
        return intersections[-maximum_intersections:]

    @staticmethod
    def _update_and_average_point(point_list, new_point, N):
        point_list.append(new_point)
        if len(point_list) > N:
            point_list.pop(0)
        if not point_list:
            return None
        avg_x = int(np.mean([p[0] for p in point_list]))
        avg_y = int(np.mean([p[1] for p in point_list]))
        return (avg_x, avg_y)

    # ==================== 状态重置 ====================

    def _reset_tracking_state(self):
        self.ray_lines = []
        self.model_centers = []
        self.prev_model_center_avg = (self.frame_width // 2, self.frame_height // 2)
        self.max_observed_distance = 0
        self.last_sphere_radius_ellipse = None
        self.stored_intersections = []
        self.last_tracking_result = None