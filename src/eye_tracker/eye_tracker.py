import cv2
import random
import math
import numpy as np


class EyeTracker:
    """眼球追踪类：封装瞳孔检测与视线方向计算，实时输出gaze_ray到txt文件。"""

    def __init__(self, cam_index=0, flip=False, crop=None, side="left"):
        """
        Parameters
        ----------
        cam_index : int
            摄像机索引。
        flip : bool
            是否垂直翻转画面。
        crop : list | None
            剪裁区域 [x1, x2, y1, y2]，None 则默认 [0, 640, 0, 480]。
        side : str
            标识 "left" 或 "right"，影响输出文件名。
        """
        self.cam_index = cam_index
        self.flip = flip
        self.crop = crop if crop is not None else [0, 640, 0, 480]
        self.side = side

        # ---- 帧尺寸固定为 640x480，算法在该分辨率下效果最佳 ----
        self.frame_width = 640
        self.frame_height = 480

        # ---- 追踪状态变量（原全局变量） ----
        self.ray_lines = []                         # 近期瞳孔椭圆射线
        self.model_centers = []                     # 近期估计的眼球中心
        self.min_model_centers = 30                 # 最少眼球中心数量
        self.max_rays = 100                         # 最多存储的射线数
        self.prev_model_center_avg = (
            self.frame_width // 2,
            self.frame_height // 2,
        )                                           # 上一个有效眼球中心
        self.max_observed_distance = 0              # 自适应眼球半径
        self.last_sphere_radius_ellipse = None      # 上一个用于扩展半径的椭圆
        self.pupil_confidence_threshold = 0.85      # 存储射线的最低置信度
        self.pupil_confidence_threshold_sphere = 0.65  # 更新眼球半径的最低置信度
        self.intersection_ray_count = 4             # 每次交集估计采样的射线数
        self.minimum_intersection_angle_degrees = 8 # 采样射线间最小角度
        self.last_tracking_result = None            # 最新追踪结果
        self.stored_intersections = []              # 历史交集点

        # ---- 锁定状态 ----
        self.sphere_radius_locked = False
        self.locked_sphere_radius = 0
        self.eye_center_locked = False
        self.locked_eye_center = (self.frame_width // 2, self.frame_height // 2)

        # ---- 运行状态 ----
        self.cap = None
        self.running = False

    # ==================== 锁定函数 ====================

    def lock_sphere_radius(self):
        """锁定当前眼球半径（max_observed_distance），之后不再自适应更新。"""
        if self.max_observed_distance > 0:
            self.sphere_radius_locked = True
            self.locked_sphere_radius = self.max_observed_distance
            print(f"[{self.side}] 眼球半径已锁定: {self.locked_sphere_radius:.1f}")
        else:
            print(f"[{self.side}] 眼球半径仍为0，无法锁定，请先让追踪稳定。")

    def unlock_sphere_radius(self):
        """解锁眼球半径，恢复自适应更新。"""
        self.sphere_radius_locked = False
        print(f"[{self.side}] 眼球半径已解锁")

    def lock_eye_center(self):
        """锁定当前眼球中心，之后不再通过射线交集更新。"""
        if self.prev_model_center_avg[0] != self.frame_width // 2:
            self.eye_center_locked = True
            self.locked_eye_center = self.prev_model_center_avg
            print(f"[{self.side}] 眼球中心已锁定: {self.locked_eye_center}")
        else:
            print(f"[{self.side}] 眼球中心尚未稳定，无法锁定。")

    def unlock_eye_center(self):
        """解锁眼球中心，恢复射线交集更新。"""
        self.eye_center_locked = False
        print(f"[{self.side}] 眼球中心已解锁")

    # ==================== 核心处理流程 ====================

    def start_tracking(self, command_queue=None):
        """打开摄像机并进入主循环（阻塞）。关闭窗口可退出。
        
        Parameters
        ----------
        command_queue : multiprocessing.Queue | None
            接收来自主进程的锁定/解锁命令队列。
            支持的命令: 'lock_radius', 'unlock_radius', 'lock_center', 'unlock_center'
        """
        self._reset_tracking_state()

        # 优先使用 V4L2 后端（Linux），失败则回退默认
        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            self.cap = cv2.VideoCapture(self.cam_index, backend)
            if self.cap.isOpened():
                break

        if not self.cap.isOpened():
            print(f"错误：无法打开摄像机 (索引 {self.cam_index})")
            return

        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.running = True
        print(f"眼球追踪已启动 (cam={self.cam_index}, side={self.side})")

        win_name = f"Eye Tracker - {self.side.upper()} Eye"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.waitKey(1)  # 触发窗口系统初始化

        frame_fail_count = 0  # 连续失败计数
        max_fail = 10

        while self.running:
            # ---- 处理来自主进程的命令 ----
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
                    except Exception:
                        pass  # 队列已空或其他读取错误，忽略

            ret, frame = self.cap.read()
            if not ret:
                frame_fail_count += 1
                if frame_fail_count >= max_fail:
                    print(f"警告：连续 {max_fail} 次无法读取帧，退出")
                    break
                cv2.waitKey(50)  # 等待后重试
                continue
            frame_fail_count = 0  # 成功读取，重置计数

            # ---- 垂直翻转 ----
            if self.flip:
                frame = cv2.flip(frame, 0)

            # ---- 剪裁 ----
            x1, x2, y1, y2 = self.crop
            if y2 > frame.shape[0] or x2 > frame.shape[1]:
                print(f"警告：crop {self.crop} 超出帧尺寸 {frame.shape}")
                continue
            frame = frame[y1:y2, x1:x2]

            # ---- 核心处理 ----
            self._process_frame(frame)

            # ---- GUI 事件泵 ----
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                print("按下退出键，停止追踪。")
                break
            try:
                if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break

        self._cleanup()

    def stop(self):
        """停止追踪并释放资源。"""
        self.running = False
        self._cleanup()

    def _cleanup(self):
        """释放摄像头并销毁窗口。"""
        self.running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()

    def get_last_tracking_result(self):
        """返回最近一次的追踪结果字典。"""
        return self.last_tracking_result

    # ==================== 单帧处理 ====================

    def _process_frame(self, frame):
        """处理单帧图像。"""
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        darkest_point = self._get_darkest_area(frame)
        if darkest_point is None:
            self.last_tracking_result = None
            return
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        darkest_pixel_value = gray_frame[darkest_point[1], darkest_point[0]]

        # 三种阈值强度
        thresholded_strict = self._apply_binary_threshold(gray_frame, darkest_pixel_value, 5)
        thresholded_strict = self._mask_outside_square(thresholded_strict, darkest_point, 250)

        thresholded_medium = self._apply_binary_threshold(gray_frame, darkest_pixel_value, 15)
        thresholded_medium = self._mask_outside_square(thresholded_medium, darkest_point, 250)

        thresholded_relaxed = self._apply_binary_threshold(gray_frame, darkest_pixel_value, 25)
        thresholded_relaxed = self._mask_outside_square(thresholded_relaxed, darkest_point, 250)

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
        """对三种阈值图像分别检测瞳孔，选出最佳椭圆并计算视线。"""
        kernel = np.ones((5, 5), np.uint8)
        image_array = [thresholded_relaxed, thresholded_medium, thresholded_strict]

        final_rotated_rect = None
        final_contours = []
        goodness = 0
        best_ratio_under_ellipse = 0
        best_center_x, best_center_y = None, None

        for i in range(3):
            dilated = cv2.dilate(image_array[i], kernel, iterations=2)
            contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            reduced = self._filter_contours_by_area_and_return_largest(contours, 1000, 3)

            if len(reduced) > 0 and len(reduced[0]) > 5:
                current_goodness = self._check_ellipse_goodness(dilated, reduced[0])
                ellipse = cv2.fitEllipse(reduced[0])
                center_x, center_y = map(int, ellipse[0])

                total_pixels = self._check_contour_pixels(reduced[0], dilated.shape)

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

        # 角度优化
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

        # 计算眼球中心均值
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

        # 更新眼球半径
        self._update_eye_sphere_radius(
            model_center_average,
            final_rotated_rect,
            best_ratio_under_ellipse,
        )

        # 存储最新追踪结果
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

        # ============ 绘制 ============
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

        # 延长视线
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

        # 计算并显示视线向量
        center_3d, direction = self._compute_gaze_vector(
            center_x, center_y,
            model_center_average[0], model_center_average[1],
            best_ratio_under_ellipse,
        )

        if center_3d is not None and direction is not None:
            origin_text = f"Origin: ({center_3d[0]:.2f}, {center_3d[1]:.2f}, {center_3d[2]:.2f})"
            dir_text = f"Direction: ({direction[0]:.2f}, {direction[1]:.2f}, {direction[2]:.2f})"

            text_origin_shadow = (12, frame.shape[0] - 38)
            text_dir_shadow = (12, frame.shape[0] - 13)
            text_origin = (10, frame.shape[0] - 40)
            text_dir = (10, frame.shape[0] - 15)

            cv2.putText(frame, origin_text, text_origin_shadow, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(frame, dir_text, text_dir_shadow, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(frame, origin_text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, dir_text, text_dir, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 置信度
        ratio_text = f"{best_ratio_under_ellipse * 100:.2f}%"
        cv2.putText(frame, ratio_text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(frame, ratio_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 最终结果窗口
        cv2.imshow(f"Eye Tracker - {self.side.upper()} Eye", frame)

    # ==================== 视线向量计算与文件输出 ====================

    def _compute_gaze_vector(self, x, y, center_x, center_y, confidence_ratio=0.0):
        """根据瞳孔屏幕坐标和眼球中心计算 3D 视线方向，并写入文件。"""
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
                [t_rot * x_a * x_a + c_rot, t_rot * x_a * y_a - s_rot * z_a, t_rot * x_a * z_a + s_rot * y_a],
                [t_rot * x_a * y_a + s_rot * z_a, t_rot * y_a * y_a + c_rot, t_rot * y_a * z_a - s_rot * x_a],
                [t_rot * x_a * z_a - s_rot * y_a, t_rot * y_a * z_a + s_rot * x_a, t_rot * z_a * z_a + c_rot],
            ])

            gaze_local = np.array([0.0, 0.0, inner_radius])
            gaze_rotated = rotation_matrix @ gaze_local
            gaze_rotated /= np.linalg.norm(gaze_rotated)

        # ---- 写入文件（仅当置信度高于 75% 时写入） ----
        if confidence_ratio < 0.75:
            return sphere_center, gaze_rotated

        file_path = f"gaze_vector_{self.side}.txt"

        def is_file_available(path):
            try:
                with open(path, "a"):
                    return True
            except IOError:
                return False

        if is_file_available(file_path):
            try:
                with open(file_path, "w") as f:
                    all_values = np.concatenate((sphere_center, gaze_rotated))
                    csv_line = ",".join(f"{v:.6f}" for v in all_values)
                    f.write(csv_line + "\n")
            except Exception as e:
                print("Write error:", e)
        else:
            print("File is currently in use. Skipping write.")

        return sphere_center, gaze_rotated

    # ==================== 阈值与遮罩 ====================

    @staticmethod
    def _apply_binary_threshold(image, darkest_pixel_value, added_threshold):
        """对图像做二值化逆阈值处理。"""
        threshold = darkest_pixel_value + added_threshold
        _, thresholded = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
        return thresholded

    @staticmethod
    def _get_darkest_area(image):
        """寻找图像中最暗的方形区域中心。"""
        ignore_bounds = 20
        image_skip_size = 10
        search_area = 20
        internal_skip_size = 5

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        min_sum = float("inf")
        darkest_point = None

        for y in range(ignore_bounds, gray.shape[0] - ignore_bounds, image_skip_size):
            for x in range(ignore_bounds, gray.shape[1] - ignore_bounds, image_skip_size):
                current_sum = 0
                num_pixels = 0
                for dy in range(0, search_area, internal_skip_size):
                    if y + dy >= gray.shape[0]:
                        break
                    for dx in range(0, search_area, internal_skip_size):
                        if x + dx >= gray.shape[1]:
                            break
                        current_sum += int(gray[y + dy][x + dx])
                        num_pixels += 1

                if current_sum < min_sum and num_pixels > 0:
                    min_sum = current_sum
                    darkest_point = (x + search_area // 2, y + search_area // 2)

        return darkest_point

    @staticmethod
    def _mask_outside_square(image, center, size):
        """保留以 center 为中心、size 为边长的方形区域，其余置零。"""
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
    def _filter_contours_by_area_and_return_largest(contours, pixel_thresh, ratio_thresh):
        """返回面积≥pixel_thresh且长宽比≤ratio_thresh的最大轮廓。"""
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
        """根据点与质心的角度过滤轮廓点。"""
        if len(contours) < 1:
            return contours

        all_contours = np.concatenate(contours[0], axis=0)
        spacing = int(len(all_contours) / 25)
        filtered_points = []
        centroid = np.mean(all_contours, axis=0)

        for i in range(len(all_contours)):
            current_point = all_contours[i]
            prev_point = all_contours[i - spacing] if i - spacing >= 0 else all_contours[-spacing]
            next_point = all_contours[i + spacing] if i + spacing < len(all_contours) else all_contours[spacing]

            vec1 = prev_point - current_point
            vec2 = next_point - current_point
            vec_to_centroid = centroid - current_point

            # 归一化所有向量，使点积仅反映方向相似度
            n1 = np.linalg.norm(vec_to_centroid)
            n2 = np.linalg.norm(vec1 + vec2)
            if n1 < 1e-6 or n2 < 1e-6:
                continue
            v_dir = vec_to_centroid / n1
            v_tangent = (vec1 + vec2) / n2

            cos_threshold = np.cos(np.radians(60))
            if np.dot(v_dir, v_tangent) >= cos_threshold:
                filtered_points.append(current_point)

        return np.array(filtered_points, dtype=np.int32).reshape((-1, 1, 2))

    # ==================== 椭圆质量检测 ====================

    @staticmethod
    def _check_ellipse_goodness(binary_image, contour):
        """评估椭圆与二值图像的重合度。返回 [覆盖率, ?, 偏心率]。"""
        if len(contour) < 5:
            return [0, 0, 0]

        ellipse = cv2.fitEllipse(contour)
        mask = np.zeros_like(binary_image)
        cv2.ellipse(mask, ellipse, (255), -1)

        ellipse_area = np.sum(mask == 255)
        if ellipse_area == 0:
            return [0, 0, 0]

        covered_pixels = np.sum((binary_image == 255) & (mask == 255))
        goodness = [0, 0, 0]
        goodness[0] = covered_pixels / ellipse_area
        goodness[2] = min(ellipse[1][1] / ellipse[1][0], ellipse[1][0] / ellipse[1][1])

        return goodness

    @staticmethod
    def _check_contour_pixels(contour, image_shape):
        """统计轮廓落在拟合椭圆内的像素数及比例。"""
        if len(contour) < 5:
            return [0, 0]

        contour_mask = np.zeros(image_shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, (255), 1)

        ellipse_mask_thick = np.zeros(image_shape, dtype=np.uint8)
        ellipse_mask_thin = np.zeros(image_shape, dtype=np.uint8)
        ellipse = cv2.fitEllipse(contour)

        cv2.ellipse(ellipse_mask_thick, ellipse, (255), 10)
        cv2.ellipse(ellipse_mask_thin, ellipse, (255), 4)

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
        """计算眼球中心到瞳孔椭圆远侧边缘的距离。"""
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

    def _update_eye_sphere_radius(self, eye_center, current_pupil_ellipse, current_pupil_confidence):
        """自适应更新眼球半径（如果未锁定）。"""
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
        """返回 [0, 180) 范围内两个角度的最小夹角差。"""
        diff = abs(a - b) % 180
        return min(diff, 180 - diff)

    @staticmethod
    def _find_line_intersection(ellipse1, ellipse2):
        """计算两条椭圆短轴方向直线的交点。"""
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

    def _compute_average_intersection(self, frame, ray_lines, number_lines, total_lines, minimum_angle_degrees):
        """从射线中采样、求交集，并返回滑动平均后的交点。"""
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
        """只保留最后 maximum_intersections 个交集点。"""
        if len(intersections) <= maximum_intersections:
            return intersections
        return intersections[-maximum_intersections:]

    @staticmethod
    def _update_and_average_point(point_list, new_point, N):
        """向列表添加新点并返回最近 N 个点的均值。"""
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
        """重置所有追踪状态变量。"""
        self.ray_lines = []
        self.model_centers = []
        self.prev_model_center_avg = (self.frame_width // 2, self.frame_height // 2)
        self.max_observed_distance = 0
        self.last_sphere_radius_ellipse = None
        self.stored_intersections = []
        self.last_tracking_result = None