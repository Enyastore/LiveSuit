"""眼球追踪核心算法模块 - C++ 加速版。

本模块是 C++ 核心库 eye_tracker_core_cpp 的薄封装层。
所有计算密集型算法已迁移至 C++，Python 仅负责：
  1) 导入 C++ 暴露的 Normalizer / GazeVectorTracker
  2) 提供与旧版 Python-only API 兼容的接口
  3) 保留必要的全局常量
"""

import logging
import os
from eye_tracker_core_cpp import (
    Normalizer as _CppNormalizer,
    GazeVectorTracker as _CppGazeVectorTracker,
    NormalizeResult,
    TrackingResult,
    PupilDebugResult,
    OpennessDebugResult,
)

logger = logging.getLogger(__name__)

# 本模块所在目录（用于解析 references.yaml 的相对路径，避免依赖当前工作目录）
_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))


def _resolve_refs_file(refs_file: str) -> str:
    """将相对路径的 refs_file 解析为模块目录下的绝对路径。

    若传入的是绝对路径，或模块目录下不存在该文件，则原样返回。
    """
    if os.path.isabs(refs_file):
        return refs_file
    resolved = os.path.join(_MODULE_DIR, refs_file)
    if os.path.exists(resolved):
        return resolved
    return refs_file  # 回退原值

# V4L2 四字符码整数值（直接设置 CAP_PROP_FOURCC 使用）
# 保留在此以便 eye_tracker_main 等模块引用
_V4L2_FOURCC_MAP = {
    "YUYV": 0x56595559,
    "MJPG": 0x47504A4D,
    "NV12": 0x3231564E,
    "H264": 0x34363248,
    "BGR3": 0x33524742,
    "RGB3": 0x33424752,
}


# ============================================================
# Normalizer — C++ 封装
# ============================================================

class Normalizer:
    """将 3D 注视向量（gaze_rotated）映射为屏幕坐标 [-1, 1]² 的归一化器。

    标定文件（references.yaml）需包含四个极值点：
      {side}_inner, {side}_outer, {side}_up, {side}_down

    无此文件时 normalize() 返回 eye_x / eye_y 均为 None，
    下游调用者须自行降级处理。

    此外支持眼睛开度标定参考值的存取（{side}_open / {side}_close）。
    """

    def __init__(self, refs_file: str = "references.yaml"):
        self._impl = _CppNormalizer(_resolve_refs_file(refs_file))

    def reload(self) -> None:
        self._impl.reload()

    def set_extreme_vectors(self, side: str, direction: str, vector) -> None:
        self._impl.set_extreme_vectors(side, direction, list(vector))

    def clear_extreme_vectors(self, side: str) -> None:
        self._impl.clear_extreme_vectors(side)

    def normalize(self, side: str, gaze_rotated) -> dict:
        result = self._impl.normalize(side, list(gaze_rotated))
        return {
            "eye_x": result.eye_x,
            "eye_y": result.eye_y,
            "missing": list(result.missing),
        }

    # ---- 眼睛开度标定 ----

    def set_openness_ref(self, side: str, ref_type: str, value: float) -> None:
        """保存眼睛开度参考距离到 YAML 文件。

        Parameters
        ----------
        side : str
            "left" 或 "right"
        ref_type : str
            "open"（全睁）或 "close"（全闭）
        value : float
            raw_eye_openness 的当前值
        """
        self._impl.set_openness_ref(side, ref_type, value)

    def get_openness_ref(self, side: str, ref_type: str):
        """读取眼睛开度参考距离。

        Returns
        -------
        float 或 None
        """
        return self._impl.get_openness_ref(side, ref_type)

    def clear_openness_ref(self, side: str) -> None:
        """清除指定眼睛的开闭参考距离（{side}_open / {side}_close）并持久化。

        Parameters
        ----------
        side : str
            "left" 或 "right"
        """
        self._impl.clear_openness_ref(side)
        logger.info(f"已清除 {side} 眼开闭参考距离")


# ============================================================
# GazeVectorTracker — C++ 封装
# ============================================================

class GazeVectorTracker:
    """瞳孔检测 → 视线方向计算 → 归一化 → 结果回传。

    所有算法运行在 C++ 侧，通过 multiprocessing.Queue 与主进程通信：
      - result_queue: 每帧产出 {side, gaze_rotated, eye_x, eye_y, confidence}
      - command_queue: 接收锁定/解锁定/headless 切换/极值重载命令

    可在 headless 模式下运行（无 X11 环境），不创建 OpenCV 窗口。
    """

    def __init__(
        self,
        cam_index: int = 0,
        flip: bool = False,
        crop=None,
        side: str = "left",
        frame_width: int = 640,
        frame_height: int = 480,
        frame_rate: int = 30,
        refs_file: str = "references.yaml",
        use_recommended_resolution: bool = True,
        dark_search_roi_scale: float = 0.70,
        fourcc_str: str = "",
        brightness: float = 0.0,
        contrast: float = 1.0,
        openness_threshold_low: int = 0,
        openness_threshold_high: int = 80,
        pupil_threshold_low: int = 0,
        pupil_threshold_high: int = 50,
    ):
        if crop is None:
            crop = [0, 0, frame_width, frame_height]
        self._impl = _CppGazeVectorTracker(
            cam_index, flip, list(crop), side,
            frame_width, frame_height, frame_rate,
            _resolve_refs_file(refs_file),
            use_recommended_resolution, dark_search_roi_scale,
            fourcc_str, brightness, contrast,
            openness_threshold_low, openness_threshold_high,
            pupil_threshold_low, pupil_threshold_high,
        )

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
        result_queue : multiprocessing.Queue | None
        headless : bool
        """
        self._impl.start_tracking(
            command_queue=command_queue,
            result_queue=result_queue,
            headless=headless,
        )

    def lock_sphere_radius(self):
        self._impl.lock_sphere_radius()

    def unlock_sphere_radius(self):
        self._impl.unlock_sphere_radius()

    def lock_eye_center(self):
        self._impl.lock_eye_center()

    def unlock_eye_center(self):
        self._impl.unlock_eye_center()

    def stop(self):
        self._impl.stop()

    def get_last_tracking_result(self):
        return self._impl.get_last_tracking_result()

    # ---- 静态调试接口（与 C++ 正式算法共用实现）----

    @staticmethod
    def debug_pupil_detect(frame, pupil_threshold_low, pupil_threshold_high,
                           dark_search_roi_scale, crop=None,
                           area_thresh=200, ratio_thresh=4):
        """瞳孔检测调试：返回 PupilDebugResult（二值图、最暗点、椭圆、优度）。

        参数 frame 为预处理后的 BGR 帧（flip / 亮度 / 对比度已应用）。
        crop 为可选 [x1, y1, x2, y2]，用于指定搜索椭圆位置；None 则用整帧中心。
        """
        return _CppGazeVectorTracker.debug_pupil_detect(
            frame, pupil_threshold_low, pupil_threshold_high,
            dark_search_roi_scale, list(crop or []), area_thresh, ratio_thresh)

    @staticmethod
    def debug_openness_detect(frame, openness_threshold_low, openness_threshold_high,
                              blur_kernel, aggregation, dark_search_roi_scale,
                              crop=None):
        """眼睛开度检测调试：返回 OpennessDebugResult（二值图、上下聚合线、开度）。

        aggregation 为 "median" 或 "average"。
        """
        return _CppGazeVectorTracker.debug_openness_detect(
            frame, openness_threshold_low, openness_threshold_high,
            blur_kernel, aggregation, dark_search_roi_scale, list(crop or []))