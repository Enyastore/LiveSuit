#ifndef EYE_TRACKER_TYPES_H
#define EYE_TRACKER_TYPES_H

#include <string>
#include <vector>
#include <optional>
#include <unordered_map>
#include <opencv2/core.hpp>

namespace eye_tracker {

// Result from normalize()
struct NormalizeResult {
    std::optional<double> eye_x;
    std::optional<double> eye_y;
    std::vector<std::string> missing;
};

// Pupil ellipse data
struct PupilEllipse {
    std::optional<cv::Point2f> center;
    std::optional<cv::Point2f> axes;      // major, minor
    std::optional<float> angle_degrees;
};

// Frame tracking result
struct TrackingResult {
    std::optional<PupilEllipse> pupil_ellipse;
    cv::Point eye_center;
    double sphere_radius;
    double raw_eye_openness = 0.0;        // 眼睛开度原始竖直距离（像素）
};

// Result sent to Python via queue
struct GazeResult {
    std::string side;
    std::vector<double> gaze_rotated;  // 3-element
    std::optional<double> eye_x;
    std::optional<double> eye_y;
    double confidence;
    double raw_eye_openness = 0.0;        // 眼睛开度原始竖直距离
};

// 瞳孔检测调试结果（供 Python 调试面板直接使用）
struct PupilDebugResult {
    bool valid = false;
    cv::Mat binary;                  // inRange + 椭圆 mask 后的二值图（灰度）
    cv::Mat dilated;                 // 膨胀后的二值图（灰度）
    cv::Point2f darkest_point;       // 最暗点坐标
    int darkest_pixel_value = 0;     // 最暗点像素值
    bool ellipse_found = false;      // 是否成功拟合椭圆
    cv::Point2f ellipse_center;      // 椭圆中心
    cv::Point2f ellipse_axes;        // 椭圆轴 (width, height)
    float ellipse_angle = 0.0f;      // 椭圆角度
    double goodness_cover = 0.0;     // 覆盖率
    double goodness_aspect = 0.0;    // 纵横比
    double goodness_total = 0.0;     // 综合分 (cover × aspect × 100)
    double ratio_under_ellipse = 0.0;  // 轮廓贴合椭圆比例（正式算法置信度）
    float roi_cx = 0.0f, roi_cy = 0.0f;  // 搜索椭圆中心
    float roi_rx = 0.0f, roi_ry = 0.0f;  // 搜索椭圆半轴
};

// 眼睛开度检测调试结果（供 Python 调试面板直接使用）
struct OpennessDebugResult {
    bool valid = false;
    cv::Mat binary;                  // inRange 结果（灰度）
    cv::Mat masked;                  // 椭圆 mask 后（灰度）
    double top_agg = 0.0;            // 最高点聚合
    double bottom_agg = 0.0;         // 最低点聚合
    double raw_distance = 0.0;       // 开度
    float roi_cx = 0.0f, roi_cy = 0.0f;  // 搜索椭圆中心
    float roi_rx = 0.0f, roi_ry = 0.0f;  // 搜索椭圆半轴
};

// Command types from Python
enum class CommandType {
    LOCK_RADIUS,
    UNLOCK_RADIUS,
    LOCK_CENTER,
    UNLOCK_CENTER,
    HEADLESS_ON,
    HEADLESS_OFF,
    RELOAD_REFS,
    CLEAR_EXTREME_VECTORS,
    SAVE_EXTREME_VECTORS,
    SET_SEARCH_ROI_SCALE,
    RESTART_CAPTURE,
    SET_POSTPROCESS,
    SET_OPENNESS_THRESHOLD,
    SET_OPENNESS_BLUR,
    SET_OPENNESS_AGGREGATION,
    SET_OPENNESS_SKIP_THRESHOLD,
    SET_OPENNESS_THRESHOLD_LOW,    // 新增：开闭检测下界阈值
    SET_OPENNESS_THRESHOLD_HIGH,   // 新增：开闭检测上界阈值
    SET_PUPIL_THRESHOLD_LOW,       // 新增：瞳孔检测下界阈值
    SET_PUPIL_THRESHOLD_HIGH,      // 新增：瞳孔检测上界阈值
    UNKNOWN
};

struct Command {
    CommandType type = CommandType::UNKNOWN;
    std::vector<std::string> args;
};

// V4L2 fourcc codes
const std::unordered_map<std::string, int> V4L2_FOURCC_MAP = {
    {"YUYV", 0x56595559},
    {"MJPG", 0x47504A4D},
    {"NV12", 0x3231564E},
    {"H264", 0x34363248},
    {"BGR3", 0x33524742},
    {"RGB3", 0x33424752},
};

} // namespace eye_tracker

#endif // EYE_TRACKER_TYPES_H