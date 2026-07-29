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
};

// Result sent to Python via queue
struct GazeResult {
    std::string side;
    std::vector<double> gaze_rotated;  // 3-element
    std::optional<double> eye_x;
    std::optional<double> eye_y;
    double confidence;
};

// Command types from Python
enum class CommandType {
    LOCK_RADIUS,
    UNLOCK_RADIUS,
    LOCK_CENTER,
    UNLOCK_CENTER,
    HEADLESS_ON,
    HEADLESS_OFF,
    RELOAD_EXTREMES,
    CLEAR_EXTREMES,
    SAVE_EXTREME,
    SET_SEARCH_ROI_SCALE,
    RESTART_CAPTURE,
    SET_POSTPROCESS,
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