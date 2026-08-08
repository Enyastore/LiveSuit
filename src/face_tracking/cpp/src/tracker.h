#ifndef EYE_TRACKER_TRACKER_H
#define EYE_TRACKER_TRACKER_H

#include "types.h"
#include "normalizer.h"
#include <string>
#include <vector>
#include <optional>
#include <atomic>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <pybind11/pytypes.h>

namespace eye_tracker {

class GazeVectorTracker {
public:
    GazeVectorTracker(
        int cam_index = 0,
        bool flip = false,
        std::vector<int> crop = {0, 0, 640, 480},
        std::string side = "left",
        int frame_width = 640,
        int frame_height = 480,
        int frame_rate = 30,
        std::string refs_file = "references.yaml",
        bool use_recommended_resolution = true,
        double dark_search_roi_scale = 0.70,
        std::string fourcc_str = "",
        double brightness = 0.0,
        double contrast = 1.0,
        int openness_threshold_low = 0,
        int openness_threshold_high = 80,
        int pupil_threshold_low = 5,
        int pupil_threshold_high = 25
    );

    // 主循环（阻塞式，接受pybind11信息流）
    void start_tracking(
        pybind11::object py_cmd_queue,
        pybind11::object py_result_queue,
        bool headless = false
    );

    // 命令
    void lock_sphere_radius();
    void unlock_sphere_radius();
    void lock_eye_center();
    void unlock_eye_center();
    void stop();

    // get函数
    TrackingResult get_last_tracking_result() const { return last_tracking_result_; }
    bool is_running() const { return running_; }
    bool is_headless() const { return headless_; }

private:
    // ========== 配置 ==========
    int cam_index_;
    bool flip_;
    std::vector<int> crop_;
    std::string side_;
    int frame_width_;
    int frame_height_;
    int frame_rate_;
    std::string fourcc_str_;
    bool use_recommended_resolution_;
    double dark_search_roi_scale_;
    double brightness_;
    double contrast_;

    // ========== 归一化器 ==========
    Normalizer normalizer_;

    // ========== 追踪状态 ==========
    std::vector<cv::RotatedRect> ray_lines_;
    std::vector<cv::Point> model_centers_;
    int min_model_centers_ = 30;
    int max_rays_ = 100;
    cv::Point prev_model_center_avg_;
    double max_observed_distance_ = 0;
    std::optional<cv::RotatedRect> last_sphere_radius_ellipse_;
    double pupil_confidence_threshold_ = 0.85;
    double pupil_confidence_threshold_sphere_ = 0.65;
    int intersection_ray_count_ = 4;
    int minimum_intersection_angle_degrees_ = 8;
    TrackingResult last_tracking_result_;
    std::vector<cv::Point> stored_intersections_;

    // ========== 眼球锁定状态 ==========
    bool sphere_radius_locked_ = false;
    double locked_sphere_radius_ = 0;
    bool eye_center_locked_ = false;
    cv::Point locked_eye_center_;

    // ========== 运行状态 ==========
    cv::VideoCapture cap_;
    std::atomic<bool> running_{false};
    bool headless_ = false;
    std::string win_name_;
    std::optional<cv::RotatedRect> last_search_ellipse_;

    // ========== 眼睛开度检测参数 ==========
    int eye_openness_threshold_low_ = 0;      // 开闭检测二值化下界 (0-255)
    int eye_openness_threshold_high_ = 80;    // 开闭检测二值化上界 (0-255)
    int eye_openness_blur_ = 3;               // 高斯模糊核 (奇数, 1=不模糊)
    std::string eye_openness_aggregation_ = "median";  // "median"（中位数） 或 "average"（平均值）
    double raw_eye_openness_ = 0.0;           // 每帧计算的开度 raw 值（在瞳孔检测之前）
    double eye_openness_skip_threshold_ = 0.0;  // 低于此值跳过眼追（0=不跳过）
    double eye_openness_top_agg_ = 0.0;       // 每帧聚合后最高点 y 坐标
    double eye_openness_bottom_agg_ = 0.0;    // 每帧聚合后最低点 y 坐标

    // ========== 瞳孔检测参数（双阈值） ==========
    int pupil_threshold_low_ = 5;    // 瞳孔二值化下界偏移（相对于最暗像素）
    int pupil_threshold_high_ = 25;  // 瞳孔二值化上界偏移（相对于最暗像素）

    // ========== 内部方法 ==========

    void reset_tracking_state();
    void cleanup();
    void switch_headless_on();
    void switch_headless_off();

    // 帧处理
    void process_frame(const cv::Mat& frame);
    void process_frames(const cv::Mat& thresholded_strict,
                         const cv::Mat& thresholded_medium,
                         const cv::Mat& thresholded_relaxed,
                         cv::Mat& frame);

    // 阈值和遮罩
    static cv::Mat apply_binary_threshold(const cv::Mat& image, int darkest_pixel_value, int added_threshold);
    static cv::Mat mask_outside_square(const cv::Mat& image, cv::Point center, int size);

    // 搜索最暗区域
    std::optional<cv::Point> get_darkest_area(const cv::Mat& image, const cv::Mat& gray_frame);

    // 瞳孔轮廓处理
    static std::vector<cv::Point> filter_contours_by_area_and_return_largest(
        const std::vector<std::vector<cv::Point>>& contours, int pixel_thresh, int ratio_thresh);
    static std::vector<cv::Point> optimize_contours_by_angle(const std::vector<cv::Point>& contour);

    // 椭圆优度
    static std::vector<double> check_ellipse_goodness(const cv::Mat& binary_image,
                                                       const std::vector<cv::Point>& contour,
                                                       const std::optional<cv::RotatedRect>& ellipse_opt);
    static std::vector<double> check_contour_pixels(const std::vector<cv::Point>& contour,
                                                      cv::Size image_shape,
                                                      const std::optional<cv::RotatedRect>& ellipse_opt);

    // 眼球球半径和眼睛中心更新
    static std::optional<double> distance_to_pupil_outer_edge(cv::Point eye_center,
                                                               const cv::RotatedRect& pupil_ellipse);
    void update_eye_sphere_radius(cv::Point eye_center,
                                   const std::optional<cv::RotatedRect>& current_pupil_ellipse,
                                   double current_pupil_confidence);

    // 射线交点
    static double angle_diff(double a, double b);
    static std::optional<cv::Point> find_line_intersection(const cv::RotatedRect& e1,
                                                             const cv::RotatedRect& e2);
    cv::Point compute_average_intersection(const cv::Mat& frame,
                                            const std::vector<cv::RotatedRect>& ray_lines,
                                            int number_lines, int total_lines,
                                            int minimum_angle_degrees);
    static std::vector<cv::Point> prune_intersections(const std::vector<cv::Point>& intersections,
                                                        int maximum);
    static cv::Point update_and_average_point(std::vector<cv::Point>& point_list,
                                                cv::Point new_point, int N);

    // 计算注视向量
    std::tuple<std::optional<cv::Point3f>, std::optional<cv::Point3f>, NormalizeResult>
    compute_gaze_vector(int x, int y, int center_x, int center_y, double confidence_ratio);

    // 绘制调试层
    void draw_debug_overlay(cv::Mat& frame, cv::Point model_center_average,
                             const std::optional<cv::RotatedRect>& final_rotated_rect,
                             int center_x, int center_y,
                             const std::optional<cv::Point3f>& center_3d,
                             const std::optional<cv::Point3f>& gaze_rotated,
                             double best_ratio_under_ellipse,
                             const NormalizeResult& norm_result);

    // ========== 眼睛开度检测 ==========
    double compute_eye_openness(const cv::Mat& gray_frame);

    // ========== 命令/结果队列 ==========
    pybind11::object py_cmd_queue_;
    pybind11::object py_result_queue_;

    // ========== 命令处理 ==========
    void process_commands();
    void process_commands(pybind11::object py_cmd_queue);

    // ========== 摄像头重启 ==========
    void restart_camera(int w, int h, int fps, const std::string& fourcc_str);
};

} // namespace eye_tracker

#endif // EYE_TRACKER_TRACKER_H