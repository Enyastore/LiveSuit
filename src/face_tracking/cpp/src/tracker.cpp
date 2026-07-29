#include "tracker.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>
#include <thread>
#include <pybind11/pybind11.h>

namespace eye_tracker {
namespace py = pybind11;

// ================================================================
// Constructor
// ================================================================

GazeVectorTracker::GazeVectorTracker(
    int cam_index, bool flip, std::vector<int> crop,
    std::string side, int frame_width, int frame_height,
    int frame_rate, std::string extreme_file,
    bool use_recommended_resolution, double dark_search_roi_scale,
    std::string fourcc_str, double brightness, double contrast)
    : cam_index_(cam_index), flip_(flip), crop_(std::move(crop)),
      side_(std::move(side)), frame_width_(frame_width),
      frame_height_(frame_height), frame_rate_(frame_rate),
      fourcc_str_(std::move(fourcc_str)),
      use_recommended_resolution_(use_recommended_resolution),
      dark_search_roi_scale_(std::max(0.1, std::min(1.0, dark_search_roi_scale))),
      brightness_(brightness), contrast_(contrast),
      normalizer_(std::move(extreme_file)),
      prev_model_center_avg_(frame_width / 2, frame_height / 2) {}

// ================================================================
// Lock / Unlock
// ================================================================

void GazeVectorTracker::lock_sphere_radius() {
    if (max_observed_distance_ > 0) {
        sphere_radius_locked_ = true;
        locked_sphere_radius_ = max_observed_distance_;
        std::cerr << "[" << side_ << "] 眼球半径已锁定: " << locked_sphere_radius_ << std::endl;
    } else {
        std::cerr << "[" << side_ << "] 眼球半径仍为0，无法锁定" << std::endl;
    }
}

void GazeVectorTracker::unlock_sphere_radius() {
    sphere_radius_locked_ = false;
    std::cerr << "[" << side_ << "] 眼球半径已解锁" << std::endl;
}

void GazeVectorTracker::lock_eye_center() {
    if (prev_model_center_avg_.x != frame_width_ / 2) {
        eye_center_locked_ = true;
        locked_eye_center_ = prev_model_center_avg_;
        std::cerr << "[" << side_ << "] 眼球中心已锁定: (" << locked_eye_center_.x
                  << ", " << locked_eye_center_.y << ")" << std::endl;
    } else {
        std::cerr << "[" << side_ << "] 眼球中心尚未稳定，无法锁定" << std::endl;
    }
}

void GazeVectorTracker::unlock_eye_center() {
    eye_center_locked_ = false;
    std::cerr << "[" << side_ << "] 眼球中心已解锁" << std::endl;
}

void GazeVectorTracker::stop() {
    running_ = false;
    cleanup();
}

// ================================================================
// State management
// ================================================================

void GazeVectorTracker::reset_tracking_state() {
    ray_lines_.clear();
    model_centers_.clear();
    prev_model_center_avg_ = cv::Point(frame_width_ / 2, frame_height_ / 2);
    max_observed_distance_ = 0;
    last_sphere_radius_ellipse_.reset();
    stored_intersections_.clear();
    last_tracking_result_ = TrackingResult{};
}

void GazeVectorTracker::cleanup() {
    running_ = false;
    if (cap_.isOpened()) {
        cap_.release();
    }
    if (!headless_) {
        try {
            cv::destroyAllWindows();
        } catch (...) {}
    }
}

void GazeVectorTracker::switch_headless_on() {
    if (headless_) return;
    try {
        cv::destroyAllWindows();
        for (int i = 0; i < 20; ++i) {
            if (cv::waitKey(1) < 0) break;
        }
    } catch (...) {}
    headless_ = true;
    std::cerr << "[" << side_ << "] 已切换到 headless 模式" << std::endl;
}

void GazeVectorTracker::switch_headless_off() {
    if (!headless_) return;
    headless_ = false;
    std::cerr << "[" << side_ << "] 已退出 headless 模式" << std::endl;
    try {
        if (!win_name_.empty()) {
            cv::namedWindow(win_name_, cv::WINDOW_NORMAL);
            cv::waitKey(1);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[" << side_ << "] 无法重建 OpenCV 窗口: " << e.what() << std::endl;
        headless_ = true;
    }
}

// ================================================================
// Camera restart
// ================================================================

void GazeVectorTracker::restart_camera(int w, int h, int fps, const std::string& fourcc_str) {
    std::cerr << "[" << side_ << "] 重启相机: " << w << "x" << h << " @" << fps << "fps"
              << " fourcc=" << fourcc_str << std::endl;

    cv::VideoCapture old_cap = std::move(cap_);
    cap_ = cv::VideoCapture();

    bool opened = false;
    for (int backend : {cv::CAP_V4L2, cv::CAP_ANY}) {
        cap_.open(cam_index_, backend);
        if (cap_.isOpened()) {
            opened = true;
            break;
        }
    }

    if (!opened) {
        std::cerr << "[" << side_ << "] 重启相机失败，保留旧相机" << std::endl;
        cap_ = std::move(old_cap);
        return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap_.set(cv::CAP_PROP_FPS, fps);

    auto it = V4L2_FOURCC_MAP.find(fourcc_str);
    if (it != V4L2_FOURCC_MAP.end()) {
        cap_.set(cv::CAP_PROP_FOURCC, it->second);
    }

    // Verify actual size
    int actual_w = (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_h = (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (actual_w != w || actual_h != h) {
        std::cerr << "[" << side_ << "] 期望 " << w << "x" << h
                  << "，实际 " << actual_w << "x" << actual_h << std::endl;
    }

    // Scale crop based on old resolution
    if (frame_width_ > 0 && frame_height_ > 0) {
        crop_[0] = crop_[0] * w / frame_width_;
        crop_[1] = crop_[1] * h / frame_height_;
        crop_[2] = crop_[2] * w / frame_width_;
        crop_[3] = crop_[3] * h / frame_height_;
    }

    frame_width_ = w;
    frame_height_ = h;
    frame_rate_ = fps;
    fourcc_str_ = fourcc_str;
    reset_tracking_state();

    std::cerr << "[" << side_ << "] 相机重启完成: " << w << "x" << h << " @" << fps << "fps" << std::endl;
}

// ================================================================
// Command processing
// ================================================================

void GazeVectorTracker::process_commands() {
    process_commands(py_cmd_queue_);
}

void GazeVectorTracker::process_commands(pybind11::object py_cmd_queue) {
    if (py_cmd_queue.is_none()) return;

    try {
        while (true) {
            // Call get_nowait() on the Python queue
            py::object cmd = py_cmd_queue.attr("get_nowait")();
            
            if (cmd.is_none()) continue;

            // Check if it's a string command
            if (py::isinstance<py::str>(cmd)) {
                std::string cmd_str = cmd.cast<std::string>();

                if (cmd_str == "lock_radius") lock_sphere_radius();
                else if (cmd_str == "unlock_radius") unlock_sphere_radius();
                else if (cmd_str == "lock_center") lock_eye_center();
                else if (cmd_str == "unlock_center") unlock_eye_center();
                else if (cmd_str == "headless_on") switch_headless_on();
                else if (cmd_str == "headless_off") switch_headless_off();
                else if (cmd_str == "reload_extremes") normalizer_.reload();
                else if (cmd_str == "clear_extremes") normalizer_.clear_extremes(side_);
                else if (cmd_str == "set_openness_threshold") {
                    // Will be handled by next iteration if set via tuple
                }
                else if (cmd_str == "set_openness_blur") {
                    // Will be handled by next iteration if set via tuple
                }
                else if (cmd_str == "set_openness_aggregation") {
                    // Will be handled by next iteration if set via tuple
                }
            }
            // Check if it's a tuple command (like ("save_extreme", direction, vector))
            else if (py::isinstance<py::tuple>(cmd)) {
                py::tuple tup = cmd.cast<py::tuple>();
                if (tup.size() == 0) continue;

                std::string cmd_type = tup[0].cast<std::string>();

                if (cmd_type == "save_extreme" && tup.size() >= 3) {
                    std::string direction = tup[1].cast<std::string>();
                    py::list vec_list = tup[2].cast<py::list>();
                    std::vector<double> vec;
                    for (auto item : vec_list) {
                        vec.push_back(item.cast<double>());
                    }
                    normalizer_.set_extreme(side_, direction, vec);
                }
                else if (cmd_type == "set_search_roi_scale" && tup.size() >= 2) {
                    dark_search_roi_scale_ = std::max(0.1, std::min(1.0, tup[1].cast<double>()));
                }
                else if (cmd_type == "restart_capture" && tup.size() >= 5) {
                    restart_camera(tup[1].cast<int>(), tup[2].cast<int>(),
                                   tup[3].cast<int>(), tup[4].cast<std::string>());
                }
                else if (cmd_type == "set_postprocess" && tup.size() >= 3) {
                    brightness_ = tup[1].cast<double>();
                    contrast_ = tup[2].cast<double>();
                }
                else if (cmd_type == "set_openness_threshold" && tup.size() >= 2) {
                    eye_openness_threshold_ = std::max(0, std::min(255, tup[1].cast<int>()));
                }
                else if (cmd_type == "set_openness_blur" && tup.size() >= 2) {
                    int v = tup[1].cast<int>();
                    // Force odd number >= 1
                    if (v < 1) v = 1;
                    if (v % 2 == 0) v += 1;
                    eye_openness_blur_ = v;
                }
                else if (cmd_type == "set_openness_aggregation" && tup.size() >= 2) {
                    std::string agg = tup[1].cast<std::string>();
                    if (agg == "median" || agg == "average") {
                        eye_openness_aggregation_ = agg;
                    }
                }
                else if (cmd_type == "set_openness_skip_threshold" && tup.size() >= 2) {
                    eye_openness_skip_threshold_ = std::max(0.0, tup[1].cast<double>());
                    std::cerr << "[" << side_ << "] 低开度跳过阈值: " << eye_openness_skip_threshold_ << std::endl;
                }
            }
        }
    } catch (const py::error_already_set& e) {
        // queue.Empty is expected when no commands; ignore
        if (!e.matches(PyExc_Exception)) throw;
    }
}

// ================================================================
// Main tracking loop
// ================================================================

void GazeVectorTracker::start_tracking(
    py::object py_cmd_queue, py::object py_result_queue, bool headless)
{
    headless_ = headless;
    reset_tracking_state();

    // Open camera
    for (int backend : {cv::CAP_V4L2, cv::CAP_ANY}) {
        cap_.open(cam_index_, backend);
        if (cap_.isOpened()) break;
    }

    if (!cap_.isOpened()) {
        std::cerr << "错误：无法打开摄像机 (索引 " << cam_index_ << ")" << std::endl;
        return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS, frame_rate_);

    auto it = V4L2_FOURCC_MAP.find(fourcc_str_);
    if (it != V4L2_FOURCC_MAP.end()) {
        cap_.set(cv::CAP_PROP_FOURCC, it->second);
    }

    // Store queues as member objects
    py_cmd_queue_ = py_cmd_queue;
    py_result_queue_ = py_result_queue;

    running_ = true;
    std::cerr << "眼球追踪已启动 (cam=" << cam_index_ << ", side=" << side_
              << ", headless=" << headless_ << ")" << std::endl;

    win_name_ = "Eye Tracker - " + std::string(1, toupper(side_[0])) + side_.substr(1) + " Eye";
    if (!headless_) {
        cv::namedWindow(win_name_, cv::WINDOW_NORMAL);
        cv::waitKey(1);
    }

    int frame_fail_count = 0;
    const int max_fail = 10;

    while (running_) {
        // Process commands
        process_commands();

        cv::Mat frame;
        bool ret = cap_.read(frame);
        if (!ret) {
            frame_fail_count++;
            if (frame_fail_count >= max_fail) {
                std::cerr << "警告：连续 " << max_fail << " 次无法读取帧，退出" << std::endl;
                break;
            }
            if (headless_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } else {
                cv::waitKey(50);
            }
            continue;
        }
        frame_fail_count = 0;

        if (flip_) {
            cv::flip(frame, frame, 0);
        }

        int x1 = crop_[0], y1 = crop_[1], x2 = crop_[2], y2 = crop_[3];
        if (y2 > frame.rows || x2 > frame.cols) {
            std::cerr << "警告：crop [" << x1 << "," << y1 << "," << x2 << "," << y2
                      << "] 超出帧尺寸 (" << frame.cols << "x" << frame.rows << ")" << std::endl;
            continue;
        }
        frame = frame(cv::Rect(x1, y1, x2 - x1, y2 - y1));

        process_frame(frame);

        if (!headless_) {
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q') {
                std::cerr << "按下退出键，停止追踪。" << std::endl;
                break;
            }
            try {
                if (cv::getWindowProperty(win_name_, cv::WND_PROP_VISIBLE) < 1) {
                    std::cerr << "[" << side_ << "] OpenCV 窗口丢失，自动切换为 headless 模式" << std::endl;
                    switch_headless_on();
                }
            } catch (const cv::Exception&) {
                std::cerr << "[" << side_ << "] X11 异常，自动切换为 headless 模式" << std::endl;
                switch_headless_on();
            }
        }
    }

    cleanup();
}

// ================================================================
// Frame processing
// ================================================================

void GazeVectorTracker::process_frame(const cv::Mat& frame) {
    cv::Mat processed_frame;

    if (use_recommended_resolution_) {
        cv::resize(frame, processed_frame, cv::Size(frame_width_, frame_height_));
    } else {
        int h = frame.rows, w = frame.cols;
        int new_w, new_h;
        if (h > w) {
            new_w = 480;
            new_h = (int)(480.0 * h / w);
        } else {
            new_w = 640;
            new_h = (int)(640.0 * h / w);
        }
        cv::resize(frame, processed_frame, cv::Size(new_w, new_h));
        frame_width_ = new_w;
        frame_height_ = new_h;
    }

    // Brightness/contrast adjustment
    if (contrast_ != 1.0 || brightness_ != 0.0) {
        processed_frame.convertTo(processed_frame, -1, contrast_, brightness_);
    }

    cv::Mat gray_frame;
    cv::cvtColor(processed_frame, gray_frame, cv::COLOR_BGR2GRAY);

    // 眼睛开度检测 — 在瞳孔追踪之前执行（轻量级）
    raw_eye_openness_ = compute_eye_openness(gray_frame);

    // 开度低于阈值 → 跳过整个眼追流水线，节省 CPU
    if (eye_openness_skip_threshold_ > 0.0 && raw_eye_openness_ < eye_openness_skip_threshold_) {
        last_tracking_result_ = TrackingResult{};
        last_tracking_result_.raw_eye_openness = raw_eye_openness_;
        return;
    }

    auto darkest_point = get_darkest_area(processed_frame, gray_frame);
    if (!darkest_point.has_value()) {
        last_tracking_result_ = TrackingResult{};
        return;
    }

    int darkest_pixel_value = gray_frame.at<uchar>(darkest_point->y, darkest_point->x);

    cv::Mat thresholded_strict = apply_binary_threshold(gray_frame, darkest_pixel_value, 5);
    thresholded_strict = mask_outside_square(thresholded_strict, *darkest_point, 250);

    cv::Mat thresholded_medium = apply_binary_threshold(gray_frame, darkest_pixel_value, 15);
    thresholded_medium = mask_outside_square(thresholded_medium, *darkest_point, 250);

    cv::Mat thresholded_relaxed = apply_binary_threshold(gray_frame, darkest_pixel_value, 25);
    thresholded_relaxed = mask_outside_square(thresholded_relaxed, *darkest_point, 250);

    process_frames(thresholded_strict, thresholded_medium, thresholded_relaxed, processed_frame);
}

// ================================================================
// Multi-threshold fusion & ellipse fitting
// ================================================================

void GazeVectorTracker::process_frames(
    const cv::Mat& thresholded_strict, const cv::Mat& thresholded_medium,
    const cv::Mat& thresholded_relaxed, cv::Mat& frame)
{
    cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
    std::vector<cv::Mat> image_array = {thresholded_relaxed, thresholded_medium, thresholded_strict};

    std::optional<cv::RotatedRect> final_rotated_rect;
    std::vector<cv::Point> final_contours;
    double goodness = 0;
    double best_ratio_under_ellipse = 0;
    int best_center_x = -1, best_center_y = -1;

    for (int i = 0; i < 3; ++i) {
        cv::Mat dilated;
        cv::dilate(image_array[i], dilated, kernel, cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        auto reduced = filter_contours_by_area_and_return_largest(contours, 1000, 3);

        if (reduced.size() > 5) {
            cv::RotatedRect ellipse = cv::fitEllipse(reduced);
            int cx = (int)ellipse.center.x;
            int cy = (int)ellipse.center.y;

            auto current_goodness = check_ellipse_goodness(dilated, reduced, ellipse);
            auto total_pixels = check_contour_pixels(reduced, dilated.size(), ellipse);

            double final_goodness = current_goodness[0] * total_pixels[0] * total_pixels[0] * total_pixels[1];

            if (final_goodness > 0 && final_goodness > goodness) {
                goodness = final_goodness;
                best_ratio_under_ellipse = total_pixels[1];
                final_contours = reduced;
                best_center_x = cx;
                best_center_y = cy;
            }
        }
    }

    // Optimize contours by angle
    if (!final_contours.empty()) {
        final_contours = {optimize_contours_by_angle(final_contours)};
    }

    if (final_contours.size() > 5) {
        final_rotated_rect = cv::fitEllipse(final_contours);

        if (best_ratio_under_ellipse >= pupil_confidence_threshold_) {
            ray_lines_.push_back(*final_rotated_rect);
            if ((int)ray_lines_.size() > max_rays_) {
                ray_lines_.erase(ray_lines_.begin(), ray_lines_.begin() + (ray_lines_.size() - max_rays_));
            }
        }
    }

    int center_x = best_center_x;
    int center_y = best_center_y;

    cv::Point model_center_average;
    if (eye_center_locked_) {
        model_center_average = locked_eye_center_;
    } else {
        model_center_average = cv::Point(frame_width_ / 2, frame_height_ / 2);
        cv::Point model_center = compute_average_intersection(
            frame, ray_lines_, intersection_ray_count_, 1500, minimum_intersection_angle_degrees_);

        if (model_center.x != 0 || model_center.y != 0) {
            model_center_average = update_and_average_point(model_centers_, model_center, 200);
        }

        if (model_center_average.x == frame_width_ / 2) {
            model_center_average = prev_model_center_avg_;
        }
        if (model_center_average.x != 0) {
            prev_model_center_avg_ = model_center_average;
        }
    }

    if (center_x == -1 || center_y == -1) {
        last_tracking_result_ = TrackingResult{};
        return;
    }

    update_eye_sphere_radius(model_center_average, final_rotated_rect, best_ratio_under_ellipse);

    // Build TrackingResult
    last_tracking_result_ = TrackingResult{};
    if (final_rotated_rect.has_value()) {
        PupilEllipse pe;
        pe.center = cv::Point2f(final_rotated_rect->center.x, final_rotated_rect->center.y);
        pe.axes = cv::Point2f(final_rotated_rect->size.width, final_rotated_rect->size.height);
        pe.angle_degrees = final_rotated_rect->angle;
        last_tracking_result_.pupil_ellipse = pe;
    }
    last_tracking_result_.eye_center = model_center_average;
    last_tracking_result_.sphere_radius = max_observed_distance_;
    last_tracking_result_.raw_eye_openness = raw_eye_openness_;

    // Compute gaze vector (uses member py_result_queue_ to send result back)
    auto [center_3d, gaze_rotated, norm_result] = compute_gaze_vector(
        center_x, center_y, model_center_average.x, model_center_average.y,
        best_ratio_under_ellipse);

    if (!headless_) {
        draw_debug_overlay(frame, model_center_average, final_rotated_rect,
                           center_x, center_y, center_3d, gaze_rotated,
                           best_ratio_under_ellipse, norm_result);
    }
}

// ================================================================
// Threshold & mask
// ================================================================

cv::Mat GazeVectorTracker::apply_binary_threshold(
    const cv::Mat& image, int darkest_pixel_value, int added_threshold)
{
    int threshold = darkest_pixel_value + added_threshold;
    cv::Mat thresholded;
    cv::threshold(image, thresholded, threshold, 255, cv::THRESH_BINARY_INV);
    return thresholded;
}

cv::Mat GazeVectorTracker::mask_outside_square(
    const cv::Mat& image, cv::Point center, int size)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);
    int half = size / 2;
    int x1 = std::max(0, center.x - half);
    int y1 = std::max(0, center.y - half);
    int x2 = std::min(image.cols, center.x + half);
    int y2 = std::min(image.rows, center.y + half);
    mask(cv::Rect(x1, y1, x2 - x1, y2 - y1)).setTo(255);
    cv::Mat result;
    cv::bitwise_and(image, mask, result);
    return result;
}

// ================================================================
// Darkest area search (vectorized)
// ================================================================

std::optional<cv::Point> GazeVectorTracker::get_darkest_area(
    const cv::Mat& image, const cv::Mat& gray_frame)
{
    int h = image.rows, w = image.cols;
    int cx_roi = w / 2, cy_roi = h / 2;
    int rx = (int)((w / 2.0) * dark_search_roi_scale_);
    int ry = (int)((h / 2.0) * dark_search_roi_scale_);

    if (rx > 0 && ry > 0) {
        last_search_ellipse_ = cv::RotatedRect(cv::Point2f(cx_roi, cy_roi),
                                                cv::Size2f(rx * 2, ry * 2), 0);
    } else {
        last_search_ellipse_.reset();
    }

    const int ignore_bounds = 20;
    const int image_skip_size = 10;
    const int search_area = 20;
    const int internal_skip_size = 5;

    int min_sum = INT_MAX;
    std::optional<cv::Point> best_point;

    // C++ vectorized scan - iterate over grid points
    for (int gy = ignore_bounds; gy < h - ignore_bounds; gy += image_skip_size) {
        for (int gx = ignore_bounds; gx < w - ignore_bounds; gx += image_skip_size) {
            int sy = gy + search_area / 2;
            int sx = gx + search_area / 2;

            // Check if within ellipse
            if (rx > 0 && ry > 0) {
                double dx = (double)(sx - cx_roi) / rx;
                double dy = (double)(sy - cy_roi) / ry;
                if (dx * dx + dy * dy > 1.0) continue;
            }

            // Sum up a grid of (internal_skip_size × internal_skip_size) pixels
            int sum = 0;
            int count = 0;
            for (int dy = 0; dy < search_area; dy += internal_skip_size) {
                for (int dx = 0; dx < search_area; dx += internal_skip_size) {
                    int py = std::min(std::max(sy + dy, 0), h - 1);
                    int px = std::min(std::max(sx + dx, 0), w - 1);
                    sum += gray_frame.at<uchar>(py, px);
                    count++;
                }
            }

            if (count > 0 && sum < min_sum) {
                min_sum = sum;
                best_point = cv::Point(sx, sy);
            }
        }
    }

    return best_point;
}

// ================================================================
// Contour processing
// ================================================================

std::vector<cv::Point> GazeVectorTracker::filter_contours_by_area_and_return_largest(
    const std::vector<std::vector<cv::Point>>& contours, int pixel_thresh, int ratio_thresh)
{
    double max_area = 0;
    int best_idx = -1;

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area >= pixel_thresh) {
            cv::Rect rect = cv::boundingRect(contours[i]);
            double ratio = std::max((double)rect.width / rect.height,
                                     (double)rect.height / rect.width);
            if (ratio <= ratio_thresh && area > max_area) {
                max_area = area;
                best_idx = (int)i;
            }
        }
    }

    if (best_idx >= 0) return contours[best_idx];
    return {};
}

std::vector<cv::Point> GazeVectorTracker::optimize_contours_by_angle(
    const std::vector<cv::Point>& contour)
{
    int n = (int)contour.size();
    if (n < 3) return contour;

    int spacing = std::max(1, n / 25);

    // Compute centroid
    cv::Point2f centroid(0, 0);
    for (const auto& pt : contour) {
        centroid.x += pt.x;
        centroid.y += pt.y;
    }
    centroid.x /= n;
    centroid.y /= n;

    std::vector<cv::Point> result;
    double cos_threshold = std::cos(60.0 * CV_PI / 180.0);
    for (int i = 0; i < n; ++i) {
        int prev_idx = (i - spacing + n) % n;
        int next_idx = (i + spacing) % n;

        cv::Point vec1 = contour[prev_idx] - contour[i];
        cv::Point vec2 = contour[next_idx] - contour[i];
        cv::Point vec_to_centroid(
            (int)(centroid.x - contour[i].x),
            (int)(centroid.y - contour[i].y));

        double n1 = std::sqrt((double)(vec_to_centroid.x * vec_to_centroid.x +
                                       vec_to_centroid.y * vec_to_centroid.y));
        double n2 = std::sqrt((double)((vec1.x + vec2.x) * (vec1.x + vec2.x) +
                                       (vec1.y + vec2.y) * (vec1.y + vec2.y)));

        if (n1 < 1e-6 || n2 < 1e-6) continue;

        cv::Point2f v_dir(vec_to_centroid.x / n1, vec_to_centroid.y / n1);
        cv::Point2f v_tangent((vec1.x + vec2.x) / n2, (vec1.y + vec2.y) / n2);

        double dot = v_dir.x * v_tangent.x + v_dir.y * v_tangent.y;
        if (dot >= cos_threshold) {
            result.push_back(contour[i]);
        }
    }

    return result;
}

// ================================================================
// Ellipse goodness
// ================================================================

std::vector<double> GazeVectorTracker::check_ellipse_goodness(
    const cv::Mat& binary_image, const std::vector<cv::Point>& contour,
    const std::optional<cv::RotatedRect>& ellipse_opt)
{
    if (contour.size() < 5) return {0, 0, 0};

    cv::RotatedRect ellipse = ellipse_opt.has_value() ? *ellipse_opt : cv::fitEllipse(contour);

    int h = binary_image.rows, w = binary_image.cols;
    int cx = (int)ellipse.center.x;
    int cy = (int)ellipse.center.y;
    int axis_major = (int)(std::max(ellipse.size.width, ellipse.size.height) / 2) + 5;

    int x1 = std::max(0, cx - axis_major);
    int y1 = std::max(0, cy - axis_major);
    int x2 = std::min(w, cx + axis_major);
    int y2 = std::min(h, cy + axis_major);

    if (x2 <= x1 || y2 <= y1) return {0, 0, 0};

    cv::Mat roi_bin = binary_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    cv::Mat mask = cv::Mat::zeros(roi_bin.size(), CV_8U);

    cv::RotatedRect ellipse_roi(
        cv::Point2f(ellipse.center.x - x1, ellipse.center.y - y1),
        ellipse.size, ellipse.angle);
    cv::ellipse(mask, ellipse_roi, cv::Scalar(255), -1);

    double ellipse_area = cv::sum(mask == 255)[0];
    if (ellipse_area == 0) return {0, 0, 0};

    cv::Mat covered;
    cv::bitwise_and(roi_bin, mask, covered);
    double covered_pixels = cv::sum(covered == 255)[0];

    std::vector<double> goodness(3, 0);
    goodness[0] = covered_pixels / ellipse_area;
    goodness[2] = std::min(ellipse.size.height / ellipse.size.width,
                           ellipse.size.width / ellipse.size.height);

    return goodness;
}

std::vector<double> GazeVectorTracker::check_contour_pixels(
    const std::vector<cv::Point>& contour, cv::Size image_shape,
    const std::optional<cv::RotatedRect>& ellipse_opt)
{
    if (contour.size() < 5) return {0, 0};

    cv::RotatedRect ellipse = ellipse_opt.has_value() ? *ellipse_opt : cv::fitEllipse(contour);

    int h = image_shape.height, w = image_shape.width;
    int cx = (int)ellipse.center.x;
    int cy = (int)ellipse.center.y;
    int axis_major = (int)(std::max(ellipse.size.width, ellipse.size.height) / 2) + 15;

    int x1 = std::max(0, cx - axis_major);
    int y1 = std::max(0, cy - axis_major);
    int x2 = std::min(w, cx + axis_major);
    int y2 = std::min(h, cy + axis_major);

    int roi_w = x2 - x1, roi_h = y2 - y1;
    if (roi_w <= 0 || roi_h <= 0) return {0, 0};

    cv::Mat contour_mask = cv::Mat::zeros(roi_h, roi_w, CV_8U);
    if (!contour.empty()) {
        std::vector<std::vector<cv::Point>> shifted_contours;
        std::vector<cv::Point> shifted;
        for (const auto& pt : contour) {
            shifted.push_back(cv::Point(pt.x - x1, pt.y - y1));
        }
        shifted_contours.push_back(shifted);
        cv::drawContours(contour_mask, shifted_contours, -1, cv::Scalar(255), 1);
    }

    cv::Mat ellipse_mask_thick = cv::Mat::zeros(roi_h, roi_w, CV_8U);
    cv::Mat ellipse_mask_thin = cv::Mat::zeros(roi_h, roi_w, CV_8U);

    cv::RotatedRect ellipse_roi(
        cv::Point2f(ellipse.center.x - x1, ellipse.center.y - y1),
        ellipse.size, ellipse.angle);
    cv::ellipse(ellipse_mask_thick, ellipse_roi, cv::Scalar(255), 10);
    cv::ellipse(ellipse_mask_thin, ellipse_roi, cv::Scalar(255), 4);

    cv::Mat overlap_thick, overlap_thin;
    cv::bitwise_and(contour_mask, ellipse_mask_thick, overlap_thick);
    cv::bitwise_and(contour_mask, ellipse_mask_thin, overlap_thin);

    double absolute_pixel_total_thick = cv::sum(overlap_thick > 0)[0];
    double absolute_pixel_total_thin = cv::sum(overlap_thin > 0)[0];
    double total_border_pixels = cv::sum(contour_mask > 0)[0];

    double ratio_under_ellipse = (total_border_pixels > 0) ?
        absolute_pixel_total_thin / total_border_pixels : 0;

    return {absolute_pixel_total_thick, ratio_under_ellipse};
}

// ================================================================
// Eye sphere radius
// ================================================================

std::optional<double> GazeVectorTracker::distance_to_pupil_outer_edge(
    cv::Point eye_center, const cv::RotatedRect& pupil_ellipse)
{
    double direction_x = pupil_ellipse.center.x - eye_center.x;
    double direction_y = pupil_ellipse.center.y - eye_center.y;
    double center_distance = std::hypot(direction_x, direction_y);

    double semi_axis_x = pupil_ellipse.size.width / 2;
    double semi_axis_y = pupil_ellipse.size.height / 2;

    if (center_distance == 0 || semi_axis_x <= 0 || semi_axis_y <= 0) {
        return std::nullopt;
    }

    double unit_x = direction_x / center_distance;
    double unit_y = direction_y / center_distance;
    double angle_rad = pupil_ellipse.angle * CV_PI / 180.0;
    double cosine = std::cos(angle_rad);
    double sine = std::sin(angle_rad);

    double local_x = cosine * unit_x + sine * unit_y;
    double local_y = -sine * unit_x + cosine * unit_y;

    double edge_offset = 1.0 / std::sqrt(
        (local_x / semi_axis_x) * (local_x / semi_axis_x) +
        (local_y / semi_axis_y) * (local_y / semi_axis_y));

    return center_distance + edge_offset;
}

void GazeVectorTracker::update_eye_sphere_radius(
    cv::Point eye_center, const std::optional<cv::RotatedRect>& current_pupil_ellipse,
    double current_pupil_confidence)
{
    if (sphere_radius_locked_) {
        max_observed_distance_ = locked_sphere_radius_;
        return;
    }

    if (last_sphere_radius_ellipse_.has_value()) {
        auto anchored_distance = distance_to_pupil_outer_edge(
            eye_center, *last_sphere_radius_ellipse_);
        if (anchored_distance.has_value()) {
            max_observed_distance_ = *anchored_distance;
        }
    }

    if (current_pupil_ellipse.has_value() &&
        current_pupil_confidence >= pupil_confidence_threshold_sphere_ &&
        (int)model_centers_.size() >= min_model_centers_)
    {
        auto current_distance = distance_to_pupil_outer_edge(
            eye_center, *current_pupil_ellipse);
        if (current_distance.has_value() &&
            (!last_sphere_radius_ellipse_.has_value() ||
             *current_distance > max_observed_distance_))
        {
            max_observed_distance_ = *current_distance;
            last_sphere_radius_ellipse_ = *current_pupil_ellipse;
        }
    }
}

// ================================================================
// Ray intersections & eye center estimation
// ================================================================

double GazeVectorTracker::angle_diff(double a, double b) {
    double diff = std::abs(a - b);
    while (diff >= 180) diff -= 180;
    return std::min(diff, 180 - diff);
}

std::optional<cv::Point> GazeVectorTracker::find_line_intersection(
    const cv::RotatedRect& e1, const cv::RotatedRect& e2)
{
    double angle1_rad = e1.angle * CV_PI / 180.0;
    double angle2_rad = e2.angle * CV_PI / 180.0;

    double minor1 = std::min(e1.size.width, e1.size.height);
    double minor2 = std::min(e2.size.width, e2.size.height);

    double dx1 = (minor1 / 2) * std::cos(angle1_rad);
    double dy1 = (minor1 / 2) * std::sin(angle1_rad);
    double dx2 = (minor2 / 2) * std::cos(angle2_rad);
    double dy2 = (minor2 / 2) * std::sin(angle2_rad);

    double A[2][2] = {{dx1, -dx2}, {dy1, -dy2}};
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    if (std::abs(det) < 1e-12) return std::nullopt;

    double Bx = e2.center.x - e1.center.x;
    double By = e2.center.y - e1.center.y;

    double t1 = (Bx * A[1][1] - By * A[0][1]) / det;
    // double t2 = (By * A[0][0] - Bx * A[1][0]) / det; // not used

    int ix = (int)(e1.center.x + t1 * dx1);
    int iy = (int)(e1.center.y + t1 * dy1);

    return cv::Point(ix, iy);
}

cv::Point GazeVectorTracker::compute_average_intersection(
    const cv::Mat& frame, const std::vector<cv::RotatedRect>& ray_lines,
    int number_lines, int total_lines, int minimum_angle_degrees)
{
    if ((int)ray_lines.size() < 2 || number_lines < 2) return cv::Point(0, 0);

    int height = frame.rows, width = frame.cols;
    int pixel_limit = 30;
    int angle_threshold = 5;

    // Random sample
    int sample_count = std::min(number_lines, (int)ray_lines.size());
    std::vector<cv::RotatedRect> selected;
    std::vector<int> indices(ray_lines.size());
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = (int)i;
    std::shuffle(indices.begin(), indices.end(), std::mt19937(std::random_device{}()));

    for (int i = 0; i < sample_count; ++i) {
        selected.push_back(ray_lines[indices[i]]);
    }

    std::vector<cv::Point> intersections;
    for (int i = 0; i < (int)selected.size() - 1; ++i) {
        if (angle_diff(selected[i].angle, selected[i+1].angle) >= minimum_angle_degrees) {
            auto intersection = find_line_intersection(selected[i], selected[i+1]);
            if (intersection.has_value() &&
                intersection->x >= 0 && intersection->x < width &&
                intersection->y >= 0 && intersection->y < height)
            {
                intersections.push_back(*intersection);
            }
        }
    }

    if (intersections.empty()) return cv::Point(0, 0);

    bool accept = true;
    if ((int)intersections.size() >= 2) {
        for (size_t i = 0; i < intersections.size() && accept; ++i) {
            for (size_t j = i + 1; j < intersections.size() && accept; ++j) {
                double d = std::hypot(intersections[i].x - intersections[j].x,
                                      intersections[i].y - intersections[j].y);
                if (d > pixel_limit) {
                    accept = false;
                    break;
                }
                double angle_diff_ij = angle_diff(selected[i].angle, selected[j].angle);
                if (angle_diff_ij < angle_threshold) {
                    accept = false;
                    break;
                }
            }
        }
    }

    if (accept) {
        stored_intersections_.insert(stored_intersections_.end(),
                                     intersections.begin(), intersections.end());
    }

    if ((int)stored_intersections_.size() > total_lines) {
        stored_intersections_ = prune_intersections(stored_intersections_, total_lines);
    }

    if (stored_intersections_.empty()) return cv::Point(0, 0);

    double avg_x = 0, avg_y = 0;
    for (const auto& pt : stored_intersections_) {
        avg_x += pt.x;
        avg_y += pt.y;
    }
    avg_x /= stored_intersections_.size();
    avg_y /= stored_intersections_.size();

    return cv::Point((int)avg_x, (int)avg_y);
}

std::vector<cv::Point> GazeVectorTracker::prune_intersections(
    const std::vector<cv::Point>& intersections, int maximum)
{
    if ((int)intersections.size() <= maximum) return intersections;
    return std::vector<cv::Point>(intersections.end() - maximum, intersections.end());
}

cv::Point GazeVectorTracker::update_and_average_point(
    std::vector<cv::Point>& point_list, cv::Point new_point, int N)
{
    point_list.push_back(new_point);
    if ((int)point_list.size() > N) {
        point_list.erase(point_list.begin());
    }
    if (point_list.empty()) return cv::Point(0, 0);

    double avg_x = 0, avg_y = 0;
    for (const auto& p : point_list) {
        avg_x += p.x;
        avg_y += p.y;
    }
    avg_x /= point_list.size();
    avg_y /= point_list.size();

    return cv::Point((int)avg_x, (int)avg_y);
}

// ================================================================
// Gaze vector computation
// ================================================================

std::tuple<std::optional<cv::Point3f>, std::optional<cv::Point3f>, NormalizeResult>
GazeVectorTracker::compute_gaze_vector(
    int x, int y, int center_x, int center_y, double confidence_ratio)
{
    int viewport_w = frame_width_;
    int viewport_h = frame_height_;

    double fov_y_deg = 45.0;
    double aspect_ratio = (double)viewport_w / viewport_h;
    double far_clip = 100.0;

    cv::Point3f camera_position(0.0f, 0.0f, 3.0f);

    double fov_y_rad = fov_y_deg * CV_PI / 180.0;
    double half_height_far = std::tan(fov_y_rad / 2) * far_clip;
    double half_width_far = half_height_far * aspect_ratio;

    double ndc_x = (2.0 * x) / viewport_w - 1.0;
    double ndc_y = 1.0 - (2.0 * y) / viewport_h;

    double far_x = ndc_x * half_width_far;
    double far_y = ndc_y * half_height_far;
    double far_z = camera_position.z - far_clip;

    cv::Point3f far_point(far_x, far_y, far_z);

    cv::Point3f ray_origin = camera_position;
    cv::Point3f ray_direction = far_point - camera_position;
    double dir_len = std::sqrt(ray_direction.x * ray_direction.x +
                               ray_direction.y * ray_direction.y +
                               ray_direction.z * ray_direction.z);
    if (dir_len > 0) {
        ray_direction.x /= -dir_len;
        ray_direction.y /= -dir_len;
        ray_direction.z /= -dir_len;
    }

    double inner_radius = 1.0 / 1.05;
    double sphere_offset_x = ((double)center_x / viewport_w) * 2.0 - 1.0;
    double sphere_offset_y = 1.0 - ((double)center_y / viewport_h) * 2.0;
    cv::Point3f sphere_center(sphere_offset_x * 1.5f, sphere_offset_y * 1.5f, 0.0f);

    // Ray-sphere intersection
    cv::Point3f origin = ray_origin;
    cv::Point3f direction = -ray_direction; // reverse direction
    cv::Point3f L = origin - sphere_center;

    double a = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
    double b = 2 * (direction.x * L.x + direction.y * L.y + direction.z * L.z);
    double c = (L.x * L.x + L.y * L.y + L.z * L.z) - inner_radius * inner_radius;

    double discriminant = b * b - 4 * a * c;

    cv::Point3f target_direction;
    if (discriminant < 0) {
        double t = -(direction.x * L.x + direction.y * L.y + direction.z * L.z) / a;
        cv::Point3f intersection_point = origin + t * direction;
        cv::Point3f intersection_local = intersection_point - sphere_center;
        float local_len = std::sqrt(intersection_local.x * intersection_local.x +
                                     intersection_local.y * intersection_local.y +
                                     intersection_local.z * intersection_local.z);
        if (local_len > 0) {
            target_direction = intersection_local / local_len;
        } else {
            return {std::nullopt, std::nullopt, NormalizeResult{}};
        }
    } else {
        double sqrt_disc = std::sqrt(discriminant);
        double t1 = (-b - sqrt_disc) / (2 * a);
        double t2 = (-b + sqrt_disc) / (2 * a);

        double t = 0;
        if (t1 > 0 && t2 > 0) {
            t = std::min(t1, t2);
        } else if (t1 > 0) {
            t = t1;
        } else if (t2 > 0) {
            t = t2;
        } else {
            return {std::nullopt, std::nullopt, NormalizeResult{}};
        }

        cv::Point3f intersection_point = origin + (float)t * direction;
        cv::Point3f intersection_local = intersection_point - sphere_center;
        float local_len = std::sqrt(intersection_local.x * intersection_local.x +
                                     intersection_local.y * intersection_local.y +
                                     intersection_local.z * intersection_local.z);
        if (local_len > 0) {
            target_direction = intersection_local / local_len;
        } else {
            return {std::nullopt, std::nullopt, NormalizeResult{}};
        }
    }

    cv::Point3f circle_local_center(0.0f, 0.0f, inner_radius);
    float clc_len = std::sqrt(circle_local_center.x * circle_local_center.x +
                               circle_local_center.y * circle_local_center.y +
                               circle_local_center.z * circle_local_center.z);
    if (clc_len > 0) circle_local_center /= clc_len;

    // Rodrigues rotation
    cv::Point3f rotation_axis = circle_local_center.cross(target_direction);
    double rot_axis_norm = std::sqrt(rotation_axis.x * rotation_axis.x +
                                      rotation_axis.y * rotation_axis.y +
                                      rotation_axis.z * rotation_axis.z);

    cv::Point3f gaze_rotated;
    if (rot_axis_norm < 1e-6) {
        gaze_rotated = circle_local_center;
    } else {
        rotation_axis /= rot_axis_norm;

        double dot = circle_local_center.x * target_direction.x +
                     circle_local_center.y * target_direction.y +
                     circle_local_center.z * target_direction.z;
        dot = std::max(-1.0, std::min(1.0, dot));
        double angle_rad = std::acos(dot);

        // Rodrigues rotation matrix
        double c = std::cos(angle_rad);
        double s = std::sin(angle_rad);
        double t = 1 - c;
        double x_a = rotation_axis.x, y_a = rotation_axis.y, z_a = rotation_axis.z;

        // Apply rotation matrix to circle_local_center
        double rot[3][3] = {
            {t * x_a * x_a + c, t * x_a * y_a - s * z_a, t * x_a * z_a + s * y_a},
            {t * x_a * y_a + s * z_a, t * y_a * y_a + c, t * y_a * z_a - s * x_a},
            {t * x_a * z_a - s * y_a, t * y_a * z_a + s * x_a, t * z_a * z_a + c}
        };

        gaze_rotated.x = rot[0][0] * circle_local_center.x +
                         rot[0][1] * circle_local_center.y +
                         rot[0][2] * circle_local_center.z;
        gaze_rotated.y = rot[1][0] * circle_local_center.x +
                         rot[1][1] * circle_local_center.y +
                         rot[1][2] * circle_local_center.z;
        gaze_rotated.z = rot[2][0] * circle_local_center.x +
                         rot[2][1] * circle_local_center.y +
                         rot[2][2] * circle_local_center.z;

        float gaze_len = std::sqrt(gaze_rotated.x * gaze_rotated.x +
                                    gaze_rotated.y * gaze_rotated.y +
                                    gaze_rotated.z * gaze_rotated.z);
        if (gaze_len > 0) gaze_rotated /= gaze_len;
    }

    // Normalize
    std::vector<double> gaze_list = {gaze_rotated.x, gaze_rotated.y, gaze_rotated.z};
    NormalizeResult norm_result = normalizer_.normalize(side_, gaze_list);

    // Send result to Python queue
    if (!py_result_queue_.is_none()) {
        try {
            py::dict result;
            result["side"] = side_;
            py::list gaze_list_py;
            for (double v : gaze_list) {
                gaze_list_py.append(v);
            }
            result["gaze_rotated"] = gaze_list_py;
            result["eye_x"] = norm_result.eye_x.has_value() ? py::cast(*norm_result.eye_x) : py::none();
            result["eye_y"] = norm_result.eye_y.has_value() ? py::cast(*norm_result.eye_y) : py::none();
            result["confidence"] = confidence_ratio;
            result["raw_eye_openness"] = last_tracking_result_.raw_eye_openness;
            py_result_queue_.attr("put_nowait")(result);
        } catch (...) {}
    }

    return {sphere_center, gaze_rotated, norm_result};
}

// ================================================================
// Debug overlay
// ================================================================

void GazeVectorTracker::draw_debug_overlay(
    cv::Mat& frame, cv::Point model_center_average,
    const std::optional<cv::RotatedRect>& final_rotated_rect,
    int center_x, int center_y,
    const std::optional<cv::Point3f>& center_3d,
    const std::optional<cv::Point3f>& gaze_rotated,
    double best_ratio_under_ellipse,
    const NormalizeResult& norm_result)
{
    // Draw search ellipse
    if (last_search_ellipse_.has_value()) {
        cv::ellipse(frame, *last_search_ellipse_, cv::Scalar(0, 200, 0), 2);

        // 在椭圆区域内绘制开度参考线
        cv::RotatedRect ellipse = *last_search_ellipse_;
        int cx = (int)ellipse.center.x;
        int cy = (int)ellipse.center.y;
        int rx = (int)(ellipse.size.width / 2);
        int ry = (int)(ellipse.size.height / 2);
        int line_x_min = std::max(0, cx - rx);
        int line_x_max = std::min(frame.cols, cx + rx);

        if (eye_openness_top_agg_ > 0 && eye_openness_bottom_agg_ > 0) {
            // 绿色水平线 — 最高点
            int top_y = (int)eye_openness_top_agg_;
            cv::line(frame, cv::Point(line_x_min, top_y),
                     cv::Point(line_x_max, top_y), cv::Scalar(0, 255, 0), 2);

            // 红色水平线 — 最低点
            int bottom_y = (int)eye_openness_bottom_agg_;
            cv::line(frame, cv::Point(line_x_min, bottom_y),
                     cv::Point(line_x_max, bottom_y), cv::Scalar(0, 0, 255), 2);

            // 青色垂直线连接两点
            int mid_x = (line_x_min + line_x_max) / 2;
            cv::line(frame, cv::Point(mid_x, top_y),
                     cv::Point(mid_x, bottom_y), cv::Scalar(255, 255, 0), 1);
        }
    }

    // Draw eye sphere
    cv::circle(frame, model_center_average, (int)max_observed_distance_,
               cv::Scalar(255, 50, 50), 2);
    cv::circle(frame, model_center_average, 8, cv::Scalar(255, 255, 0), -1);

    if (final_rotated_rect.has_value() && center_x >= 0 && center_y >= 0) {
        cv::line(frame, model_center_average, cv::Point(center_x, center_y),
                 cv::Scalar(255, 150, 50), 2);
    }

    if (final_rotated_rect.has_value()) {
        cv::ellipse(frame, *final_rotated_rect, cv::Scalar(20, 255, 255), 2);
    }

    if (final_rotated_rect.has_value() && center_x >= 0 && center_y >= 0) {
        int dx = center_x - model_center_average.x;
        int dy = center_y - model_center_average.y;
        int ext_x = model_center_average.x + 2 * dx;
        int ext_y = model_center_average.y + 2 * dy;
        cv::line(frame, cv::Point(center_x, center_y), cv::Point(ext_x, ext_y),
                 cv::Scalar(200, 255, 0), 3);
    }

    if (center_3d.has_value() && gaze_rotated.has_value()) {
        char origin_text[128];
        std::snprintf(origin_text, sizeof(origin_text),
                      "Origin: (%.2f, %.2f, %.2f)",
                      center_3d->x, center_3d->y, center_3d->z);
        char dir_text[128];
        std::snprintf(dir_text, sizeof(dir_text),
                      "Direction: (%.2f, %.2f, %.2f)",
                      gaze_rotated->x, gaze_rotated->y, gaze_rotated->z);

        cv::putText(frame, origin_text, cv::Point(12, frame.rows - 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
        cv::putText(frame, dir_text, cv::Point(12, frame.rows - 13),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
        cv::putText(frame, origin_text, cv::Point(10, frame.rows - 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, dir_text, cv::Point(10, frame.rows - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    // Normalization status HUD
    if (!norm_result.missing.empty()) {
        std::string msg = "Insufficient extreme vectors! (";
        for (size_t i = 0; i < norm_result.missing.size(); ++i) {
            if (i > 0) msg += ", ";
            msg += norm_result.missing[i];
        }
        msg += ") is missing.";
        cv::putText(frame, msg, cv::Point(10, frame.rows - 65),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
        cv::putText(frame, msg, cv::Point(10, frame.rows - 65),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    } else if (norm_result.eye_x.has_value() && norm_result.eye_y.has_value()) {
        char norm_text[128];
        std::snprintf(norm_text, sizeof(norm_text),
                      "Eye X: %+.3f   Eye Y: %+.3f",
                      *norm_result.eye_x, *norm_result.eye_y);
        cv::putText(frame, norm_text, cv::Point(10, frame.rows - 65),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
        cv::putText(frame, norm_text, cv::Point(10, frame.rows - 65),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    char ratio_text[32];
    std::snprintf(ratio_text, sizeof(ratio_text), "%.2f%%", best_ratio_under_ellipse * 100);
    cv::putText(frame, ratio_text, cv::Point(12, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4);
    cv::putText(frame, ratio_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

    cv::imshow(win_name_, frame);
}

// ================================================================
// 眼睛开度检测 — 二值化 + 竖直扫描 + 中位数/平均值聚合
// ================================================================

double GazeVectorTracker::compute_eye_openness(const cv::Mat& gray_frame) {
    int h = gray_frame.rows, w = gray_frame.cols;

    // 获取 dark_search_ellipse 参数（与 get_darkest_area 一致）
    int cx_roi = w / 2, cy_roi = h / 2;
    int rx = (int)((w / 2.0) * dark_search_roi_scale_);
    int ry = (int)((h / 2.0) * dark_search_roi_scale_);

    if (rx <= 0 || ry <= 0) return 0.0;

    // 1. 高斯模糊
    cv::Mat blurred = gray_frame;
    if (eye_openness_blur_ > 1) {
        cv::GaussianBlur(gray_frame, blurred,
                         cv::Size(eye_openness_blur_, eye_openness_blur_), 0);
    }

    // 2. 二值化（THRESH_BINARY_INV：黑色→白色前景）
    cv::Mat binary;
    cv::threshold(blurred, binary, eye_openness_threshold_, 255, cv::THRESH_BINARY_INV);

    // 3. 椭圆掩膜：只保留 dark_search_ellipse 内的像素
    cv::Mat mask = cv::Mat::zeros(h, w, CV_8U);
    cv::ellipse(mask, cv::RotatedRect(cv::Point2f(cx_roi, cy_roi),
                                       cv::Size2f(rx * 2, ry * 2), 0),
                cv::Scalar(255), -1);
    cv::Mat masked;
    cv::bitwise_and(binary, mask, masked);

    // 4. 逐列扫描
    //    限定在椭圆边界框内扫描，提高效率
    int min_x = std::max(0, cx_roi - rx);
    int max_x = std::min(w, cx_roi + rx);
    int min_y = std::max(0, cy_roi - ry);
    int max_y = std::min(h, cy_roi + ry);

    std::vector<double> top_points, bottom_points;

    for (int x = min_x; x < max_x; ++x) {
        int top_y = -1;
        int bottom_y = -1;

        // 从上往下找第一个白色像素
        for (int y = min_y; y < max_y; ++y) {
            if (masked.at<uchar>(y, x) > 0) {
                top_y = y;
                break;
            }
        }

        // 从下往上找第一个白色像素
        for (int y = max_y - 1; y >= min_y; --y) {
            if (masked.at<uchar>(y, x) > 0) {
                bottom_y = y;
                break;
            }
        }

        // 该列必须既有最高点又有最低点，且 top != bottom（高度至少 1）
        if (top_y >= 0 && bottom_y >= 0 && bottom_y > top_y) {
            top_points.push_back((double)top_y);
            bottom_points.push_back((double)bottom_y);
        }
    }

    if (top_points.empty() || bottom_points.empty()) {
        return 0.0;
    }

    // 5. 聚合
    double top_agg, bottom_agg;
    if (eye_openness_aggregation_ == "average") {
        double sum_top = 0, sum_bottom = 0;
        for (size_t i = 0; i < top_points.size(); ++i) {
            sum_top += top_points[i];
            sum_bottom += bottom_points[i];
        }
        top_agg = sum_top / top_points.size();
        bottom_agg = sum_bottom / bottom_points.size();
    } else {
        // 中位数
        size_t n = top_points.size();
        size_t mid = n / 2;
        std::nth_element(top_points.begin(), top_points.begin() + mid, top_points.end());
        std::nth_element(bottom_points.begin(), bottom_points.begin() + mid, bottom_points.end());
        if (n % 2 == 0) {
            // 偶数个，取中间两数的均值
            auto mid2 = mid - 1;
            std::nth_element(top_points.begin(), top_points.begin() + mid2, top_points.end());
            std::nth_element(bottom_points.begin(), bottom_points.begin() + mid2, bottom_points.end());
            top_agg = (top_points[mid] + top_points[mid2]) * 0.5;
            bottom_agg = (bottom_points[mid] + bottom_points[mid2]) * 0.5;
        } else {
            top_agg = top_points[mid];
            bottom_agg = bottom_points[mid];
        }
    }

    double raw_distance = bottom_agg - top_agg;

    // 存储聚合结果供 draw_debug_overlay 使用
    eye_openness_top_agg_ = top_agg;
    eye_openness_bottom_agg_ = bottom_agg;

    return std::max(0.0, raw_distance);
}

} // namespace eye_tracker
