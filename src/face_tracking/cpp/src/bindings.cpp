#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include "tracker.h"
#include "normalizer.h"

namespace py = pybind11;
using namespace eye_tracker;

namespace {

// numpy.ndarray (uint8, C 连续) → cv::Mat（零拷贝，仅限同步调用）
cv::Mat numpy_to_cv_mat(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr) {
    auto buf = arr.request();
    if (buf.ndim == 3) {
        return cv::Mat((int)buf.shape[0], (int)buf.shape[1], CV_8UC3, buf.ptr);
    }
    return cv::Mat((int)buf.shape[0], (int)buf.shape[1], CV_8UC1, buf.ptr);
}

// cv::Mat → numpy.ndarray（拷贝，通过 capsule 管理生命周期）
py::array_t<uint8_t> cv_mat_to_numpy(const cv::Mat& mat) {
    if (mat.empty()) {
        std::vector<py::ssize_t> shape = {0, 0};
        return py::array_t<uint8_t>(shape, nullptr);
    }
    cv::Mat* clone = new cv::Mat(mat.clone());
    py::capsule cap(clone, [](void* p) { delete static_cast<cv::Mat*>(p); });
    return py::array_t<uint8_t>(
        {clone->rows, clone->cols},
        {clone->step[0], clone->step[1]},
        clone->data,
        cap);
}

} // namespace

PYBIND11_MODULE(eye_tracker_core_cpp, m) {
    m.doc() = "C++ implementation of eye tracking core algorithms";

    // ---- 归一化器 ----
    py::class_<Normalizer>(m, "Normalizer")
        .def(py::init<const std::string&>(), py::arg("refs_file") = "references.yaml")
        .def("reload", &Normalizer::reload)
        .def("set_extreme_vectors", &Normalizer::set_extreme_vectors,
             py::arg("side"), py::arg("direction"), py::arg("vector"))
        .def("clear_extreme_vectors", &Normalizer::clear_extreme_vectors, py::arg("side"))
        .def("normalize", &Normalizer::normalize,
             py::arg("side"), py::arg("gaze_rotated"))
        // 眼睛开度标定
        .def("set_openness_ref", &Normalizer::set_openness_ref,
             py::arg("side"), py::arg("ref_type"), py::arg("value"))
        .def("get_openness_ref", &Normalizer::get_openness_ref,
             py::arg("side"), py::arg("ref_type"))
        .def("clear_openness_ref", &Normalizer::clear_openness_ref,
             py::arg("side"));

    // ---- 归一化结果  ----
    py::class_<NormalizeResult>(m, "NormalizeResult")
        .def_readonly("eye_x", &NormalizeResult::eye_x)
        .def_readonly("eye_y", &NormalizeResult::eye_y)
        .def_readonly("missing", &NormalizeResult::missing);

    // ---- 瞳孔椭圆 ----
    py::class_<PupilEllipse>(m, "PupilEllipse")
        .def_readonly("center", &PupilEllipse::center)
        .def_readonly("axes", &PupilEllipse::axes)
        .def_readonly("angle_degrees", &PupilEllipse::angle_degrees);

    // ---- 追踪结果 ----
    py::class_<TrackingResult>(m, "TrackingResult")
        .def_readonly("pupil_ellipse", &TrackingResult::pupil_ellipse)
        .def_readonly("eye_center", &TrackingResult::eye_center)
        .def_readonly("sphere_radius", &TrackingResult::sphere_radius);

    // ---- 瞳孔检测调试结果 ----
    py::class_<PupilDebugResult>(m, "PupilDebugResult")
        .def_readonly("valid", &PupilDebugResult::valid)
        .def_property_readonly("binary", [](const PupilDebugResult& r) { return cv_mat_to_numpy(r.binary); })
        .def_property_readonly("dilated", [](const PupilDebugResult& r) { return cv_mat_to_numpy(r.dilated); })
        .def_readonly("darkest_point", &PupilDebugResult::darkest_point)
        .def_readonly("darkest_pixel_value", &PupilDebugResult::darkest_pixel_value)
        .def_readonly("ellipse_found", &PupilDebugResult::ellipse_found)
        .def_readonly("ellipse_center", &PupilDebugResult::ellipse_center)
        .def_readonly("ellipse_axes", &PupilDebugResult::ellipse_axes)
        .def_readonly("ellipse_angle", &PupilDebugResult::ellipse_angle)
        .def_readonly("goodness_cover", &PupilDebugResult::goodness_cover)
        .def_readonly("goodness_aspect", &PupilDebugResult::goodness_aspect)
        .def_readonly("goodness_total", &PupilDebugResult::goodness_total)
        .def_readonly("ratio_under_ellipse", &PupilDebugResult::ratio_under_ellipse)
        .def_readonly("roi_cx", &PupilDebugResult::roi_cx)
        .def_readonly("roi_cy", &PupilDebugResult::roi_cy)
        .def_readonly("roi_rx", &PupilDebugResult::roi_rx)
        .def_readonly("roi_ry", &PupilDebugResult::roi_ry);

    // ---- 眼睛开度检测调试结果 ----
    py::class_<OpennessDebugResult>(m, "OpennessDebugResult")
        .def_readonly("valid", &OpennessDebugResult::valid)
        .def_property_readonly("binary", [](const OpennessDebugResult& r) { return cv_mat_to_numpy(r.binary); })
        .def_property_readonly("masked", [](const OpennessDebugResult& r) { return cv_mat_to_numpy(r.masked); })
        .def_readonly("top_agg", &OpennessDebugResult::top_agg)
        .def_readonly("bottom_agg", &OpennessDebugResult::bottom_agg)
        .def_readonly("raw_distance", &OpennessDebugResult::raw_distance)
        .def_readonly("roi_cx", &OpennessDebugResult::roi_cx)
        .def_readonly("roi_cy", &OpennessDebugResult::roi_cy)
        .def_readonly("roi_rx", &OpennessDebugResult::roi_rx)
        .def_readonly("roi_ry", &OpennessDebugResult::roi_ry);

    // ---- 注视向量追踪器 ----
    py::class_<GazeVectorTracker>(m, "GazeVectorTracker")
        .def(py::init<
                int, bool, std::vector<int>, std::string,
                int, int, int, std::string,
                bool, double, std::string,
                double, double,
                int, int, int, int>(),
             py::arg("cam_index") = 0,
             py::arg("flip") = false,
             py::arg("crop") = std::vector<int>{0, 0, 640, 480},
             py::arg("side") = "left",
             py::arg("frame_width") = 640,
             py::arg("frame_height") = 480,
             py::arg("frame_rate") = 30,
             py::arg("refs_file") = "references.yaml",
             py::arg("use_recommended_resolution") = true,
             py::arg("dark_search_roi_scale") = 0.70,
             py::arg("fourcc_str") = "",
             py::arg("brightness") = 0.0,
             py::arg("contrast") = 1.0,
             py::arg("openness_threshold_low") = 0,
             py::arg("openness_threshold_high") = 80,
             py::arg("pupil_threshold_low") = 0,
             py::arg("pupil_threshold_high") = 50)
        .def("start_tracking", &GazeVectorTracker::start_tracking,
             py::arg("command_queue") = py::none(),
             py::arg("result_queue") = py::none(),
             py::arg("headless") = false)
        .def("lock_sphere_radius", &GazeVectorTracker::lock_sphere_radius)
        .def("unlock_sphere_radius", &GazeVectorTracker::unlock_sphere_radius)
        .def("lock_eye_center", &GazeVectorTracker::lock_eye_center)
        .def("unlock_eye_center", &GazeVectorTracker::unlock_eye_center)
        .def("stop", &GazeVectorTracker::stop)
        .def("get_last_tracking_result", &GazeVectorTracker::get_last_tracking_result)
        .def("is_running", &GazeVectorTracker::is_running)
        .def("is_headless", &GazeVectorTracker::is_headless)
        // 静态调试接口（供调试面板调用，与正式算法共用实现）
        .def_static("debug_pupil_detect",
            [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> frame,
               int pupil_threshold_low, int pupil_threshold_high,
               double dark_search_roi_scale, const std::vector<int>& crop,
               int area_thresh, int ratio_thresh) {
                cv::Mat cv_frame = numpy_to_cv_mat(frame);
                return GazeVectorTracker::debug_pupil_detect(
                    cv_frame, pupil_threshold_low, pupil_threshold_high,
                    dark_search_roi_scale, crop, area_thresh, ratio_thresh);
            },
            py::arg("frame"), py::arg("pupil_threshold_low"), py::arg("pupil_threshold_high"),
            py::arg("dark_search_roi_scale"), py::arg("crop") = std::vector<int>{},
            py::arg("area_thresh") = 200, py::arg("ratio_thresh") = 4)
        .def_static("debug_openness_detect",
            [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> frame,
               int openness_threshold_low, int openness_threshold_high,
               int blur_kernel, const std::string& aggregation,
               double dark_search_roi_scale, const std::vector<int>& crop) {
                cv::Mat cv_frame = numpy_to_cv_mat(frame);
                return GazeVectorTracker::debug_openness_detect(
                    cv_frame, openness_threshold_low, openness_threshold_high,
                    blur_kernel, aggregation, dark_search_roi_scale, crop);
            },
            py::arg("frame"), py::arg("openness_threshold_low"), py::arg("openness_threshold_high"),
            py::arg("blur_kernel"), py::arg("aggregation"), py::arg("dark_search_roi_scale"),
            py::arg("crop") = std::vector<int>{});

    // ---- cv::点坐标工具类--
    py::class_<cv::Point>(m, "CvPoint")
        .def_readonly("x", &cv::Point::x)
        .def_readonly("y", &cv::Point::y)
        .def("__repr__", [](const cv::Point& p) {
            return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        });

    py::class_<cv::Point2f>(m, "CvPoint2f")
        .def_readonly("x", &cv::Point2f::x)
        .def_readonly("y", &cv::Point2f::y)
        .def("__repr__", [](const cv::Point2f& p) {
            return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        });
}