#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include "tracker.h"
#include "normalizer.h"

namespace py = pybind11;
using namespace eye_tracker;

PYBIND11_MODULE(eye_tracker_core_cpp, m) {
    m.doc() = "C++ implementation of eye tracking core algorithms";

    // ---- 归一化器 ----
    py::class_<Normalizer>(m, "Normalizer")
        .def(py::init<const std::string&>(), py::arg("extreme_file") = "extreme_vectors.yaml")
        .def("reload", &Normalizer::reload)
        .def("set_extreme", &Normalizer::set_extreme,
             py::arg("side"), py::arg("direction"), py::arg("vector"))
        .def("clear_extremes", &Normalizer::clear_extremes, py::arg("side"))
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
             py::arg("extreme_file") = "extreme_vectors.yaml",
             py::arg("use_recommended_resolution") = true,
             py::arg("dark_search_roi_scale") = 0.70,
             py::arg("fourcc_str") = "",
             py::arg("brightness") = 0.0,
             py::arg("contrast") = 1.0,
             py::arg("openness_threshold_low") = 0,
             py::arg("openness_threshold_high") = 80,
             py::arg("pupil_threshold_low") = 5,
             py::arg("pupil_threshold_high") = 25)
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
        .def("is_headless", &GazeVectorTracker::is_headless);

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