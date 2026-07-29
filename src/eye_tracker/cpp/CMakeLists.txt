cmake_minimum_required(VERSION 3.14)
project(eye_tracker_core_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Find OpenCV (manual paths since cmake config/pkg-config not available) ----
set(OPENCV_INCLUDE_DIRS "/usr/include/opencv4")
set(OPENCV_LIBS
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    opencv_imgcodecs
)

# ---- Find yaml-cpp ----
find_package(yaml-cpp REQUIRED)

# ---- Find pybind11 (system headers are installed) ----
find_package(pybind11 REQUIRED)

# ---- Source files ----
set(SOURCES
    src/normalizer.cpp
    src/tracker.cpp
    src/bindings.cpp
)

# ---- Build shared library ----
pybind11_add_module(eye_tracker_core_cpp ${SOURCES})

target_include_directories(eye_tracker_core_cpp
    PRIVATE
        ${OPENCV_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        src/
)

target_link_libraries(eye_tracker_core_cpp
    PRIVATE
        ${OPENCV_LIBS}
        yaml-cpp
)

# Set output directory to the eye_tracker package dir
set_target_properties(eye_tracker_core_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/.."
)