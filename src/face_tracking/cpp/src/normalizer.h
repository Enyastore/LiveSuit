#ifndef EYE_TRACKER_NORMALIZER_H
#define EYE_TRACKER_NORMALIZER_H

#include "types.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace eye_tracker {

class Normalizer {
public:
    explicit Normalizer(const std::string& refs_file = "references.yaml");

    void reload();
    void set_extreme_vectors(const std::string& side, const std::string& direction,
                             const std::vector<double>& vector);
    void clear_extreme_vectors(const std::string& side);

    NormalizeResult normalize(const std::string& side,
                              const std::vector<double>& gaze_rotated) const;
                              
    void set_openness_ref(const std::string& side, const std::string& ref_type, double value);
    std::optional<double> get_openness_ref(const std::string& side, const std::string& ref_type) const;
    void clear_openness_ref(const std::string& side);

private:
    std::string refs_file_path_;
    std::unordered_map<std::string, std::vector<double>> extreme_vectors_;

    // 眼睛开度参考值内存缓存（避免每帧读盘，行为与注视极值向量一致）
    // key 为 side（"left"/"right"）
    std::unordered_map<std::string, double> cached_open_ref_;
    std::unordered_map<std::string, double> cached_close_ref_;

    void load_refs();

    static std::tuple<std::optional<double>, std::optional<double>>
    orthogonal_project(const std::vector<double>& current,
                       const std::vector<double>& inner,
                       const std::vector<double>& outer,
                       const std::vector<double>& up,
                       const std::vector<double>& down);

    // Resolve refs file path relative to the Python module directory
    std::string resolve_path(const std::string& filename) const;
};

} // namespace eye_tracker

#endif // EYE_TRACKER_NORMALIZER_H