#ifndef EYE_TRACKER_NORMALIZER_H
#define EYE_TRACKER_NORMALIZER_H

#include "types.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace eye_tracker {

class Normalizer {
public:
    explicit Normalizer(const std::string& extreme_file = "extreme_vectors.yaml");

    void reload();
    void set_extreme(const std::string& side, const std::string& direction,
                     const std::vector<double>& vector);
    void clear_extremes(const std::string& side);

    NormalizeResult normalize(const std::string& side,
                              const std::vector<double>& gaze_rotated) const;

private:
    std::string extreme_file_path_;
    std::unordered_map<std::string, std::vector<double>> extremes_;

    void load_extremes();

    static std::tuple<std::optional<double>, std::optional<double>>
    orthogonal_project(const std::vector<double>& current,
                       const std::vector<double>& inner,
                       const std::vector<double>& outer,
                       const std::vector<double>& up,
                       const std::vector<double>& down);

    // Resolve extreme file path relative to the Python module directory
    std::string resolve_path(const std::string& filename) const;
};

} // namespace eye_tracker

#endif // EYE_TRACKER_NORMALIZER_H