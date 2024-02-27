#ifndef H_LASER_FIELDS
#define H_LASER_FIELDS

#include <unordered_map>
#include <vector>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

class LaserFields {
    typedef std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>> fieldset_type;
    std::unordered_map<std::string, fieldset_type> fields_;
    std::string fallback_;

public:
    bool init(const std::string &file, const std::string &fallback);
    const std::vector<geometry_msgs::msg::Point>& get_field(const std::string &c, const std::string &area);
    bool fill_marker(visualization_msgs::msg::Marker &marker, const std::string &c, const std::string &area);
};

#endif
