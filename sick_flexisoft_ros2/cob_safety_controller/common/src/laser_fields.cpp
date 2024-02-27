#include <tinyxml2.h>

#include <iostream>
#include <locale>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <boost/unordered_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include <cob_safety_controller/laser_fields.h>
#include "../include/cob_safety_controller/laser_fields.h"

class comma_numpunct : public std::numpunct<char>
{
  protected:
    virtual char do_thousands_sep() const { return ','; }
};

bool LaserFields::init(const std::string &file, const std::string &fallback){
    fallback_ = fallback;
    
    std::locale comma_locale(std::locale(), new comma_numpunct());
    // open file
    tinyxml2::XMLDocument doc;
    std::cout << file << std::endl;
    tinyxml2::XMLError eResult = doc.LoadFile( file.c_str() );
    if(eResult != tinyxml2::XML_SUCCESS) return false;
    
    // for each case
    for (tinyxml2::XMLElement *area = doc.FirstChildElement("AreaList")->FirstChildElement("Area"); area; area = area->NextSiblingElement())
    {
        std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>> &c = fields_[area->Attribute("Name")];   //std::unordered instead of boost::unordered
        // for each field
        for (tinyxml2::XMLElement *field = area->FirstChildElement("FieldList")->FirstChildElement("Field"); field; field = field->NextSiblingElement())
        {
            std::vector<geometry_msgs::msg::Point> &points = c[field->Attribute("Type")];
            // for all points
            for (tinyxml2::XMLElement *point = field->FirstChildElement("UserPointList")->FirstChildElement("UserPoint"); point; point = point->NextSiblingElement())
            {
                geometry_msgs::msg::Point pt;

                double radius, angle;

                std::istringstream de_radius(point->Attribute("Distance"));
                de_radius.imbue(comma_locale);
                de_radius >> radius;
                radius /= 100.0;

                std::istringstream de_angle(point->Attribute("Angle"));
                de_angle.imbue(comma_locale);
                de_angle >> angle;
                angle -= 135;
                angle *= M_PI / 180.0;

                pt.x = radius * cos(angle);
                pt.y = radius * sin(angle);
                points.push_back(pt);
            }
        }
    }
    return fields_.find(fallback_) != fields_.end();
}

const std::vector<geometry_msgs::msg::Point> &LaserFields::get_field(const std::string &c, const std::string &area)
{
    static const std::vector<geometry_msgs::msg::Point> empty;

    auto case_it = fields_.find(c);
    if (case_it == fields_.end())
    {
        case_it = fields_.find(fallback_);
    }
    if (case_it == fields_.end())
    {
        return empty;
    }

    auto area_it = case_it->second.find(area);
    if (area_it == case_it->second.end())
    {
        return empty;
    }
    else
    {
        return area_it->second;
    }
}

bool LaserFields::fill_marker(visualization_msgs::msg::Marker &marker, const std::string &c, const std::string &area)
{
    const auto &field = get_field(c, area);
    geometry_msgs::msg::Point zero;

    switch (marker.type)
    {
    case visualization_msgs::msg::Marker::TRIANGLE_LIST:
        marker.points.clear();
        for (auto it = field.begin() + 1; it != field.end(); ++it)
        {
            marker.points.push_back(zero);
            marker.points.push_back(*(it - 1));
            marker.points.push_back(*it);
        }
        return true;
        break;
    case visualization_msgs::msg::Marker::LINE_LIST:
        marker.points.clear();
        for (auto it = field.begin(); it != field.end(); ++it)
        {
            marker.points.push_back(zero);
            marker.points.push_back(*it);
        }
        return true;
        break;
    case visualization_msgs::msg::Marker::LINE_STRIP:
    case visualization_msgs::msg::Marker::CUBE_LIST:
    case visualization_msgs::msg::Marker::SPHERE_LIST:
    case visualization_msgs::msg::Marker::POINTS:
        marker.points = field;
        return true;
        break;
    default:
        return false;
    }
}

