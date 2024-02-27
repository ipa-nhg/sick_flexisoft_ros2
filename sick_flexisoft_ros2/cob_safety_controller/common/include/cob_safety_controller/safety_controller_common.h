#ifndef H_SAFETY_CONTROLLER_COMMON
#define H_SAFETY_CONTROLLER_COMMON

// #include <ros/ros.h>
// #include <XmlRpcException.h>
// #include <boost/static_assert.hpp>
// #include <boost/format.hpp>
// #include <boost/scoped_ptr.hpp>

// #include <cob_msgs/EmergencyStopState.h>
// #include <cob_msgs/SafetyControllerState.h>
// #include <cob_srvs/SetInt.h>
// #include <diagnostic_msgs/DiagnosticArray.h>
// #include <diagnostic_updater/DiagnosticStatusWrapper.h>
// #include <geometry_msgs/Polygon.h>
// #include <geometry_msgs/PolygonStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <sick_flexisoft_client/flexiclient.h>
// #include <std_msgs/Bool.h>
// #include <visualization_msgs/MarkerArray.h>

// #include <cob_safety_controller/laser_fields.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
// #include <XmlRpcException.h> // You may need to replace this with ROS 2 equivalent if available
#include <boost/static_assert.hpp>
#include <boost/format.hpp>
#include <boost/scoped_ptr.hpp>

#include <cob_msgs/msg/emergency_stop_state.hpp>
#include <cob_msgs/msg/safety_controller_state.hpp>
#include <cob_srvs/cob_srvs/srv/set_int.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/odometry.hpp>
// Include other ROS 2 message types as needed

// You may need to replace this with appropriate ROS 2 package and header if available
#include <sick_flexisoft_client/flexiclient.h>

#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "laser_fields.h"

void parseFootprint(const std::string &name, geometry_msgs::msg::Polygon &poly)
{
    poly.points.clear();
    auto node = rclcpp::Node::make_shared("parse_footprint_node");
    try
    {
        auto points = node->get_parameter(name).as_double_array();
        for (size_t i = 0; i < points.size(); ++i)
        {
            geometry_msgs::msg::Point32 p;

            if (points.size() == 3)
            {
                p.x = points[0];
                p.y = points[1];
                p.z = points[2];
            }
            else if (points.size() == 2)
            {
                p.x = points[0];
                p.y = points[1];
                p.z = 0;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Invalid point in %s at index %zu", name.c_str(), i);
                return;
            }
            poly.points.push_back(p);
        }
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_WARN(node->get_logger(), "Could not read %s, skipping", name.c_str());
        return;
    }
    // RCLCPP_INFO(node->get_logger(), "Footprint %s: %s", name.c_str(), poly);
}

void rotateFootprint(const geometry_msgs::msg::Polygon &input, double theta, geometry_msgs::msg::Polygon &output)
{
    double cs = cos(theta);
    double sn = sin(theta);

    output.points.clear();
    output.points.reserve(input.points.size());

    for (size_t i = 0; i < input.points.size(); ++i)
    {
        geometry_msgs::msg::Point32 p;
        p.z = input.points[i].z;
        p.x = input.points[i].x * cs + input.points[i].y * sn;
        p.y = -input.points[i].x * sn + input.points[i].y * cs;
        output.points.push_back(p);
    }
}

class safety_controller_config
{
public:
    std::string port;
    std::string host;
    double threshold_linear_slow;
    double threshold_linear_fast;
    double threshold_angular_fast;
    std::string front_xml;
    std::string left_xml;
    std::string right_xml;
    int marker_type;
    double scale_x;
    double scale_y;
    double scale_z;
    double odometry_timeout;
    bool check_external_state_valid;
    double state_is_valid_timeout;
    bool has_wireless_emstop;
    bool warn_for_laser_bridged;
    bool warn_for_wireless_bridged;
    double reconnect_duration;
    int reconnect_attempts;
    bool has_magnetic_safety_switch;
};

class safety_controller_data
{
public:
    // input data
    nav_msgs::msg::Odometry in_odometry;
    std_msgs::msg::Bool in_state_is_valid;
    rclcpp::Time stamp_state_is_valid;

    // output data
    visualization_msgs::msg::MarkerArray out_marker;
    bool out_marker_active;
    cob_msgs::msg::EmergencyStopState out_emergency_stop_state;
    bool out_emergency_stop_state_active;
    cob_msgs::msg::SafetyControllerState out_safety_controller_state;
    bool out_safety_controller_state_active;
    diagnostic_msgs::msg::DiagnosticArray out_diagnostics;
    bool out_diagnostics_active;
    geometry_msgs::msg::Polygon out_footprint;
    bool out_footprint_active;
    geometry_msgs::msg::PolygonStamped out_footprint_stamped;
    bool out_footprint_stamped_active;
};

class safety_controller_impl : public rclcpp::Node
{
    /* protected region user member variables on begin */
    struct FlexiInput{
        /*0.0*/ uint8_t gateway_not_connected:1;
        /*0.1*/ uint8_t laser_stop_ok:1;
        /*0.2*/ uint8_t:1;
        /*0.3*/ uint8_t laser_1_dirty:1;
        /*0.4*/ uint8_t laser_2_dirty:1;
        /*0.5*/ uint8_t laser_3_dirty:1;
        /*0.6*/ uint8_t input06:1; //empty or fall_sensors_released
        /*0.7*/ uint8_t ack_needed:1;

        /*1.0 to 3.7*/ uint8_t _bytes_1_to_3[3-1+1];

        /*4.0*/ uint8_t external_stop_ok:1;
        /*4.1*/ uint8_t:1;
        /*4.2*/ uint8_t brake_auto:1;
        /*4.3*/ uint8_t:1;
        /*4.4*/ uint8_t input44:1; //wireless_emstop_ok or fall_sensor_front_ok
        /*4.5*/ uint8_t input45:1; //empty or fall_sensor_left_ok
        /*4.6*/ uint8_t input46:1; //laser_bridged or fall_sensor_right_ok
        /*4.7*/ uint8_t input47:1; //wireless_bridged or empty

        /*5.0 to 15.7*/ uint8_t _bytes_5_to_15[15-5+1];

        /*16.0*/ uint8_t base_active:1;
        /*16.1*/ uint8_t:1;
        /*16.2*/ uint8_t torso_active:1;
        /*16.3*/ uint8_t:1;
        /*16.4*/ uint8_t:1;
        /*16.5*/ uint8_t:1;
        /*16.6*/ uint8_t:1;
        /*16.7*/ uint8_t:1;

        /*17.0 to 24.7*/ uint8_t _bytes_17_to_24[24-17+1];

        /*25.0*/ uint8_t:1;
        /*25.1*/ uint8_t input251:1; //empty or efi_bus_front_io_error
        /*25.2*/ uint8_t:1;
        /*25.3*/ uint8_t:1;
        /*25.4*/ uint8_t:1;
        /*25.5*/ uint8_t:1;
        /*25.6*/ uint8_t:1;
        /*25.7*/ uint8_t:1;

        /*26.0*/ uint8_t:1;
        /*26.1*/ uint8_t input261:1; //empty or efi_bus_left_io_error
        /*26.2*/ uint8_t:1;
        /*26.3*/ uint8_t:1;
        /*26.4*/ uint8_t:1;
        /*26.5*/ uint8_t:1;
        /*26.6*/ uint8_t:1;
        /*26.7*/ uint8_t:1;

        /*27.0*/ uint8_t:1;
        /*27.1*/ uint8_t input271:1; //empty or efi_bus_right_io_error
        /*27.2*/ uint8_t:1;
        /*27.3*/ uint8_t:1;
        /*27.4*/ uint8_t:1;
        /*27.5*/ uint8_t:1;
        /*27.6*/ uint8_t:1;
        /*27.7*/ uint8_t:1;

        /*28.0*/ uint8_t:1;
        /*28.1*/ uint8_t:1;
        /*28.2*/ uint8_t:1;
        /*28.3*/ uint8_t:1;
        /*28.4*/ uint8_t front_1_free:1;
        /*28.5*/ uint8_t front_2_free:1;
        /*28.6*/ uint8_t:1;
        /*28.7*/ uint8_t front_3_free:1;

        /*29.0*/ uint8_t:1;
        /*29.1*/ uint8_t:1;
        /*29.2*/ uint8_t:1;
        /*29.3*/ uint8_t:1;
        /*29.4*/ uint8_t left_1_free:1;
        /*29.5*/ uint8_t left_2_free:1;
        /*29.6*/ uint8_t:1;
        /*29.7*/ uint8_t left_3_free:1;

        /*30.0*/ uint8_t:1;
        /*30.1*/ uint8_t:1;
        /*30.2*/ uint8_t:1;
        /*30.3*/ uint8_t:1;
        /*30.4*/ uint8_t right_1_free:1;
        /*30.5*/ uint8_t right_2_free:1;
        /*30.6*/ uint8_t:1;
        /*30.7*/ uint8_t right_3_free:1;

    }  __attribute__ ((__packed__));
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInput) <= flexi::FlexiInputData::max_data_length, "FlexiInput does not fit into payload");
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInput) == 31, "FlexiInput is not 31 bytes");

    struct FlexiInputWireless{
        /*0.0*/ uint8_t gateway_not_connected:1;
        /*0.1*/ uint8_t laser_stop_ok:1;
        /*0.2*/ uint8_t:1;
        /*0.3*/ uint8_t laser_1_dirty:1;
        /*0.4*/ uint8_t laser_2_dirty:1;
        /*0.5*/ uint8_t laser_3_dirty:1;
        /*0.6*/ uint8_t:1;
        /*0.7*/ uint8_t ack_needed:1;

        /*1.0 to 3.7*/ uint8_t _bytes_1_to_3[3-1+1];

        /*4.0*/ uint8_t external_stop_ok:1;
        /*4.1*/ uint8_t:1;
        /*4.2*/ uint8_t brake_auto:1;
        /*4.3*/ uint8_t:1;
        /*4.4*/ uint8_t wireless_emstop_ok:1;
        /*4.5*/ uint8_t:1;
        /*4.6*/ uint8_t laser_bridged:1;
        /*4.7*/ uint8_t wireless_bridged:1;

        /*5.0 to 15.7*/ uint8_t _bytes_5_to_15[15-5+1];

        /*16.0*/ uint8_t base_active:1;
        /*16.1*/ uint8_t:1;
        /*16.2*/ uint8_t torso_active:1;
        /*16.3*/ uint8_t:1;
        /*16.4*/ uint8_t:1;
        /*16.5*/ uint8_t:1;
        /*16.6*/ uint8_t:1;
        /*16.7*/ uint8_t:1;

        /*17.0 to 27.7*/ uint8_t _bytes_17_to_27[27-17+1];

        /*28.0*/ uint8_t:1;
        /*28.1*/ uint8_t:1;
        /*28.2*/ uint8_t:1;
        /*28.3*/ uint8_t:1;
        /*28.4*/ uint8_t front_1_free:1;
        /*28.5*/ uint8_t front_2_free:1;
        /*28.6*/ uint8_t:1;
        /*28.7*/ uint8_t front_3_free:1;

        /*29.0*/ uint8_t:1;
        /*29.1*/ uint8_t:1;
        /*29.2*/ uint8_t:1;
        /*29.3*/ uint8_t:1;
        /*29.4*/ uint8_t left_1_free:1;
        /*29.5*/ uint8_t left_2_free:1;
        /*29.6*/ uint8_t:1;
        /*29.7*/ uint8_t left_3_free:1;

        /*30.0*/ uint8_t:1;
        /*30.1*/ uint8_t:1;
        /*30.2*/ uint8_t:1;
        /*30.3*/ uint8_t:1;
        /*30.4*/ uint8_t right_1_free:1;
        /*30.5*/ uint8_t right_2_free:1;
        /*30.6*/ uint8_t:1;
        /*30.7*/ uint8_t right_3_free:1;

    }  __attribute__ ((__packed__));
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInputWireless) <= flexi::FlexiInputData::max_data_length, "FlexiInputWireless does not fit into payload");
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInputWireless) == 31, "FlexiInputWireless is not 31 bytes");

    /* protected region user member variables on begin */
    struct FlexiInputFallSensors{
        /*0.0*/ uint8_t gateway_not_connected:1;
        /*0.1*/ uint8_t laser_stop_ok:1;
        /*0.2*/ uint8_t:1;
        /*0.3*/ uint8_t laser_1_dirty:1;
        /*0.4*/ uint8_t laser_2_dirty:1;
        /*0.5*/ uint8_t laser_3_dirty:1;
        /*0.6*/ uint8_t fall_sensors_released:1;
        /*0.7*/ uint8_t ack_needed:1;

        /*1.0 to 3.7*/ uint8_t _bytes_1_to_3[3-1+1];

        /*4.0*/ uint8_t external_stop_ok:1;
        /*4.1*/ uint8_t:1;
        /*4.2*/ uint8_t brake_auto:1;
        /*4.3*/ uint8_t:1;
        /*4.4*/ uint8_t fall_sensor_front_ok:1;
        /*4.5*/ uint8_t fall_sensor_left_ok:1;
        /*4.6*/ uint8_t fall_sensor_right_ok:1;
        /*4.7*/ uint8_t:1;

        /*5.0*/ uint8_t magnetic_safety_switch:1;
        /*5.1*/ uint8_t input51:1;
        /*5.2*/ uint8_t external_stop_2_ok:1;
        /*5.3*/ uint8_t input53:1;
        /*5.4*/ uint8_t digital_in_0:1;
        /*5.5*/ uint8_t digital_in_1:1;
        /*5.6*/ uint8_t digital_in_2:1;
        /*5.7*/ uint8_t digital_in_3:1;

        /*6.0 to 15.7*/ uint8_t _bytes_6_to_15[15-6+1];

        /*16.0*/ uint8_t base_active:1;
        /*16.1*/ uint8_t:1;
        /*16.2*/ uint8_t torso_active:1;
        /*16.3*/ uint8_t:1;
        /*16.4*/ uint8_t:1;
        /*16.5*/ uint8_t:1;
        /*16.6*/ uint8_t:1;
        /*16.7*/ uint8_t:1;

        /*17.0*/ uint8_t digital_out_0:1;
        /*17.1*/ uint8_t digital_out_1:1;
        /*17.2*/ uint8_t digital_out_2:1;
        /*17.3*/ uint8_t digital_out_3:1;
        /*17.4*/ uint8_t:1;
        /*17.5*/ uint8_t:1;
        /*17.6*/ uint8_t:1;
        /*17.7*/ uint8_t:1;

        /*18.0 to 24.7*/ uint8_t _bytes_18_to_24[24-18+1];

        /*25.0*/ uint8_t:1;
        /*25.1*/ uint8_t efi_bus_front_io_error:1;
        /*25.2*/ uint8_t:1;
        /*25.3*/ uint8_t:1;
        /*25.4*/ uint8_t:1;
        /*25.5*/ uint8_t:1;
        /*25.6*/ uint8_t:1;
        /*25.7*/ uint8_t:1;

        /*26.0*/ uint8_t:1;
        /*26.1*/ uint8_t efi_bus_left_io_error:1;
        /*26.2*/ uint8_t:1;
        /*26.3*/ uint8_t:1;
        /*26.4*/ uint8_t:1;
        /*26.5*/ uint8_t:1;
        /*26.6*/ uint8_t:1;
        /*26.7*/ uint8_t:1;

        /*27.0*/ uint8_t:1;
        /*27.1*/ uint8_t efi_bus_right_io_error:1;
        /*27.2*/ uint8_t:1;
        /*27.3*/ uint8_t:1;
        /*27.4*/ uint8_t:1;
        /*27.5*/ uint8_t:1;
        /*27.6*/ uint8_t:1;
        /*27.7*/ uint8_t:1;

        /*28.0*/ uint8_t:1;
        /*28.1*/ uint8_t:1;
        /*28.2*/ uint8_t:1;
        /*28.3*/ uint8_t:1;
        /*28.4*/ uint8_t front_1_free:1;
        /*28.5*/ uint8_t front_2_free:1;
        /*28.6*/ uint8_t:1;
        /*28.7*/ uint8_t front_3_free:1;

        /*29.0*/ uint8_t:1;
        /*29.1*/ uint8_t:1;
        /*29.2*/ uint8_t:1;
        /*29.3*/ uint8_t:1;
        /*29.4*/ uint8_t left_1_free:1;
        /*29.5*/ uint8_t left_2_free:1;
        /*29.6*/ uint8_t:1;
        /*29.7*/ uint8_t left_3_free:1;

        /*30.0*/ uint8_t:1;
        /*30.1*/ uint8_t:1;
        /*30.2*/ uint8_t:1;
        /*30.3*/ uint8_t:1;
        /*30.4*/ uint8_t right_1_free:1;
        /*30.5*/ uint8_t right_2_free:1;
        /*30.6*/ uint8_t:1;
        /*30.7*/ uint8_t right_3_free:1;

    }  __attribute__ ((__packed__));
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInputFallSensors) <= flexi::FlexiInputData::max_data_length, "FlexiInputFallSensors does not fit into payload");
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInputFallSensors) == 31, "FlexiIntputFallSensors is not 31 bytes");

    struct FlexiOutput{
        /*0.0*/ uint8_t laser_case:5;
        /*0.5*/ uint8_t far_front:1;
        /*0.6*/ uint8_t far_left:1;
        /*0.7*/ uint8_t far_right:1;

        /*1.0 to 1.1*/ uint8_t enable_components:2; // base: bit 0, torso: bit 1
        /*1.2*/ uint8_t:1;
        /*1.3*/ uint8_t:1;
        /*1.4*/ uint8_t:1;
        /*1.5*/ uint8_t:1;
        /*1.6*/ uint8_t:1;
        /*1.7*/ uint8_t:1;

        /*2.0*/ uint8_t digital_out_0:1;
        /*2.1*/ uint8_t digital_out_1:1;
        /*2.2*/ uint8_t digital_out_2:1;
        /*2.3*/ uint8_t digital_out_3:1;
        /*2.4*/ uint8_t:1;
        /*2.5*/ uint8_t:1;
        /*2.6*/ uint8_t:1;
        /*2.7*/ uint8_t:1;
        }  __attribute__ ((__packed__));
    BOOST_STATIC_ASSERT_MSG(sizeof(FlexiOutput) <= flexi::FlexiOutputData::max_data_length, "FlexiOutput does not fit into payload");

    safety_controller_config config_;

    boost::scoped_ptr <flexi::FlexiClient> flexi_client_;
    FlexiInput flexi_input_;
    boost::mutex flexi_input_mutex_;
    FlexiOutput *flexi_output_p_; // direct access to protocol data
    flexi::FlexiMsg flexi_output_msg_;

    LaserFields front_fields, left_fields, right_fields;
    visualization_msgs::msg::MarkerArray laser_markers;

    uint8_t enable_components_;
    bool state_was_valid_;
    unsigned int reconnection_attempts_;

    geometry_msgs::msg::Polygon footprint_angular_slow_;
    geometry_msgs::msg::Polygon footprint_angular_fast_;
    geometry_msgs::msg::Polygon footprint_linear_slow_;
    geometry_msgs::msg::Polygon footprint_linear_fast_;

    /* protected region user member variables end */

public:
    safety_controller_impl() : Node("safety_controller_node")
    {
        /* protected region user constructor on begin */
        memset(&flexi_input_, 0, sizeof(flexi_input_));

        BOOST_STATIC_ASSERT_MSG(sizeof(FlexiOutput) <= 10, "FlexiOutput does not fit into first field with 10 bytes");

        // set up output message with one field
        std::vector<uint8_t> output_fields[5] = { std::vector<uint8_t>(sizeof(FlexiOutput)), std::vector<uint8_t>(0), std::vector<uint8_t>(0), std::vector<uint8_t>(0), std::vector<uint8_t>(0) };
        flexi_output_msg_.set_output(output_fields);

        flexi_output_p_ = (FlexiOutput*) flexi_output_msg_.payload.output.data;

        flexi_output_p_->enable_components = enable_components_ = 3; // base and torso enabled

        state_was_valid_ = false;

        reconnection_attempts_ = 0;
        /* protected region user constructor end */
    }

    bool connect(safety_controller_config config)
    {
        flexi_client_.reset(new flexi::FlexiClient(std::bind(&safety_controller_impl::handle_input, this, std::placeholders::_1)));
        if (!flexi_client_->connect(config.host, config.port))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("safety_controller_impl"), "safety_controller_impl:connect(): Error connecting to host [" << config.host << "] on port [" << config.port << "]");
            return false;
        }
        flexi_client_->start_worker();

        BOOST_STATIC_ASSERT_MSG(sizeof(FlexiInput) <= 32, "FlexiInput does not fit into first field with 32 bytes");
        flexi::FlexiMsg control;
        // flexisoft should send update each 1000 ms or if set 1 has changed
        control.set_control(true, false, false,false, 1000);
        flexi_client_->send(control);

        flexi_client_->send(flexi_output_msg_); // send initial values
        return true;
    }

    void configure(safety_controller_config config)
    {
        /* protected region user configure on begin */
        this->config_ = config;
        if(!this->connect(this->config_))
                exit (EXIT_FAILURE);

        if(!front_fields.init(config.front_xml, "00_rotate"))
                exit (EXIT_FAILURE);
        if(!left_fields.init(config.left_xml, "00_rotate"))
                exit (EXIT_FAILURE);
        if(!right_fields.init(config.right_xml, "00_rotate"))
                exit (EXIT_FAILURE);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = rclcpp::Time();
        marker.id = 0;
        marker.type = config.marker_type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = config.scale_x;
        marker.scale.y = config.scale_y;
        marker.scale.z = config.scale_z;

        marker.color.a = 1.0;
        marker.color.r = 1.0;

        marker.ns = "front";
        marker.header.frame_id = "base_laser_front_link";
        laser_markers.markers.push_back(marker);

        marker.ns = "left";
        marker.header.frame_id = "base_laser_left_link";
        laser_markers.markers.push_back(marker);

        marker.ns = "right";
        marker.header.frame_id = "base_laser_right_link";
        laser_markers.markers.push_back(marker);

        marker.color.a = 1.0;
        marker.color.g = 1.0; // r=1, g=1: yellow
        marker.pose.position.z = -0.01;

        marker.ns = "front_warn";
        marker.header.frame_id = "base_laser_front_link";
        laser_markers.markers.push_back(marker);

        marker.ns = "left_warn";
        marker.header.frame_id = "base_laser_left_link";
        laser_markers.markers.push_back(marker);

        marker.ns = "right_warn";
        marker.header.frame_id = "base_laser_right_link";
        laser_markers.markers.push_back(marker);

        parseFootprint("footprint_angular_slow", footprint_angular_slow_);
        parseFootprint("footprint_angular_fast", footprint_angular_fast_);
        parseFootprint("footprint_linear_slow", footprint_linear_slow_);
        parseFootprint("footprint_linear_fast", footprint_linear_fast_);
        /* protected region user configure end */
    }

    bool update(safety_controller_data &data, safety_controller_config config)
    {
        /* protected region user update on begin */
        bool connected = true;
        FlexiInputWireless flexi_in_wireless;
        std::memcpy(&flexi_in_wireless, &flexi_input_, sizeof(flexi_input_));
        FlexiInputFallSensors flexi_in_fall_sensors;
        std::memcpy(&flexi_in_fall_sensors, &flexi_input_, sizeof(flexi_input_));

        diagnostic_updater::DiagnosticStatusWrapper wrapper;
        wrapper.name = "safety";
        wrapper.hardware_id = "none";
        wrapper.summary(wrapper.OK, "ready");

        double lin_velocity = sqrt(data.in_odometry.twist.twist.linear.y*data.in_odometry.twist.twist.linear.y + data.in_odometry.twist.twist.linear.x*data.in_odometry.twist.twist.linear.x);
        double rot_velocity = fabs(data.in_odometry.twist.twist.angular.z);

        std::string case_name = "00_rotate";

        // reset far flags
        flexi_output_p_->far_front = 0;
        flexi_output_p_->far_left = 0;
        flexi_output_p_->far_right = 0;

        rclcpp::Time now = this->now();
        rclcpp::Duration time_from_last_state_is_valid = rclcpp::Duration::from_seconds(0.0);
        if(config.check_external_state_valid)
        {
            time_from_last_state_is_valid = now - data.stamp_state_is_valid;
            if(config.state_is_valid_timeout <= 0 || time_from_last_state_is_valid < rclcpp::Duration::from_seconds(config.state_is_valid_timeout)){
                if(!data.in_state_is_valid.data){
                    wrapper.summary(wrapper.WARN, "current state is not valid");
                    if(state_was_valid_){
                        enable_components_ = flexi_output_p_->enable_components;
                        flexi_output_p_->enable_components = 0;
                    }
                }else if(enable_components_ != flexi_output_p_->enable_components){
                    wrapper.summary(wrapper.WARN, "please confirm invalid state with an e-stop press and release");
                }
                state_was_valid_ = data.in_state_is_valid.data;
            }else{
                wrapper.summary(wrapper.WARN, "state_is_valid timeout");
            }
        }

        // if odometry timed out, force field for fast rotation
        rclcpp::Duration time_from_last_odometry = now - data.in_odometry.header.stamp;
        if(data.in_odometry.header.stamp != builtin_interfaces::msg::Time() && config.odometry_timeout > 0 && time_from_last_odometry >= rclcpp::Duration::from_seconds(config.odometry_timeout)){
            wrapper.mergeSummary(wrapper.WARN, "odometry timeout");
            lin_velocity = 0.0;
            rot_velocity = config.threshold_angular_fast;
        }

        if(lin_velocity < config.threshold_linear_slow){ // does not move in linear direction
            flexi_output_p_->laser_case = 0; // case = rotate
            if(rot_velocity >= config.threshold_angular_fast){
                flexi_output_p_->far_front = 1;
                flexi_output_p_->far_left = 1;
                flexi_output_p_->far_right = 1;
                data.out_footprint = footprint_angular_fast_;
            }else{
                data.out_footprint = footprint_angular_slow_;
            }
        }else{

            double direction = atan2(data.in_odometry.twist.twist.linear.y, data.in_odometry.twist.twist.linear.x) /M_PI*180.0 + 7.5; // direction lower bound, aligned to 15 deg
            if(direction < 0) direction += 360; // enforce [0,360) range
            flexi_output_p_->laser_case = (int(direction/15) % 24) + 1; // select case  in [1,24]

            double theta = -(flexi_output_p_->laser_case-1) * M_PI / 12.0 /*(180.0/15)*/;

            case_name = boost::str(boost::format("%02d_drive_%d") % (int)flexi_output_p_->laser_case % (int)((flexi_output_p_->laser_case-1)*15));

            // principal directions: front  1, left  9, right  17
            if( flexi_output_p_->laser_case  < 9 || flexi_output_p_->laser_case > 17){ // drive mode for front
                flexi_output_p_->far_front = lin_velocity < config.threshold_linear_fast ? 0 : 1;
            }
            if( flexi_output_p_->laser_case  > 1 && flexi_output_p_->laser_case < 17){  // drive mode for left
                flexi_output_p_->far_left = lin_velocity < config.threshold_linear_fast ? 0 : 1;
            }
            if( flexi_output_p_->laser_case  > 9){ // drive mode for right
                flexi_output_p_->far_right = lin_velocity < config.threshold_linear_fast ? 0 : 1;
            }

            if(flexi_output_p_->far_front || flexi_output_p_->far_left || flexi_output_p_->far_right){
                rotateFootprint(footprint_linear_fast_, theta, data.out_footprint);
            }else{
                rotateFootprint(footprint_linear_slow_, theta, data.out_footprint);
            }
        }

        data.out_footprint_stamped.header.stamp = now;
        data.out_footprint_stamped.header.frame_id = "base_link";
        data.out_footprint_stamped.polygon = data.out_footprint;

        // ROS_INFO_STREAM("case_name: " << case_name << " " << (bool) flexi_output_p_->far_front<< " " << (bool) flexi_output_p_->far_left<< " " << (bool) flexi_output_p_->far_right);

        bool was_stopped = data.out_emergency_stop_state.emergency_button_stop;

        // set emergency information
        if (config.has_wireless_emstop)
        {
            if(flexi_in_wireless.wireless_bridged)
            {
                flexi_in_wireless.wireless_emstop_ok = true;
            }
            data.out_emergency_stop_state.emergency_button_stop = !flexi_in_wireless.external_stop_ok ||
                                                                  !flexi_in_wireless.wireless_emstop_ok;
            if (flexi_in_wireless.laser_bridged)
            {
                flexi_input_.laser_stop_ok = true;
            }
        }
        else if (config.has_magnetic_safety_switch)
        {
            data.out_emergency_stop_state.emergency_button_stop = !flexi_in_fall_sensors.external_stop_ok ||
                                                                    !flexi_in_fall_sensors.external_stop_2_ok;
        }
        else
        {
            data.out_emergency_stop_state.emergency_button_stop = !flexi_in_fall_sensors.external_stop_ok;
        }
        data.out_emergency_stop_state.scanner_stop = !flexi_input_.laser_stop_ok;

        bool em_state;
        em_state = !data.out_emergency_stop_state.emergency_button_stop &&
                    !data.out_emergency_stop_state.scanner_stop &&
                    flexi_input_.brake_auto &&
                    !flexi_input_.ack_needed;

        if (!config.has_wireless_emstop){
            em_state = em_state &&
                        flexi_in_fall_sensors.fall_sensor_front_ok &&
                        flexi_in_fall_sensors.fall_sensor_left_ok &&
                        flexi_in_fall_sensors.fall_sensor_right_ok &&
                        flexi_in_fall_sensors.fall_sensors_released &&
                        !flexi_in_fall_sensors.efi_bus_front_io_error &&
                        !flexi_in_fall_sensors.efi_bus_left_io_error &&
                        !flexi_in_fall_sensors.efi_bus_right_io_error;
        }

        if(em_state){
            data.out_emergency_stop_state.emergency_state = data.out_emergency_stop_state.EMFREE;
        }else{
            data.out_emergency_stop_state.emergency_state = data.out_emergency_stop_state.EMSTOP;
            wrapper.mergeSummary(wrapper.ERROR, "emergency stop");
        }

        if(was_stopped && !data.out_emergency_stop_state.emergency_button_stop){
            flexi_output_p_->enable_components = enable_components_;
        }

        if(!flexi_client_->send(flexi_output_msg_))
        {
            RCLCPP_ERROR(this->get_logger(), "safety_controller_impl:update(): Error sending msg to flexisoft. Try reconnecting...");
            this->reconnection_attempts_++;
            int reconnect_attempts = 1;
            while(reconnect_attempts < config.reconnect_attempts && !this->connect(this->config_))
            {
                reconnect_attempts++;
                this->reconnection_attempts_++;
                RCLCPP_WARN(this->get_logger(), "safety_controller_impl:update(): reconnect attempt %d failed...", reconnect_attempts);
                rclcpp::sleep_for(std::chrono::seconds(int(config.reconnect_duration)));

            }
            if(reconnect_attempts >= config.reconnect_attempts)
            {
                RCLCPP_ERROR(this->get_logger(), "safety_controller_impl:update(): could not reconnect to flexisoft. Exiting...");
                connected = false;
            }
            else
                RCLCPP_INFO(this->get_logger(), "safety_controller_impl:update(): reconnect successful");
        }

        wrapper.add("CONNECTIVITY","-----------------------");
        if(!connected){
            wrapper.mergeSummary(wrapper.ERROR, "disconnected from flexisoft");
        }
        wrapper.add("reconnection_attempts", this->reconnection_attempts_);

        wrapper.add("EM STATE","---------------------------");
        wrapper.add("ack_needed", (bool)flexi_input_.ack_needed);
        wrapper.add("button_stop_ok", (bool)flexi_input_.external_stop_ok);
        wrapper.add("brake_stop_ok", (bool)flexi_input_.brake_auto); // brake release needs to be inverted due to HW cabeling!
        wrapper.add("laser_stop_ok", (bool)flexi_input_.laser_stop_ok);

        data.out_safety_controller_state.header.stamp = now;

        data.out_safety_controller_state.has_wireless_emstop = config.has_wireless_emstop;
        data.out_safety_controller_state.has_fall_sensors = !config.has_wireless_emstop;
        data.out_safety_controller_state.has_magnetic_safety_switch = config.has_magnetic_safety_switch;

        data.out_safety_controller_state.ack_needed = (bool)flexi_input_.ack_needed;
        data.out_safety_controller_state.emergency_button_stop = !(bool)flexi_input_.external_stop_ok;
        data.out_safety_controller_state.brake_button_stop = !(bool)flexi_input_.brake_auto;
        data.out_safety_controller_state.laser_stop = !(bool)flexi_input_.laser_stop_ok;

        if(config.has_wireless_emstop)
        {
            wrapper.add("wireless_stop_ok", (bool)flexi_in_wireless.wireless_emstop_ok);
            wrapper.add("laser_not_bridged", !(bool)flexi_in_wireless.laser_bridged);
            wrapper.add("wireless_not_bridged", !(bool)flexi_in_wireless.wireless_bridged);

            if(config.warn_for_laser_bridged && flexi_in_wireless.laser_bridged){
                wrapper.mergeSummary(wrapper.WARN, "laser bridged");
            }

            if(config.warn_for_wireless_bridged && flexi_in_wireless.wireless_bridged){
                wrapper.mergeSummary(wrapper.WARN, "wireless bridged");
            }

            data.out_safety_controller_state.wireless_stop = !(bool)flexi_in_wireless.wireless_emstop_ok;
            data.out_safety_controller_state.laser_bridged = (bool)flexi_in_wireless.laser_bridged;
            data.out_safety_controller_state.wireless_bridged = (bool)flexi_in_wireless.wireless_bridged;

        }
        else
        {
            if(config.has_magnetic_safety_switch)
            {
                wrapper.add("external_stop_2_ok", (bool)flexi_in_fall_sensors.external_stop_2_ok);
                wrapper.add("magnetic_safety_switch", (bool)flexi_in_fall_sensors.magnetic_safety_switch);
                if(!flexi_in_fall_sensors.magnetic_safety_switch)
                    wrapper.mergeSummary(wrapper.WARN, "magnetic safety switch not closed");

                data.out_safety_controller_state.external_stop = !(bool)flexi_in_fall_sensors.external_stop_2_ok;
                data.out_safety_controller_state.magnetic_safety_switch = (bool)flexi_in_fall_sensors.magnetic_safety_switch;
            }
            wrapper.add("fall_sensor_front_ok", (bool)flexi_in_fall_sensors.fall_sensor_front_ok);
            wrapper.add("fall_sensor_left_ok", (bool)flexi_in_fall_sensors.fall_sensor_left_ok);
            wrapper.add("fall_sensor_right_ok", (bool)flexi_in_fall_sensors.fall_sensor_right_ok);
            wrapper.add("fall_sensors_released", (bool)flexi_in_fall_sensors.fall_sensors_released);

            data.out_safety_controller_state.fall_sensor_front = !(bool)flexi_in_fall_sensors.fall_sensor_front_ok;
            data.out_safety_controller_state.fall_sensor_left = !(bool)flexi_in_fall_sensors.fall_sensor_left_ok;
            data.out_safety_controller_state.fall_sensor_right = !(bool)flexi_in_fall_sensors.fall_sensor_right_ok;
            data.out_safety_controller_state.fall_sensor_released = (bool)flexi_in_fall_sensors.fall_sensors_released;
        }

        wrapper.add("ACTUATORS","--------------------------");
        wrapper.add("base_active", (bool)flexi_input_.base_active);
        wrapper.add("torso_active", (bool)flexi_input_.torso_active);
        wrapper.add("base_enabled", (bool)(flexi_output_p_->enable_components & 1));
        wrapper.add("torso_enabled", (bool)(flexi_output_p_->enable_components & 2));

        if((flexi_output_p_->enable_components & 1) == 0){
            wrapper.mergeSummary(wrapper.ERROR, "base disabled");
        }
        if((flexi_output_p_->enable_components & 2) == 0){
            wrapper.mergeSummary(wrapper.ERROR, "torso disabled");
        }

        data.out_safety_controller_state.base_active = (bool)flexi_input_.base_active;
        data.out_safety_controller_state.torso_active = (bool)flexi_input_.torso_active;
        data.out_safety_controller_state.base_enabled = (bool)(flexi_output_p_->enable_components & 1);
        data.out_safety_controller_state.torso_enabled = (bool)(flexi_output_p_->enable_components & 2);

        wrapper.add("ODOMETRY","---------------------------");
        wrapper.add("odometry_time_stamp", data.in_odometry.header.stamp.sec);  // replace data.in_odometry.header.stamp.toSec() 
        wrapper.add("time_since_last_odometry", time_from_last_odometry.seconds());
        wrapper.add("time_since_last_state_is_valid", time_from_last_state_is_valid.seconds());

        wrapper.add("LASERSCANNER","-----------------------");
        bool laser_front_ok = (flexi_output_p_->far_front == 0 || flexi_input_.front_2_free)  && flexi_input_.front_1_free && !flexi_input_.laser_1_dirty;
        bool laser_left_ok = (flexi_output_p_->far_left == 0 || flexi_input_.left_2_free)  && flexi_input_.left_1_free && !flexi_input_.laser_2_dirty;
        bool laser_right_ok = (flexi_output_p_->far_right == 0 || flexi_input_.right_2_free)  && flexi_input_.right_1_free && !flexi_input_.laser_3_dirty;
        if(!config.has_wireless_emstop) {
            laser_front_ok = laser_front_ok && !flexi_in_fall_sensors.efi_bus_front_io_error;
            laser_left_ok = laser_left_ok && !flexi_in_fall_sensors.efi_bus_left_io_error;
            laser_right_ok = laser_right_ok && !flexi_in_fall_sensors.efi_bus_right_io_error;
        }

        wrapper.add("laser_front_ok", laser_front_ok);
        wrapper.add("laser_left_ok", laser_left_ok);
        wrapper.add("laser_right_ok", laser_right_ok);

        data.out_safety_controller_state.laser_front_ok = laser_front_ok;
        data.out_safety_controller_state.laser_left_ok  = laser_left_ok;
        data.out_safety_controller_state.laser_right_ok = laser_right_ok;

        wrapper.add("front_1_free", (bool)flexi_input_.front_1_free);
        wrapper.add("front_2_free", (bool)flexi_input_.front_2_free);
        wrapper.add("front_3_free", (bool)flexi_input_.front_3_free);

        wrapper.add("left_1_free", (bool)flexi_input_.left_1_free);
        wrapper.add("left_2_free", (bool)flexi_input_.left_2_free);
        wrapper.add("left_3_free", (bool)flexi_input_.left_3_free);

        wrapper.add("right_1_free", (bool)flexi_input_.right_1_free);
        wrapper.add("right_2_free", (bool)flexi_input_.right_2_free);
        wrapper.add("right_3_free", (bool)flexi_input_.right_3_free);

        wrapper.add("laser_case", (int)flexi_output_p_->laser_case);
        wrapper.add("far_front", (int)flexi_output_p_->far_front);
        wrapper.add("far_left", (int)flexi_output_p_->far_left);
        wrapper.add("far_right", (int)flexi_output_p_->far_right);

        if((bool)flexi_input_.laser_1_dirty)
            wrapper.mergeSummary(wrapper.WARN, "laser front dirty");
        if((bool)flexi_input_.laser_2_dirty)
            wrapper.mergeSummary(wrapper.WARN, "laser left dirty");
        if((bool)flexi_input_.laser_3_dirty)
            wrapper.mergeSummary(wrapper.WARN, "laser right dirty");
        wrapper.add("laser_front_dirty", (bool)flexi_input_.laser_1_dirty);
        wrapper.add("laser_left_dirty", (bool)flexi_input_.laser_2_dirty);
        wrapper.add("laser_right_dirty", (bool)flexi_input_.laser_3_dirty);

        if(!config.has_wireless_emstop) {
            if((bool)flexi_in_fall_sensors.efi_bus_front_io_error)
                wrapper.mergeSummary(wrapper.WARN, "efi_bus_front_io_error");
            if((bool)flexi_in_fall_sensors.efi_bus_left_io_error)
                wrapper.mergeSummary(wrapper.WARN, "efi_bus_left_io_error");
            if((bool)flexi_in_fall_sensors.efi_bus_right_io_error)
                wrapper.mergeSummary(wrapper.WARN, "efi_bus_right_io_error");
            wrapper.add("efi_bus_front_io_error", (bool)flexi_in_fall_sensors.efi_bus_front_io_error);
            wrapper.add("efi_bus_left_io_error", (bool)flexi_in_fall_sensors.efi_bus_left_io_error);
            wrapper.add("efi_bus_right_io_error", (bool)flexi_in_fall_sensors.efi_bus_right_io_error);
        }

        //laser visualization
        data.out_marker = laser_markers;
        front_fields.fill_marker(data.out_marker.markers[0], case_name, flexi_output_p_->far_front ? "ftWarn" : "ftProtect");
        left_fields.fill_marker(data.out_marker.markers[1], case_name, flexi_output_p_->far_left ? "ftWarn" : "ftProtect");
        right_fields.fill_marker(data.out_marker.markers[2], case_name, flexi_output_p_->far_right ? "ftWarn" : "ftProtect");

        data.out_marker.markers[0].color.b = laser_front_ok ? 0.0: 1.0;
        data.out_marker.markers[1].color.b = laser_left_ok ? 0.0: 1.0;
        data.out_marker.markers[2].color.b = laser_right_ok ? 0.0: 1.0;

        if(flexi_output_p_->laser_case == 0){
            front_fields.fill_marker(data.out_marker.markers[3], case_name, "ftWarn2");
            left_fields.fill_marker(data.out_marker.markers[4], case_name, "ftWarn2");
            right_fields.fill_marker(data.out_marker.markers[5], case_name, "ftWarn2");
        }else{
            front_fields.fill_marker(data.out_marker.markers[3], case_name, flexi_output_p_->far_front ? "ftWarn2" : "ftWarn");
            left_fields.fill_marker(data.out_marker.markers[4], case_name, flexi_output_p_->far_left ? "ftWarn2" : "ftWarn");
            right_fields.fill_marker(data.out_marker.markers[5], case_name, flexi_output_p_->far_right ? "ftWarn2" : "ftWarn");
        }

        data.out_diagnostics.status.clear();
        data.out_diagnostics.header.stamp = now;
        data.out_diagnostics.status.push_back(wrapper);
        return connected;
        /* protected region user update end */
    }

    bool callback_set_mode(cob_srvs::srv::SetInt::Request &req, cob_srvs::srv::SetInt::Response &res , safety_controller_config config)
    {
        /* protected region user implementation of service callback for set_mode on begin */
        switch (req.data){
        case 0:
        case 1:
        case 2:
        case 3:
            enable_components_ = flexi_output_p_->enable_components = req.data;
            res.success = true;
            break;
        default:
            res.success = false;
        }
        /* protected region user implementation of service callback for set_mode end */
        return true;
    }

    /* protected region user additional functions on begin */
    void handle_input(const flexi::FlexiInputData &input)
    {
        boost::mutex::scoped_lock lock(flexi_input_mutex_);
        flexi_input_ = *(FlexiInput*)input.data;
    }
    /* protected region user additional functions end */
};
#endif
