#include <rclcpp/rclcpp.hpp>

#include <cob_msgs/cob_msgs/msg/emergency_stop_state.hpp>
#include <cob_msgs/cob_msgs/msg/safety_controller_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
// #include <dynamic_reconfigure/server.h>
// #include <cob_safety_controller/safety_controllerConfig.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cob_srvs/cob_srvs/srv/set_int.hpp>
// #include <cob_srvs/srv/set_int.hpp>

//#include <cob_safety_controller/safety_controller_common.h>
#include "../../common/include/cob_safety_controller/safety_controller_common.h"

class safety_controller_ros : public rclcpp::Node
{
public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_;
    rclcpp::Publisher<cob_msgs::msg::EmergencyStopState>::SharedPtr emergency_stop_state_;
    rclcpp::Publisher<cob_msgs::msg::SafetyControllerState>::SharedPtr safety_controller_state_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_stamped_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_is_valid_;
    rclcpp::Service<cob_srvs::srv::SetInt>::SharedPtr set_mode_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr server;

    safety_controller_data component_data_;
    safety_controller_config component_config_;
    safety_controller_impl component_implementation_;
    
    safety_controller_ros() : Node("safety_controller")
    {

        // Callback function setup
        auto callback = [this](const std::vector<rclcpp::Parameter>& parameters, rcl_interfaces::msg::ParameterEvent event) {
            this->configure_callback(parameters);
        };
        server = this->add_on_set_parameters_callback(std::bind(&safety_controller_ros::configure_callback, this, std::placeholders::_1));
        
        
        // Advertise services
        auto set_mode_remap = this->get_parameter("set_mode_remap").as_string();
        set_mode_ = this->create_service<cob_srvs::srv::SetInt>(set_mode_remap, std::bind(&safety_controller_impl::callback_set_mode, &component_implementation_, std::placeholders::_1, std::placeholders::_2, component_config_));
        
        // Advertise publishers
        marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 1);
        emergency_stop_state_ = this->create_publisher<cob_msgs::msg::EmergencyStopState>("emergency_stop_state", 1);
        safety_controller_state_ = this->create_publisher<cob_msgs::msg::SafetyControllerState>("safety_controller_state", 1);
        diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
        footprint_ = this->create_publisher<geometry_msgs::msg::Polygon>("footprint", 1);
        footprint_stamped_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("footprint_stamped", 1);

        // Subscribe to topics
        odometry_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", 1, std::bind(&safety_controller_ros::topicCallback_odometry, this, std::placeholders::_1));

        // Retrieve parameters
        component_config_.port = this->declare_parameter("port", "empty");
        component_config_.host = this->declare_parameter("host", "empty");
        component_config_.threshold_linear_slow = this->declare_parameter("threshold_linear_slow", 0.0);
        component_config_.threshold_linear_fast = this->declare_parameter("threshold_linear_fast", 0.0);
        component_config_.threshold_angular_fast = this->declare_parameter("threshold_angular_fast", 0.0);
        component_config_.front_xml = this->get_parameter("front_xml").get_value<std::string>();
        component_config_.left_xml = this->get_parameter("left_xml").get_value<std::string>();
        component_config_.right_xml = this->get_parameter("right_xml").get_value<std::string>();
        component_config_.marker_type = this->declare_parameter("marker_type", 0);
        component_config_.scale_x = this->declare_parameter("scale_x", 0.0);
        component_config_.scale_y = this->declare_parameter("scale_y", 0.0);
        component_config_.scale_z = this->declare_parameter("scale_z", 0.0);
        component_config_.odometry_timeout = this->declare_parameter("odometry_timeout", 1.0);
        component_config_.state_is_valid_timeout = this->declare_parameter("state_is_valid_timeout", 1.0);
        component_config_.check_external_state_valid = this->declare_parameter("check_external_state_valid", false);
        component_config_.has_wireless_emstop = this->declare_parameter("has_wireless_emstop", true);
        component_config_.warn_for_laser_bridged = this->declare_parameter("warn_for_laser_bridged", true);
        component_config_.warn_for_wireless_bridged = this->declare_parameter("warn_for_wireless_bridged", true);
        component_config_.reconnect_attempts = this->declare_parameter("reconnect_attempts", 5);
        component_config_.reconnect_duration = this->declare_parameter("reconnect_duration", 5.0);
        component_config_.has_magnetic_safety_switch = this->declare_parameter("has_magnetic_safety_switch", false);

        if (component_config_.check_external_state_valid)
            state_is_valid_ = this->create_subscription<std_msgs::msg::Bool>("state_is_valid", 1, std::bind(&safety_controller_ros::topicCallback_state_is_valid, this, std::placeholders::_1));
    }


    void topicCallback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        component_data_.in_odometry = *msg;
    }

    void topicCallback_state_is_valid(const std_msgs::msg::Bool::SharedPtr msg)
    {
        component_data_.in_state_is_valid = *msg;
        component_data_.stamp_state_is_valid = now();
    }

    // Parameter event callback
    rcl_interfaces::msg::SetParametersResult configure_callback(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : parameters)
        {
            if (param.get_name() == "port")
                component_config_.port = param.get_value<std::string>();
            else if (param.get_name() == "host")
                component_config_.host = param.get_value<std::string>();
            else if (param.get_name() == "threshold_linear_slow")
                component_config_.threshold_linear_slow = param.get_value<double>();
            else if (param.get_name() == "threshold_linear_fast")
                component_config_.threshold_linear_fast = param.get_value<double>();
            else if (param.get_name() == "threshold_angular_fast")
                component_config_.threshold_angular_fast = param.get_value<double>();
            else if (param.get_name() == "front_xml")
                component_config_.front_xml = param.get_value<std::string>();
            else if (param.get_name() == "left_xml")
                component_config_.left_xml = param.get_value<std::string>();
            else if (param.get_name() == "right_xml")
                component_config_.right_xml = param.get_value<std::string>();
            else if (param.get_name() == "marker_type")
                component_config_.marker_type = param.get_value<int>();
            else if (param.get_name() == "scale_x")
                component_config_.scale_x = param.get_value<double>();
            else if (param.get_name() == "scale_y")
                component_config_.scale_y = param.get_value<double>();
            else if (param.get_name() == "scale_z")
                component_config_.scale_z = param.get_value<double>();
            else if (param.get_name() == "odometry_timeout")
                component_config_.odometry_timeout = param.get_value<double>();
            else if (param.get_name() == "state_is_valid_timeout")
                component_config_.state_is_valid_timeout = param.get_value<double>();
        }
        this->configure();

        return result;
    }

    // Configure component with current parameters
    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_marker_active = true;
        component_data_.out_emergency_stop_state_active = true;
        component_data_.out_safety_controller_state_active = true;
        component_data_.out_diagnostics_active = true;
        component_data_.out_footprint_active = true;
        component_data_.out_footprint_stamped_active = true;
    }

    void update()
    {
        static bool keep_running = true;
        activate_all_output();

        if (component_data_.out_diagnostics_active)
            diagnostics_->publish(component_data_.out_diagnostics);

        if (keep_running)
        {
            keep_running = component_implementation_.update(component_data_, component_config_);
            if (component_data_.out_marker_active)
                marker_->publish(component_data_.out_marker);
            if (component_data_.out_emergency_stop_state_active)
                emergency_stop_state_->publish(component_data_.out_emergency_stop_state);
            if (component_data_.out_safety_controller_state_active)
                safety_controller_state_->publish(component_data_.out_safety_controller_state);
            if (component_data_.out_footprint_active)
                footprint_->publish(component_data_.out_footprint);
            if (component_data_.out_footprint_stamped_active)
                footprint_stamped_->publish(component_data_.out_footprint_stamped);
        }
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<safety_controller_ros>();
    node->configure();

    rclcpp::Rate loop_rate(10.0);
    while (rclcpp::ok())
    {
        node->update();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

