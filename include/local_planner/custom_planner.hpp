// Header file for the local planner

// mandatory preprocessor directives for the ROS2
#ifndef LOCAL_PLANNER__CUSTOM_PLANNER_HPP_
#define LOCAL_PLANNER__CUSTOM_PLANNER_HPP_

// include section for libraries
#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

// include section for files
#include "nav2_core/controller.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace local_planner
{

    class CustomPlanner : public nav2_core::Controller
    {

    public:
        // assignments copied from tutorial provided by nav2
        CustomPlanner() = default;
        ~CustomPlanner() override = default;

        // mandatory functionsto override
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                       std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        // mandatory overrides
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        // mandatory override
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &,
            nav2_core::GoalChecker *) override;

        // mandatory override
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

        bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose,
            const rclcpp::Duration &transform_tolerance) const;


        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
        rclcpp::Clock::SharedPtr clock_;

        double desired_linear_vel_;
        double lookahead_dist_;
        double max_angular_vel_;
        rclcpp::Duration transform_tolerance_{0, 0};
        float single_local_plan_length;
        double distance_global_score_factor;
        double distance_goal_score_factor;
        int local_plan_laserscan_number;
        int local_plan_projection_number;
        double vel_reduction_factor;
        double local_plan_rotation_rate_factor;
        int local_plan_rotation_rate;



        double max_angular_vel = 1.62;
        double wheel_base = 0.555;
        double robot_width = 0.67;
        double robot_length = 0.99;

        typedef std::vector<std::tuple<double, double>> velocities;

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

        // VARIABLES FOR THIS PLANNER
        float ranges[360];
        const int NUMBER_OF_LIDAR_READINGS = 360;

        void incomingRanges(const sensor_msgs::msg::LaserScan &msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    };

}

// mandatory preprocessor directive
#endif