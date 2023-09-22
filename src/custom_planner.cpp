// includes for libraries
#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

// includes for files
#include "local_planner/custom_planner.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

// loading namespaces
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace local_planner
{

    /*
    Cleans up resources used by the plugin
    */
    void CustomPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        auto node = parent.lock();
        node_ = parent;
        if (!node)
        {
            throw nav2_core::PlannerException("Unable to lock node!");
        }

        laser_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan",
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            std::bind(&CustomPlanner::incomingRanges, this, std::placeholders::_1));

        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        declare_parameter_if_not_declared(
            node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lookahead_dist",
            rclcpp::ParameterValue(0.4));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".single_local_plan_length", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".distance_global_score_factor", rclcpp::ParameterValue(1.0));

        node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
        node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
        double transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
        node->get_parameter(plugin_name_ + ".single_local_plan_length", step_length);
        node->get_parameter(plugin_name_ + ".distance_global_score_factor", distance_to_global_factor);

        global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(
            logger_,
            "Cleaning up the Local Planner plugin");
        global_pub_.reset();
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(
            logger_,
            "Activating Local Planner plugin");
        global_pub_->on_activate();
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(
            logger_,
            "Deactivating Local Planner plugin");
        global_pub_->on_deactivate();
    }
    /*
         Returns cmd_vel, computes the local path using whatever algorithm is being utiized
         */
    geometry_msgs::msg::TwistStamped CustomPlanner::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &,
        nav2_core::GoalChecker *)
    {

        auto transformed_plan = transformGlobalPlan(pose);


        auto goal_pose_it = std::prev(transformed_plan.poses.end());

        auto goal_pose = goal_pose_it->pose;

        double linear_vel, angular_vel;
        linear_vel = 1.0;
        angular_vel = 0.0;

        // first check if pose is in front
        if (goal_pose.position.x > 0)
        {
            // then check if it's on the left side or in front
            if (goal_pose.position.y >= 0)
            {
                linear_vel = 1.0;
                angular_vel = 0.3;
                //std::cout << "GOAL DETECTED FRONT LEFT" << std::endl;

            } // if not check if it's on right side
            else
            {
                linear_vel = 1.0;
                angular_vel = -0.3;
                //std::cout << "GOAL DETECTED FRONT RIGHT" << std::endl;
            }
            // if it's behind the robot
        }
        else
        {
            // if it's on the left side or behind
            if (goal_pose.position.y >= 0)
            {
                linear_vel = 0.3;
                angular_vel = 1.0;
                //std::cout << "GOAL DETECTED BACK LEFT" << std::endl;

                // if it's on the right side or behind
            }
            else if (goal_pose.position.y < 0)
            {
                linear_vel = 0.3;
                angular_vel = -1.0;
                //std::cout << "GOAL DETECTED BACK RIGHT" << std::endl;
            }
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose.header.frame_id;
        // cmd_vel.header.stamp = clock->now();

        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    void
    CustomPlanner::setSpeedLimit(
        const double &speed_limit,
        const bool &percentage)
    {

        if (percentage)
        {
            // Speed limit is expressed in % from maximum speed of robot
            desired_linear_vel_ = 2.0 * speed_limit / 100.0;
        }
        else
        {
            // Speed limit is expressed in absolute value
            desired_linear_vel_ = speed_limit;
        }
    }

    void CustomPlanner::setPlan(const nav_msgs::msg::Path &path)
    {
        global_pub_->publish(path);
        global_plan_ = path;
    }

    nav_msgs::msg::Path local_planner::CustomPlanner::transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose)
    {
        if (global_plan_.poses.empty())
        {
            throw nav2_core::PlannerException("Received plan with zero length");
        }

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!transformPose(
                tf_, global_plan_.header.frame_id, pose,
                robot_pose, transform_tolerance_))
        {
            throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
        }

        // We'll discard points on the plan that are outside the local costmap
        nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
                                costmap->getResolution() / 2.0;

        // First find the closest pose on the path to the robot
        auto transformation_begin =
            nav2_util::geometry_utils::min_by(
                global_plan_.poses.begin(), global_plan_.poses.end(),
                [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
                {
                    return euclidean_distance(robot_pose, ps);
                });

        // From the closest point, look for the first point that's further then dist_threshold from the
        // robot. These points are definitely outside of the costmap so we won't transform them.
        auto transformation_end = std::find_if(
            transformation_begin, end(global_plan_.poses),
            [&](const auto &global_plan_pose)
            {
                return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
            });

        // Helper function for the transform below. Transforms a PoseStamped from global frame to local
        auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
        {
            // We took a copy of the pose, let's lookup the transform at the current time
            geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
            stamped_pose.header.frame_id = global_plan_.header.frame_id;
            stamped_pose.header.stamp = pose.header.stamp;
            stamped_pose.pose = global_plan_pose.pose;
            transformPose(
                tf_, costmap_ros_->getBaseFrameID(),
                stamped_pose, transformed_pose, transform_tolerance_);
            return transformed_pose;
        };

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan;
        std::transform(
            transformation_begin, transformation_end,
            std::back_inserter(transformed_plan.poses),
            transformGlobalPoseToLocal);
        transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
        transformed_plan.header.stamp = pose.header.stamp;

        // Remove the portion of the global plan that we've already passed so we don't
        // process it on the next iteration (this is called path pruning)
        global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
        global_pub_->publish(transformed_plan);

        if (transformed_plan.poses.empty())
        {
            throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
        }

        return transformed_plan;
    }

    void CustomPlanner::incomingRanges(const sensor_msgs::msg::LaserScan &msg)
    {
        for (int i = 0; i < NUMBER_OF_LIDAR_READINGS; i++)
        {

            this->ranges[i] = msg.ranges[i];

            // std::cout << this->ranges[i] << std::endl;

            // RCLCPP_INFO(this->get_logger(), "Lidar range message: '%f'", msg.ranges[i]);
        }
    }

    bool CustomPlanner::transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string frame,
        const geometry_msgs::msg::PoseStamped &in_pose,
        geometry_msgs::msg::PoseStamped &out_pose,
        const rclcpp::Duration &transform_tolerance) const
    {
        // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

        if (in_pose.header.frame_id == frame)
        {
            out_pose = in_pose;
            return true;
        }

        try
        {
            tf->transform(in_pose, out_pose, frame);
            return true;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            auto transform = tf->lookupTransform(
                frame,
                in_pose.header.frame_id,
                tf2::TimePointZero);
            if (
                (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
                transform_tolerance)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("tf_help"),
                    "Transform data too old when converting from %s to %s",
                    in_pose.header.frame_id.c_str(),
                    frame.c_str());
                RCLCPP_ERROR(
                    rclcpp::get_logger("tf_help"),
                    "Data time: %ds %uns, Transform time: %ds %uns",
                    in_pose.header.stamp.sec,
                    in_pose.header.stamp.nanosec,
                    transform.header.stamp.sec,
                    transform.header.stamp.nanosec);
                return false;
            }
            else
            {
                tf2::doTransform(in_pose, out_pose, transform);
                return true;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Exception in transformPose: %s",
                ex.what());
            return false;
        }
        return false;
    }

} // namespace local_planner

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
    local_planner::CustomPlanner,
    nav2_core::Controller)
