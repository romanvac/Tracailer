#pragma once

#include <string.h>
#include <iostream>
#include <random>
#include <time.h>
#include <eigen3/Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "planner/trailer.hpp"
#include "planner/grid_map.h"
#include "planner/hybrid_astar.h"
#include "planner/arc_opt.h"

#include "planner/msg/arc_trailer_traj.hpp"
#include "planner/msg/trailer_state.hpp"

namespace trailer_planner
{
    class Planner
    {
        private:
            bool has_odom = false;
            bool in_plan = false;

            Eigen::Vector2d odom_vw;
            Eigen::VectorXd odom_pos;
            Eigen::VectorXd start_pos;
            Eigen::VectorXd end_pos;

            // trajs
            std::vector<Eigen::VectorXd> front_path;
            ArcTraj arc_traj;

            // members
            Trailer::Ptr trailer;
            GridMap::Ptr grid_map;
            HybridAstar hybrid_astar;
            ArcOpt arc_opt;

            // ros
            rclcpp::Node* node_ptr = nullptr;
            rclcpp::Logger logger_ = rclcpp::get_logger("planner");
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr front_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr end_pub;
            rclcpp::Publisher<planner::msg::ArcTrailerTraj>::SharedPtr arc_traj_pub;
            rclcpp::Publisher<planner::msg::TrailerState>::SharedPtr trailer_set_pub;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
            rclcpp::Subscription<planner::msg::TrailerState>::SharedPtr odom_sub;

            Eigen::Vector3d poseToSE2(const geometry_msgs::msg::Pose& pose) const;
            Eigen::VectorXd initialPoseToState(const Eigen::Vector3d& pose) const;
            planner::msg::TrailerState stateToTrailerStateMsg(const Eigen::VectorXd& state) const;
            bool planToGoal(const Eigen::Vector3d& goal);
            void handleGoal(const Eigen::Vector3d& goal);

        public:
            void init(rclcpp::Node* node);
            void rcvInitialPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
            void rcvNavGoalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
            void rcvOdomCallBack(const planner::msg::TrailerState::SharedPtr msg);
            bool plan(Eigen::VectorXd start, Eigen::VectorXd end);
            void vis_front();
            void vis_end();
            void pub_end();
    };
}
