#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "planner/trailer.hpp"
#include "planner/msg/trailer_state.hpp"

using std::vector;

class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode() : rclcpp::Node("simulator_node")
    {
        // Parameters (ROS 2 requires declaration). Keep ROS 1 defaults.
        declareOrGet<double>("sim.time_resolution", time_resolution, 0.01);
        declareOrGet<double>("sim.time_delay",      time_delay,      0.0);
        declareOrGet<double>("sim.noise_std",       noise_std,       0.0);
        declareOrGet<double>("sim.max_speed",       max_speed,       10.0);

        trailer.init(this);

        state.resize(TRAILER_NUM + 3);
        for (size_t i = 0; i < static_cast<size_t>(TRAILER_NUM + 3); i++)
            state[i] = trailer.pthetas[i];

        im_cmd.speed = 0.0;
        im_cmd.steering_angle = 0.0;

        trailer.showTrailer(state, 1);

        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        command_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "cmd", rclcpp::QoS(1000),
            std::bind(&SimulatorNode::rcvCmdCallBack, this, std::placeholders::_1));
        set_sub_ = this->create_subscription<planner::msg::TrailerState>(
            "/trailer_set", rclcpp::QoS(1000),
            std::bind(&SimulatorNode::rcvSetCallBack, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<planner::msg::TrailerState>("odom", 10);
        sensor_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sensor_odom", 10);

        auto period = std::chrono::duration<double>(time_resolution);
        simulate_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimulatorNode::simCallback, this));
    }

private:
    template <typename T>
    void declareOrGet(const std::string& name, T& out, const T& default_value)
    {
        if (!this->has_parameter(name))
            this->declare_parameter<T>(name, default_value);
        this->get_parameter(name, out);
    }

    static void normYaw(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }

    double guassRandom(double std)
    {
        return std * distribution_(generator_);
    }

    void rcvCmdCallBack(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr msg)
    {
        if (!rcv_cmd_)
        {
            rcv_cmd_ = true;
            cmd_buff_.push_back(*msg);
            get_cmdtime_ = this->now();
        }
        else
        {
            cmd_buff_.push_back(*msg);
            if ((this->now() - get_cmdtime_).seconds() > time_delay)
            {
                im_cmd = cmd_buff_[0];
                cmd_buff_.erase(cmd_buff_.begin());
            }
        }
    }

    void rcvSetCallBack(const planner::msg::TrailerState::ConstSharedPtr msg)
    {
        state[0] = msg->odoms[0].pose.pose.position.x;
        state[1] = msg->odoms[0].pose.pose.position.y;
        for (int i = 0; i < TRAILER_NUM + 1; i++)
            state[2 + i] = msg->odoms[i].pose.pose.position.z;
    }

    void simCallback()
    {
        planner::msg::TrailerState new_odom;
        nav_msgs::msg::Odometry odom_temp;
        odom_temp.header.frame_id = "world";

        double v = std::max(std::min(static_cast<double>(im_cmd.speed), max_speed), -max_speed)
                   + guassRandom(noise_std);
        double delta = std::max(std::min(static_cast<double>(im_cmd.steering_angle), trailer.max_steer),
                                -trailer.max_steer) + guassRandom(noise_std);
        Eigen::VectorXd new_state, new_se2_state, v_state, w_state;
        new_state = state;
        v_state.resize(TRAILER_NUM + 1);
        w_state.resize(TRAILER_NUM + 1);

        // state transition use vel
        double w = v * tan(delta) / trailer.wheel_base;
        double y = time_resolution * w;

        if (fabs(w) > 1e-4)
        {
            new_state(0) = state(0) + v / w * (sin(state(2) + y) - sin(state(2)));
            new_state(1) = state(1) - v / w * (cos(state(2) + y) - cos(state(2)));
            new_state(2) = state(2) + y;
            normYaw(new_state(2));
        }
        else
        {
            new_state(0) = state(0) + v * time_resolution * cos(state(2));
            new_state(1) = state(1) + v * time_resolution * sin(state(2));
            new_state(2) = state(2);
        }
        v_state[0] = v;
        w_state[0] = w;

        for (size_t i = 0; i < TRAILER_NUM; i++)
        {
            double sthetad = sin(state(i + 2) - state(i + 3));
            double cthetad = cos(state(i + 2) - state(i + 3));
            double w_temp = w;
            w = (v * sthetad - trailer.Ltail[i] * w * cthetad) / trailer.Lhead[i];
            v = v * cthetad + trailer.Ltail[i] * w_temp * sthetad;
            v_state[i + 1] = v;
            w_state[i + 1] = w;
            new_state(i + 3) = state(i + 3) + w * time_resolution;
            normYaw(new_state(i + 3));
        }

        trailer.gainSE2State(new_state, new_se2_state);
        for (size_t i = 0; i < TRAILER_NUM + 1; i++)
        {
            odom_temp.pose.pose.position.x = new_se2_state[3 * i];
            odom_temp.pose.pose.position.y = new_se2_state[3 * i + 1];
            double yy = new_se2_state[3 * i + 2];
            odom_temp.pose.pose.orientation.w = cos(yy / 2.0);
            odom_temp.pose.pose.orientation.x = 0.0;
            odom_temp.pose.pose.orientation.y = 0.0;
            odom_temp.pose.pose.orientation.z = sin(yy / 2.0);
            odom_temp.twist.twist.angular.z = w_state[i];
            odom_temp.twist.twist.linear.x = v_state[i];
            new_odom.odoms.push_back(odom_temp);
        }
        trailer.showTrailer(new_state, 1);
        new_odom.time_now = this->now();
        odom_pub_->publish(new_odom);
        state = new_state;
        sensor_odom_pub_->publish(new_odom.odoms[0]);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "robot";
        transform.transform.translation.x = new_odom.odoms[0].pose.pose.position.x;
        transform.transform.translation.y = new_odom.odoms[0].pose.pose.position.y;
        transform.transform.translation.z = new_odom.odoms[0].pose.pose.position.z;
        transform.transform.rotation = new_odom.odoms[0].pose.pose.orientation;
        broadcaster_->sendTransform(transform);
    }

    // simulator parameters
    double time_resolution = 0.01;
    double time_delay = 0.0;
    double noise_std = 0.0;
    double max_speed = 10.0;

    // simulator state
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_{0.0, 1.0};
    ackermann_msgs::msg::AckermannDrive im_cmd;
    vector<ackermann_msgs::msg::AckermannDrive> cmd_buff_;
    trailer_planner::Trailer trailer;
    Eigen::VectorXd state;
    bool rcv_cmd_ = false;
    rclcpp::Time get_cmdtime_;

    // ros 2 interface
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr command_sub_;
    rclcpp::Subscription<planner::msg::TrailerState>::SharedPtr set_sub_;
    rclcpp::Publisher<planner::msg::TrailerState>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_odom_pub_;
    rclcpp::TimerBase::SharedPtr simulate_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
