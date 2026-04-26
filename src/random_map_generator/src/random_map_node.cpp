#include <chrono>
#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using std::vector;
using std::default_random_engine;
using std::uniform_real_distribution;

class RandomMapNode : public rclcpp::Node
{
public:
    RandomMapNode() : rclcpp::Node("random_map_node")
    {
        int case_id = 0;
        obs_num      = declareOrGet<std::vector<int64_t>>("map.obs_num", std::vector<int64_t>{1, 1, 1});
        resolution   = declareOrGet<double>("map.resolution",    resolution);
        fix_generator= declareOrGet<bool>  ("map.fix_generator", fix_generator);
        size_x       = declareOrGet<double>("map.size_x",        size_x);
        size_y       = declareOrGet<double>("map.size_y",        size_y);
        min_width    = declareOrGet<double>("map.min_width",     min_width);
        max_width    = declareOrGet<double>("map.max_width",     max_width);
        min_obs_dis  = declareOrGet<double>("map.min_obs_dis",   min_obs_dis);
        vis_rate     = declareOrGet<double>("map.vis_rate",      vis_rate);
        sensor_rate  = declareOrGet<double>("map.sensor_rate",   sensor_rate);
        sensor_range = declareOrGet<double>("map.sensor_range",  sensor_range);
        height       = declareOrGet<double>("map.height",        height);
        case_id      = declareOrGet<int>   ("map.case_id",       0);

        if (!fix_generator)
        {
            std::random_device rd;
            eng = default_random_engine(rd());
        }
        else
            eng = default_random_engine(0);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(1000),
            std::bind(&RandomMapNode::rcvOdomCallBack, this, std::placeholders::_1));
        local_map_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_cloud", 1);
        global_map_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_cloud", 1);
        mesh_map_pub_    = this->create_publisher<visualization_msgs::msg::Marker>("mesh_obstacles", 1);
        polygon_map_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("global_polygon", 1);

        switch (case_id)
        {
            case 0: generateRandomCase();   break;
            default: generateRandomCase();  break;
        }

        kd_tree.setInputCloud(cloud_map.makeShared());

        auto vis_period = std::chrono::duration<double>(1.0 / vis_rate);
        vis_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(vis_period),
            std::bind(&RandomMapNode::visCallback, this));
        auto sensor_period = std::chrono::duration<double>(1.0 / sensor_rate);
        sensor_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_period),
            std::bind(&RandomMapNode::sensorCallback, this));
    }

private:
    template <typename T>
    T declareOrGet(const std::string& name, const T& default_value)
    {
        if (!this->has_parameter(name))
            this->declare_parameter<T>(name, default_value);
        T value = default_value;
        this->get_parameter(name, value);
        return value;
    }

    static bool crossBoolean2(Eigen::Vector2d a, Eigen::Vector2d b)
    {
        return (a(0) * b(1) - b(0) * a(1) > 0);
    }

    pcl::PointCloud<pcl::PointXYZ> fillConvexPolygon(vector<Eigen::Vector2d> poly_vs)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

        if (poly_vs.size() < 3)
            return cloud_polygon;

        double down = 9999.0;
        double up = -9999.0;
        double left = 9999.0;
        double right = -9999.0;

        for (size_t i = 0; i < poly_vs.size(); i++)
        {
            if (poly_vs[i][0] > right) right = poly_vs[i][0];
            if (poly_vs[i][0] < left)  left  = poly_vs[i][0];
            if (poly_vs[i][1] > up)    up    = poly_vs[i][1];
            if (poly_vs[i][1] < down)  down  = poly_vs[i][1];
        }

        double max_h = rand_h(eng);
        for (double x = left; x < right + resolution; x += resolution)
        {
            for (double y = down; y < up + resolution; y += resolution)
            {
                bool in_poly = false;
                Eigen::Vector2d O(x, y);

                for (size_t i = 0; i < poly_vs.size() - 2; i++)
                {
                    Eigen::Vector2d A = poly_vs[0];
                    Eigen::Vector2d B = poly_vs[i + 1];
                    Eigen::Vector2d C = poly_vs[i + 2];
                    if (crossBoolean2(B - A, O - A) &&
                        crossBoolean2(C - B, O - B) &&
                        crossBoolean2(A - C, O - C))
                    {
                        in_poly = true;
                        break;
                    }
                }

                if (in_poly)
                {
                    pcl::PointXYZ pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = 0.0;
                    for (double h = 0.0; h <= max_h; h += resolution)
                    {
                        pt.z = h;
                        cloud_polygon.push_back(pt);
                    }
                }
            }
        }

        return cloud_polygon;
    }

    std::pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> generatePolygon(int K)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

        rand_w = uniform_real_distribution<double>(min_width, max_width);
        rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

        double radius = rand_w(eng);
        double theta = rand_theta(eng);
        double angle_res = 2.0 * M_PI / K;
        double small_r = radius * sin(angle_res / 2.0);

        rand_radius = uniform_real_distribution<double>(-small_r, small_r);

        vector<Eigen::Vector2d> vs;
        for (int i = 0; i < K; i++)
        {
            double a = angle_res * i + theta;
            double delta_theta = rand_theta(eng);
            double delta_radius = rand_radius(eng);
            Eigen::Vector2d p(cos(a) * radius + cos(a + delta_theta) * delta_radius,
                              sin(a) * radius + sin(a + delta_theta) * delta_radius);
            vs.push_back(p);
        }
        cloud_polygon = fillConvexPolygon(vs);

        return std::make_pair(vs, cloud_polygon);
    }

    void generateRandomCase()
    {
        pcl::PointXYZ pt_random;

        rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
        rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);
        rand_h = uniform_real_distribution<double>(height / 2.0, height);

        centers.clear();
        for (size_t k = 0; k < obs_num.size(); k++)
        {
            for (int j = 0; j < obs_num[k]; j++)
            {
                double x = rand_x(eng);
                double y = rand_y(eng);

                x = floor(x / resolution) * resolution + resolution / 2.0;
                y = floor(y / resolution) * resolution + resolution / 2.0;

                bool collision = false;
                for (size_t i = 0; i < centers.size(); i++)
                {
                    if ((Eigen::Vector2d(x, y) - centers[i]).squaredNorm() < min_obs_dis * min_obs_dis)
                    {
                        collision = true;
                        break;
                    }
                }
                if (collision)
                {
                    j--;
                    continue;
                }
                else
                {
                    centers.push_back(Eigen::Vector2d(x, y));
                }

                auto cloud_polygon = generatePolygon(k + 3);
                for (size_t i = 0; i < cloud_polygon.second.points.size(); i++)
                {
                    pt_random.x = cloud_polygon.second.points[i].x + x;
                    pt_random.y = cloud_polygon.second.points[i].y + y;
                    pt_random.z = cloud_polygon.second.points[i].z;
                    cloud_map.points.push_back(pt_random);
                }

                vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
                geometry_msgs::msg::Point init_p;
                init_p.x = vector_polygon[0].x() + x;
                init_p.y = vector_polygon[0].y() + y;
                init_p.z = 0.0;
                polygon_msg.data.push_back(k + 3);
                polygon_msg.data.push_back(init_p.x);
                polygon_msg.data.push_back(init_p.y);
                for (size_t i = 1; i < k + 2; i++)
                {
                    mesh_msg.points.push_back(init_p);
                    geometry_msgs::msg::Point p;
                    p.x = vector_polygon[i].x() + x;
                    p.y = vector_polygon[i].y() + y;
                    p.z = 0.0;
                    mesh_msg.points.push_back(p);
                    polygon_msg.data.push_back(p.x);
                    polygon_msg.data.push_back(p.y);
                    p.x = vector_polygon[i + 1].x() + x;
                    p.y = vector_polygon[i + 1].y() + y;
                    p.z = 0.0;
                    mesh_msg.points.push_back(p);
                }
                polygon_msg.data.push_back(vector_polygon.back().x() + x);
                polygon_msg.data.push_back(vector_polygon.back().y() + y);
            }
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMapPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud_map));
        vg.setInputCloud(cloudMapPtr);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        vg.filter(cloud_map);

        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        cloud_map.is_dense = true;
        has_map = true;

        pcl::toROSMsg(cloud_map, global_msg);
        global_msg.header.frame_id = "world";

        mesh_msg.id = 0;
        mesh_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        mesh_msg.action = visualization_msgs::msg::Marker::ADD;
        mesh_msg.pose.orientation.w = 1.0;
        mesh_msg.scale.x = 1.0;
        mesh_msg.scale.y = 1.0;
        mesh_msg.scale.z = 1.0;
        mesh_msg.color.r = 0.2;
        mesh_msg.color.g = 0.2;
        mesh_msg.color.b = 0.2;
        mesh_msg.color.a = 0.3;
        mesh_msg.header.frame_id = "world";
    }

    void visCallback()
    {
        if (!has_map)
            return;
        global_map_pub_->publish(global_msg);
        mesh_map_pub_->publish(mesh_msg);
        polygon_map_pub_->publish(polygon_msg);
    }

    void rcvOdomCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        sensor_pose.x = msg->pose.pose.position.x;
        sensor_pose.y = msg->pose.pose.position.y;
        sensor_pose.z = 0.0;
        has_odom = true;
    }

    void sensorCallback()
    {
        if (!has_map || !has_odom)
            return;

        pcl::PointCloud<pcl::PointXYZ> local_map;

        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        idx_map.setConstant(-1);
        dis_map.setConstant(9999.0);

        pcl::PointXYZ pt;
        if (kd_tree.radiusSearch(sensor_pose, sensor_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
            {
                pt = cloud_map.points[pointIdxRadiusSearch[i]];
                int idx = floor((atan2(pt.y - sensor_pose.y, pt.x - sensor_pose.x) + M_PI + laser_res / 2.0) / laser_res);
                if (idx >= 0 && idx < LINE_NUM && dis_map[idx] > pointRadiusSquaredDistance[i])
                {
                    idx_map[idx] = idx;
                    dis_map[idx] = pointRadiusSquaredDistance[i];
                }
            }

            for (int i = 0; i < LINE_NUM; i++)
            {
                if (idx_map[i] != -1)
                {
                    double angle = idx_map[i] * laser_res - M_PI;
                    double dist = sqrt(dis_map[i]);
                    pt.x = dist * cos(angle) + sensor_pose.x;
                    pt.y = dist * sin(angle) + sensor_pose.y;
                    pt.z = 0.0;
                    local_map.push_back(pt);
                }
            }
        }

        local_map.width = local_map.points.size();
        local_map.height = 1;
        local_map.is_dense = true;

        sensor_msgs::msg::PointCloud2 local_msg;
        pcl::toROSMsg(local_map, local_msg);
        local_msg.header.frame_id = "world";
        local_map_pub_->publish(local_msg);
    }

    // pcl data
    vector<Eigen::Vector2d> centers;
    pcl::PointCloud<pcl::PointXYZ> cloud_map;
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    pcl::PointXYZ sensor_pose;
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    // random
    default_random_engine eng;
    uniform_real_distribution<double> rand_x;
    uniform_real_distribution<double> rand_y;
    uniform_real_distribution<double> rand_w;
    uniform_real_distribution<double> rand_theta;
    uniform_real_distribution<double> rand_radius;
    uniform_real_distribution<double> rand_h;

    // cached messages
    sensor_msgs::msg::PointCloud2 global_msg;
    visualization_msgs::msg::Marker mesh_msg;
    std_msgs::msg::Float64MultiArray polygon_msg;

    // params / state
    bool has_odom = false;
    bool has_map = false;
    bool fix_generator = false;
    vector<int64_t> obs_num = {1, 1, 1};
    double resolution = 0.1;
    double size_x = 30.0;
    double size_y = 30.0;
    double min_width = 0.3;
    double max_width = 0.8;
    double min_obs_dis = 0.3;
    double vis_rate = 10.0;
    double sensor_rate = 10.0;
    double sensor_range = 5.0;
    double height = 0.0;

    // laser
    static constexpr int LINE_NUM = 128;
    double laser_res = 2.0 * M_PI / LINE_NUM;
    Eigen::VectorXi idx_map = Eigen::VectorXi::Constant(LINE_NUM, -1);
    Eigen::VectorXd dis_map = Eigen::VectorXd::Constant(LINE_NUM, 9999.0);

    // ros 2
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_map_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr polygon_map_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomMapNode>());
    rclcpp::shutdown();
    return 0;
}