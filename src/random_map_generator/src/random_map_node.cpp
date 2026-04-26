#include <chrono>
#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

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
            case 1: generateNarrowCase();   break;
            case 2: generateComplexCase();  break;
            case 3: generateThinGapCase();  break;
            case 4: generatePointCase();    break;
            case 5: generateParkCase();     break;
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
                    cloud_polygon.push_back(pt);
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

    void finalizeMeshFlat()
    {
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

    void generateRandomCase()
    {
        pcl::PointXYZ pt_random;

        rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
        rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);

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
                    pt_random.z = 0.0;
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

        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        cloud_map.is_dense = true;
        has_map = true;

        pcl::toROSMsg(cloud_map, global_msg);
        global_msg.header.frame_id = "world";
        finalizeMeshFlat();
    }

    void generateFromRectList(const vector<double>& px, const vector<double>& py,
                              const vector<vector<Eigen::Vector2d>>& vs_list)
    {
        pcl::PointXYZ pt_random;
        for (size_t p = 0; p < px.size(); p++)
        {
            double x = px[p];
            double y = py[p];

            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;

            std::pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon;
            cloud_polygon.first = vs_list[p];
            cloud_polygon.second = fillConvexPolygon(vs_list[p]);

            for (size_t i = 0; i < cloud_polygon.second.points.size(); i++)
            {
                pt_random.x = cloud_polygon.second.points[i].x + x;
                pt_random.y = cloud_polygon.second.points[i].y + y;
                pt_random.z = 0.0;
                cloud_map.points.push_back(pt_random);
            }

            vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
            geometry_msgs::msg::Point init_p;
            init_p.x = vector_polygon[0].x() + x;
            init_p.y = vector_polygon[0].y() + y;
            init_p.z = 0.0;
            polygon_msg.data.push_back(4);
            polygon_msg.data.push_back(init_p.x);
            polygon_msg.data.push_back(init_p.y);
            for (int i = 1; i < 3; i++)
            {
                mesh_msg.points.push_back(init_p);
                geometry_msgs::msg::Point pnt;
                pnt.x = vector_polygon[i].x() + x;
                pnt.y = vector_polygon[i].y() + y;
                pnt.z = 0.0;
                mesh_msg.points.push_back(pnt);
                polygon_msg.data.push_back(pnt.x);
                polygon_msg.data.push_back(pnt.y);
                pnt.x = vector_polygon[i + 1].x() + x;
                pnt.y = vector_polygon[i + 1].y() + y;
                pnt.z = 0.0;
                mesh_msg.points.push_back(pnt);
            }
            polygon_msg.data.push_back(vector_polygon.back().x() + x);
            polygon_msg.data.push_back(vector_polygon.back().y() + y);
        }

        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        cloud_map.is_dense = true;
        has_map = true;

        pcl::toROSMsg(cloud_map, global_msg);
        global_msg.header.frame_id = "world";
        finalizeMeshFlat();
    }

    void generateNarrowCase()
    {
        vector<double> px{0.0, 0.0};
        vector<double> py{3.0, -3.0};
        double w = 2;
        vector<Eigen::Vector2d> vs{{-3, -w}, {3, -w}, {3, w}, {-3, w}};
        vector<vector<Eigen::Vector2d>> vs_list{vs, vs};
        generateFromRectList(px, py, vs_list);
    }

    void generateComplexCase()
    {
        vector<double> px{0.0, 0.0, -4.0, -6.0};
        vector<double> py{-3.0, 3.0, -3.0, 0.0};
        double w = 2.5;
        vector<Eigen::Vector2d> a1{{-3.0, -w}, {3.0, -w}, {3.0, w}, {-3.0, w}};
        vector<Eigen::Vector2d> a2{{-3.0, -w}, {3.0, -w}, {3.0, w}, {-3.0, w}};
        vector<Eigen::Vector2d> a3{{-1.0, -2.0}, {1.0, -2.0}, {1.0, 2.0}, {-1.0, 2.0}};
        vector<Eigen::Vector2d> a4{{-1.0, -5.0}, {1.0, -5.0}, {1.0, 5.0}, {-1.0, 5.0}};
        vector<vector<Eigen::Vector2d>> vs_list{a1, a2, a3, a4};
        generateFromRectList(px, py, vs_list);
    }

    void generateThinGapCase()
    {
        vector<double> px{15.0, -15.0, 0.0, 0.0};
        vector<double> py{0.0, 0.0, 15.0, -15.0};
        double w = 0.05;
        double gap_width_half = 0.4;
        vector<Eigen::Vector2d> wall1{{-0.2, -15.0}, {0.2, -15.0}, {0.2, 15.0}, {-0.2, 15.0}};
        vector<Eigen::Vector2d> wall2{{-15.0, -0.2}, {15.0, -0.2}, {15.0, 0.2}, {-15.0, 0.2}};
        vector<vector<Eigen::Vector2d>> vs_list{wall1, wall1, wall2, wall2};
        vector<Eigen::Vector2d> pgap{{10.0, -10.0},
                                     {-8.0, -6.0},
                                     {-8.0, -2.0},
                                     {2.0, 2.0},
                                     {6.0, 4.0},
                                     {-2.0, 7.0},
                                     {3.0, 10.0},
                                     {2.0, 14.0}};
        for (size_t i = 0; i < pgap.size(); i++)
        {
            double h = (pgap[i].x() + 15.0 - gap_width_half) / 2.0;
            vector<Eigen::Vector2d> gap1{{-h, -w}, {h, -w}, {h, w}, {-h, w}};
            vs_list.push_back(gap1);
            px.push_back(pgap[i].x() - h - gap_width_half);
            py.push_back(pgap[i].y());
            h = (15.0 - pgap[i].x() - gap_width_half) / 2.0;
            vector<Eigen::Vector2d> gap2{{-h, -w}, {h, -w}, {h, w}, {-h, w}};
            vs_list.push_back(gap2);
            px.push_back(pgap[i].x() + h + gap_width_half);
            py.push_back(pgap[i].y());
        }
        generateFromRectList(px, py, vs_list);
    }

    void generatePointCase()
    {
        double w = 0.01;
        double h = 3.0;
        double gap = 0.7;
        double x_cord = 2.0;
        vector<double> px{x_cord, x_cord};
        vector<double> py{h - 0.5, h - 0.5};
        py[1] = py[1] - 2 * h - gap;
        vector<Eigen::Vector2d> point{{-w, -h}, {w, -h}, {w, h}, {-w, h}};
        vector<vector<Eigen::Vector2d>> vs_list({point, point});

        double h_temp[2] = {h + 0.5, h};
        for (int i = 0; i < 2; i++)
        {
            x_cord += 3.0;
            double y = h_temp[i];
            px.push_back(x_cord);
            px.push_back(x_cord);
            py.push_back(y);
            py.push_back(y - 2 * h - gap);
            vs_list.push_back(point);
            vs_list.push_back(point);
        }
        generateFromRectList(px, py, vs_list);
    }

    void generateParkCase()
    {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("random_map_generator");
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pkg_share + "/meshes/park.pcd", cloud_map) == -1)
        {
            PCL_ERROR("Failed to read PCD file.\n");
            return;
        }

        for (size_t i = 0; i < cloud_map.points.size(); i++)
        {
            double temp = cloud_map.points[i].z;
            if (temp < 1e-3)
            {
                cloud_map.points[i].x = 100.0;
                cloud_map.points[i].y = 100.0;
                cloud_map.points[i].z = 100.0;
                continue;
            }
        }

        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        cloud_map.is_dense = true;
        has_map = true;

        pcl::toROSMsg(cloud_map, global_msg);
        global_msg.header.frame_id = "world";

        mesh_msg.id = 0;
        mesh_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh_msg.action = visualization_msgs::msg::Marker::ADD;
        mesh_msg.pose.orientation.w = 1.0;
        mesh_msg.scale.x = 1.0;
        mesh_msg.scale.y = 1.0;
        mesh_msg.scale.z = 1.0;
        mesh_msg.color.a = 0.5;
        mesh_msg.color.r = 0.0;
        mesh_msg.color.g = 0.0;
        mesh_msg.color.b = 0.0;
        mesh_msg.header.frame_id = "world";
        mesh_msg.mesh_resource = "package://random_map_generator/meshes/park.dae";
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
