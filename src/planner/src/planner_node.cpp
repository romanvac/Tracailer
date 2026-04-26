#include "planner/planner.h"
#include <rclcpp/rclcpp.hpp>

using namespace trailer_planner;

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : rclcpp::Node("planner_node") {}

    void setup()
    {
        planner_.init(this);
    }

private:
    Planner planner_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
