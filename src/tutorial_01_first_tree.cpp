#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "behavior_tree_tutorial/dummy_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace DummyNodes;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_tutorial_01");
    RCLCPP_INFO(node->get_logger(), "Starting BehaviorTree Tutorial 01: First Tree");

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode&) {
        return CheckBattery();
    });

    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode&) {
        return gripper.open();
    });
    factory.registerSimpleAction("CloseGripper", [&](BT::TreeNode&) {
        return gripper.close();
    });
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");


    try
    {
        // Get package share directory
        std::string package_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");
        std::string xml_file = package_path + "/trees/tutorial2.xml";
        
        RCLCPP_INFO(node->get_logger(), "Loading tree from: %s", xml_file.c_str());
        auto tree = factory.createTreeFromFile(xml_file);


        RCLCPP_INFO(node->get_logger(), "Executing behavior tree...");
        tree.tickRootWhileRunning();

        RCLCPP_INFO(node->get_logger(), "Behavior tree execution completed successfully!");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}


