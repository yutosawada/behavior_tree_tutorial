#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "behavior_tree_tutorial/dummy_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace DummyNodes;

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_tutorial_01");

    RCLCPP_INFO(node->get_logger(), "Starting BehaviorTree Tutorial 01: First Tree");

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // You can use C++11 lambdas or std::bind
    factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode&) {
        return CheckBattery();
    });

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode&) {
        return gripper.open();
    });
    factory.registerSimpleAction("CloseGripper", [&](BT::TreeNode&) {
        return gripper.close();
    });

    // Trees are created at deployment-time (i.e. at run-time, but only
    // once at the beginning).

    // IMPORTANT: when the object "tree" goes out of scope, all the
    // TreeNodes are destroyed
    try
    {
        // Get package share directory
        std::string package_path = ament_index_cpp::get_package_share_directory("behavior_tree_tutorial");
        std::string xml_file = package_path + "/trees/my_tree.xml";
        
        RCLCPP_INFO(node->get_logger(), "Loading tree from: %s", xml_file.c_str());
        auto tree = factory.createTreeFromFile(xml_file);

        // To "execute" a Tree you need to "tick" it.
        // The tick is propagated to the children based on the logic of the tree.
        // In this case, the entire sequence is executed, because all the children
        // of the Sequence return SUCCESS.
        RCLCPP_INFO(node->get_logger(), "Executing behavior tree...");
        tree.tickWhileRunning();

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

/* Expected output:
    [ Battery: OK ]
    GripperInterface::open
    ApproachObject: approach_object
    GripperInterface::close
*/
