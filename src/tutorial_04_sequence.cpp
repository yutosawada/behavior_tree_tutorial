#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "behavior_tree_tutorial/dummy_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace DummyNodes;

static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree>
        <Sequence>
            <BatteryOK/>
            <SaySomething   message="mission started..." />
            <MoveBase       goal="1;2;3"/>
            <SaySomething   message="mission completed!" />
        </Sequence>
    </BehaviorTree>
</root>
)";

static const char* xml_text_reactive = R"(
<root BTCPP_format="4">
    <BehaviorTree>
        <ReactiveSequence>
            <BatteryOK/>
            <Sequence>
                <SaySomething   message="mission started..." />
                <MoveBase       goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
        </ReactiveSequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_tutorial_04");

    RCLCPP_INFO(node->get_logger(), "Starting BehaviorTree Tutorial 04: Reactive and Asynchronous behaviors");

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // Register nodes
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    // Choose which tree to execute (change this to test different behaviors)
    bool use_reactive_sequence = false;
    
    if (argc > 1 && std::string(argv[1]) == "--reactive") {
        use_reactive_sequence = true;
        RCLCPP_INFO(node->get_logger(), "Using ReactiveSequence");
    } else {
        RCLCPP_INFO(node->get_logger(), "Using normal Sequence");
    }

    const char* xml_to_use = use_reactive_sequence ? xml_text_reactive : xml_text;

    try
    {
        auto tree = factory.createTreeFromText(xml_to_use);

        // Here, instead of tree.tickWhileRunning(),
        // we prefer our own loop.
        RCLCPP_INFO(node->get_logger(), "--- ticking");
        auto status = tree.tickOnce();
        RCLCPP_INFO(node->get_logger(), "--- status: %s", BT::toStr(status).c_str());

        while (status == BT::NodeStatus::RUNNING && rclcpp::ok())
        {
            // Sleep to avoid busy loops.
            // do NOT use other sleep functions!
            // Small sleep time is OK, here we use a large one only to
            // have less messages on the console.
            tree.sleep(std::chrono::milliseconds(100));

            RCLCPP_INFO(node->get_logger(), "--- ticking");
            status = tree.tickOnce();
            RCLCPP_INFO(node->get_logger(), "--- status: %s", BT::toStr(status).c_str());
        }

        RCLCPP_INFO(node->get_logger(), "Behavior tree execution completed with status: %s", 
                   BT::toStr(status).c_str());
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
