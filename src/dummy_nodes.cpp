#include "behavior_tree_tutorial/dummy_nodes.h"

using namespace BT;

namespace DummyNodes
{

// Simple function that return a NodeStatus
NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return NodeStatus::SUCCESS;
}

//-------------------------------
// GripperInterface implementation
//-------------------------------

GripperInterface::GripperInterface() : _open(true)
{
}

// We want to wrap into an ActionNode the methods open() and close()
NodeStatus GripperInterface::open()
{
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return NodeStatus::SUCCESS;
}

//-------------------------------
// ApproachObject implementation
//-------------------------------

ApproachObject::ApproachObject(const std::string& name) :
    BT::SyncActionNode(name, {})
{
}

// Custom Action Node implementation
NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return NodeStatus::SUCCESS;
}

//-------------------------------
// SaySomething implementation
//-------------------------------

SaySomething::SaySomething(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList SaySomething::providedPorts()
{
    // This action has a single input port called "message"
    return { BT::InputPort<std::string>("message") };
}

// Override the virtual function tick()
BT::NodeStatus SaySomething::tick()
{
    auto msg = getInput<std::string>("message");
    // Check if expected is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error());
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

//-------------------------------
// ThinkWhatToSay implementation
//-------------------------------

ThinkWhatToSay::ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ThinkWhatToSay::providedPorts()
{
  return { BT::OutputPort<std::string>("text") };
}

// This Action writes a value into the port "text"
BT::NodeStatus ThinkWhatToSay::tick()
{
  // the output may change at each tick(). Here we keep it simple.
  setOutput("text", "The answer is 42");
  return BT::NodeStatus::SUCCESS;
}

//-------------------------------
// MoveBaseAction implementation
//-------------------------------

MoveBaseAction::MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList MoveBaseAction::providedPorts()
{
    return { BT::InputPort<Pose2D>("goal") };
}

BT::NodeStatus MoveBaseAction::onStart()
{
    if (!getInput<Pose2D>("goal", _goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }
    printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
           _goal.x, _goal.y, _goal.theta);

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (220 ms)
    _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if (std::chrono::system_clock::now() >= _completion_time)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
    printf("[ MoveBase: ABORTED ]\n");
}


} // namespace DummyNodes
