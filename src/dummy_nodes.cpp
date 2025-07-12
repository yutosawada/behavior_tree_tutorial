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

// Custom Action Node implementation
NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return NodeStatus::SUCCESS;
}

} // namespace DummyNodes
