#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <iostream>

namespace DummyNodes
{

//-------------------------------
// Simple Action Nodes
//-------------------------------

BT::NodeStatus CheckBattery();

class GripperInterface
{
public:
    GripperInterface() : _open(true) {}

    BT::NodeStatus open();
    BT::NodeStatus close();

private:
    bool _open; // shared information
};

//-------------------------------
// Custom Action Node (by inheritance)
//-------------------------------

class ApproachObject : public BT::SyncActionNode
{
public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

} // namespace DummyNodes
