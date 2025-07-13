#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace DummyNodes
{

//-------------------------------
// Custom type for position
//-------------------------------

struct Pose2D
{
    double x, y, theta;
};

//-------------------------------
// Simple Action Nodes
//-------------------------------

BT::NodeStatus CheckBattery();

class GripperInterface
{
public:
    GripperInterface();

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
    ApproachObject(const std::string& name);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public BT::SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature 
  SaySomething(const std::string& name, const BT::NodeConfiguration& config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  // Override the virtual function tick()
  BT::NodeStatus tick() override;
};

class ThinkWhatToSay : public BT::SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  // This Action writes a value into the port "text"
  BT::NodeStatus tick() override;
};

//-------------------------------
// Asynchronous Action Node
//-------------------------------

class MoveBaseAction : public BT::StatefulActionNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Pose2D _goal;
    std::chrono::system_clock::time_point _completion_time;
};

} // namespace DummyNodes
