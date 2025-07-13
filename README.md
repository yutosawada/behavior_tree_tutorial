# BehaviorTree Tutorial Package

This package implements the BehaviorTree.CPP tutorials for ROS2 Humble.

## Tutorial 01: First Tree

This tutorial demonstrates the basics of creating and executing a behavior tree.

### Building the package

```bash
cd /workspace
colcon build --packages-select behavior_tree_tutorial
source install/setup.bash
```

### Running the tutorial

Using ros2 run:
```bash
ros2 run behavior_tree_tutorial tutorial_01_first_tree
```

Using launch file:
```bash
ros2 launch behavior_tree_tutorial tutorial_01_launch.py
```

### Expected Output

```
[ Battery: OK ]
GripperInterface::open
ApproachObject: approach_object
GripperInterface::close
```

## Tree Structure

The behavior tree consists of:
- **Sequence Node**: Executes children in order, stops on first failure
- **CheckBattery**: Simple condition that always returns SUCCESS
- **OpenGripper**: Action to open the gripper
- **ApproachObject**: Custom action node to approach an object
- **CloseGripper**: Action to close the gripper

## Files

- `src/tutorial_01_first_tree.cpp`: Main executable
- `src/dummy_nodes.cpp`: Implementation of custom nodes
- `include/behavior_tree_tutorial/dummy_nodes.h`: Header file for custom nodes
- `trees/my_tree.xml`: XML definition of the behavior tree

## Tutorial 04: Reactive and Asynchronous behaviors

This tutorial demonstrates:
- StatefulActionNode for asynchronous actions
- Difference between Sequence and ReactiveSequence

### Running tutorial 04

Using ros2 run:
```bash
# Normal Sequence
ros2 run behavior_tree_tutorial tutorial_04_sequence

# ReactiveSequence
ros2 run behavior_tree_tutorial tutorial_04_sequence --reactive
```

Using launch file:
```bash
ros2 launch behavior_tree_tutorial tutorial_04_launch.py
```

### Expected Output (Normal Sequence)

```
--- ticking
[ Battery: OK ]
Robot says: mission started...
[ MoveBase: SEND REQUEST ]. goal: x=1.0 y=2.0 theta=3.0
--- status: RUNNING

--- ticking
--- status: RUNNING

--- ticking
[ MoveBase: FINISHED ]
Robot says: mission completed!
--- status: SUCCESS
```

### Expected Output (ReactiveSequence)

```
--- ticking
[ Battery: OK ]
Robot says: mission started...
[ MoveBase: SEND REQUEST ]. goal: x=1.0 y=2.0 theta=3.0
--- status: RUNNING

--- ticking
[ Battery: OK ]
--- status: RUNNING

--- ticking
[ Battery: OK ]
[ MoveBase: FINISHED ]
Robot says: mission completed!
--- status: SUCCESS
```
