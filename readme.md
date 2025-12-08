ROS Turtlesim Leader–Follower Formation Control

This project implements a multi-turtle leader–follower formation control system using the Turtlesim simulator in ROS (Robot Operating System).
A leader turtle navigates the environment while follower turtles maintain a geometric formation relative to the leader’s pose.
The system demonstrates concepts of multi-robot coordination, control, message passing, and service-based resets.

 
 Project Overview

This ROS package accomplishes the following:

Spawns one leader and multiple followers in Turtlesim

Leader publishes its pose and moves with predefined control logic

Followers compute offset vectors to maintain formation (e.g., triangle, line, V-shape)

A reset service teleports all turtles to starting positions when leader hits boundaries

The project uses:

Custom messages

Custom services

Separate ROS nodes for leader and followers

Automatic execution via launch files
Package Structure
com760_yourbcode/
├── launch/
│   ├── spawn_turtles.launch
│   ├── formation_control.launch
├── scripts/
│   ├── leader.py
│   ├── follower.py
│   ├── reset_service.py
├── msg/
│   └── TurtleInfo.msg
├── srv/
│   └── ResetTurtles.srv
├── urdf/
│   └── turtle_model.urdf        (optional)
├── world/
│   └── turtlesim_world.yaml     (optional)
├── CMakeLists.txt
└── package.xml


Note: The package name must be lowercase (e.g., com760_yourbcode) to comply with ROS conventions.

How It Works
1. Leader Node (leader.py)

Publishes /leader_pose

Moves forward with velocity commands

Monitors boundaries in Turtlesim

Calls reset service when close to borders

2. Follower Node (follower.py)

Each follower:

Subscribes to the leader’s pose

Computes relative offset (dx, dy) to maintain formation

Issues its own /cmd_vel velocity commands

Smoothly adjusts direction to follow leader path

3. Reset Service (reset_service.py)

Handles:

Teleporting all turtles to initial coordinates

Resetting headings

Ensuring formation restarts cleanly after boundary detection
