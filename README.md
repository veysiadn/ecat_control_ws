## Welcome to Veysi ADIN & Chunwoo Kim's ROS2 EtherCAT package.

  This repository contains ROS2 and EtherCAT based control framework for our spine surgery robot project, but this implementation can be used with any robotic system with small modifications.Contains EtherCAT real-time thread with priority of 98. Software consists of four components (ROS2 nodes) : 
  
  • EtherCAT node : Responsible for EtherCAT communication between master and slaves and publishes acquired feedback from slaves under /slavefeedback topicname in 1 kHz frequency. Additionally, subscribes to/mastercommands topic published from control node and sends control commands to slaves via EtherCATcommunication.
  
  • Control node : Kinematic calculations will be done in this node. This node subscribes /slavefeedback topic published from EtherCAT node and publishes control commands under /mastercommands topic.Currently this node retrieves button and joystick data from Xbox Controller via USB communication.
  
  • GUI node : Consist of camera viewer and slave feedback visualizers such as motor state, communication state, emergency button state. Publishes under /guidata consists of soft emergency button events, subscribes to /mastercommands and /slavefeedback topics to give visual feedback to user
  
 • Surgical Tool Node : Contains Dynamixel SDK for RS485 communication with Dynamixel motors.
 
 Please check guides, links and documentations before installation, or using this control framework.

## Guides

- [ROS2](https://docs.ros.org/en/foxy/index.html)
- [ROS2 Life-cycle Node](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS2 DDS](https://design.ros2.org/articles/ros_on_dds.html)
- [ROS2 QoS](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [EtherCAT](https://www.ethercat.org/en/technology.html)
- [Etherlab Webpage](https://www.etherlab.org/en/ethercat/index.php)
- [IgH EtherCAT Library Documentation](https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf)
- [Real-time Linux](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/start)
- [ROS2 Real-time Background](https://design.ros2.org/articles/realtime_background.html)
- [Article on EtherCAT-RT PREEMPT- Xenomai](https://www.ripublication.com/ijaer17/ijaerv12n21_94.pdf)

## Prerequisites
- [RT_Preempt Linux and IgH EtherCAT Implementation](https://github.com/veysiadn/IgHEtherCATImplementation)
- If you want to use [Xenomai-Instalattion](https://github.com/veysiadn/xenomai-install)
- [ROS2 Foxy Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Dynamixel-SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## Implementation
  
```sh
mkdir spinerobot_ws 
cd spinerobot_ws
git clone https://github.com/veysiadn/spinerobot_ws src
sudo -s
source /opt/ros/foxy/setup.bash
colcon build --symlink install
. install/setup.bash
ros2 launch ./src/ethercat_nodes_launch.py
```

