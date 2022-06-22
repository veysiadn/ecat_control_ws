## Welcome to Veysi ADIN & Chunwoo Kim's ROS2 EtherCAT package.

  This repository contains ROS2 and EtherCAT based control framework for our spine surgery robot project, but this implementation can be used with any robotic system with small modifications.Contains EtherCAT real-time thread with priority of 98. Software consists of four components (ROS2 nodes) : 
  
**• EtherCAT node:** Responsible for EtherCAT communication between master and slaves and publishes acquired feedback from slaves under /slave_feedback topic name in 1 kHz frequency. Additionally, subscribes to /controller topic, published from the control input node, / gui_buttons topic published from the GUI node, and /safety_info topic published from the safety node. Calculates target values for each motor and sends control commands to slaves via EtherCAT communication and publishes those values under the /master_commands topic. 


**• Control node:** Acquires input data from the input device and in the current setup input device is the Xbox controller. Therefore, it gets axis data and button data from the Xbox controller and publishes it under the /controller topic. Additionally, this node subscribes to the /slave_feedback topic published from the EtherCAT node.


**• GUI node:** Consists of camera viewer and slave feedback visualizers such as motor state, communication state, and emergency button state. Publishes under /gui_buttons data consists of soft button events, subscribes to /master_ commands, and /slave_feedback, and /safety info topics to give visual feedback to the user. The current version of the GUI node is shown in Figure 25. Control user interface buttons activated based on lifecycle node state.

**• Safety node:** Subscribes to all published topics and checks the consistency in the system, decides about the state of the system. System-related safety information is checked every cycle. Manages lifecycle node and state transitions are triggered from this node.

Published and subscribed topics for each node are shown in figure below,

![Nodes Published and Subscribed Topics](https://github.com/veysiadn/ecat_control_ws/blob/master/docs/img/node_pu_sub_topics.jpg)

 and the contents of the topics are shown in figure below.
  
![Topics and contained messages](https://github.com/veysiadn/ecat_control_ws/blob/master/docs/img/topic_msgs.jpg)

Note that published topic messages can be changed by changing msg file contents in case custom message is required.

#### Please check guides, links and documentations before installation, or using this control framework.

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
- [IgH EtherCAT](https://github.com/veysiadn/IgHEtherCATImplementation)
- [RT_Preempt Linux](https://github.com/veysiadn/RT_PREEMPT_INSTALL)
- If you want to use [Xenomai-Instalattion](https://github.com/veysiadn/xenomai-install)
- [ROS2 Foxy Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

## Building and running
### Building  
```sh
  git clone https://github.com/veysiadn/ecat_control_ws
  source /opt/ros/foxy/setup.bash
  cd ecat_control_ws
  colcon build --symlink-install
```
Note that, before running, you might need to change lauch_all_nodes.py file, to isolate specific CPU core in your system.
#### Running
```sh
  sudo -s
  source /opt/ros/foxy/setup.bash
  . install/setup.bash
  ros2 launch launch_all_nodes.py
```

