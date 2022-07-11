## Debugging the Control Software.

 This document provides several methods to debug your control software and find issue regarding the software.
 Please, try out the provided examples for the software and check out the guide resources before starting development and debugging. Currently 5 different nodes are implemented for the workbench of the control software. If you only need to use ethercat package, you might need to implement your own lifecycle node manager, or implement launch file to manage lifecycle of the EtherCAT node. Other nodes and their responsibilities are explained in readme.md document in this repository. Additional log package is added, to decouple timing measurements from real-time communication node, to a seperate node. Currently it subscribes slave feedback values and extracts timing parameters and prints the timinig values.

## Possible Errors and Debugging.
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
##### Note that, two launch files are provided. In launch_all_nodes.py file, state transitions are requested by pressing buttons in GUI node, and Safety Node changes state for the EtherCAT node. In other launch file, ethercat_nodes_launch.py file, state transitions directly triggered in the python script, and in the current version, it brings the EtherCAT node to the active state.
