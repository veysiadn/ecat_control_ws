## Debugging the Control Software.

  This document provides several methods to debug your control software and find issue regarding the software.
 Please, try out the provided examples for the software, and check out the guide resources before starting development and debugging. Currently 5 different nodes are implemented for the workbench of the control software. If you only need to use ethercat package, you might need to implement your own lifecycle node manager, or implement launch file to manage lifecycle of the EtherCAT node. Additonally, there might be slight coupling between EtherCAT node and other nodes, therefore, keep in mind that you may need to change several parameters in the EtherCAT package as well.
 Other nodes and their responsibilities are explained in readme document in this repository. 
 Recently additional log package is added, to decouple timing measurements from real-time communication node, to a seperate node. Currently it subscribes slave feedback values and extracts timing parameters and prints the timinig values.

## Possible Errors and Debugging.

Errors regarding the servo drive is printed out to the console by the safety node, and error register of the driver is monitored via EtherCAT communication. In case of any error in the driver, please check the printed error message and check your driver's datasheet first to see if you can solve your issue.

You can check kernel messages to see possible warnings or errors related to Etherlab library, or any other kernel related issue in your implementation. Note that this helps you identify the error, the solution must be investigated seperataly, through google search, or writing the mailing list of the library.

```sh
    dmesg -w
```

#### If Rx queue overflow error happens too often 
If your CPU has Skylake architecture, you might need to add this kernel parameters to your grub file. It fixed the overflow issue in my case. 

```sh
  sudo nano /etc/default/grub
```
Change the GRUB_CMDLINE_LINUX_DEFAULT parameter. If you have other parameters, just copy ' i915.enable_rc6=0 i915.enable_dc=0 nosmap' part and paste to CMDLINE.
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash i915.enable_rc6=0 i915.enable_dc=0 nosmap"
```
Then, update the grub and reboot your system.
```
sudo update-grub
sudo reboot
```
#### If EtherCAT communication is fine, but can't enable motors. Check if you are changing control word, before enabling the motors.
