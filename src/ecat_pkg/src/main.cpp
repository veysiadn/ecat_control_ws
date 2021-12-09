/****************************************************************************/
#include "rclcpp/rclcpp.hpp"
/****************************************************************************/
#include "ecat_node.hpp"
/****************************************************************************/
#include "ecat_lifecycle.hpp"
std::unique_ptr<EthercatLifeCycleNode::EthercatLifeCycle> ecat_lifecycle_node;

void signalHandler(int /*signum*/)
{
    //sig = 0;
    //usleep(1e3);
    ecat_lifecycle_node->shutdown();
}

int main(int argc, char **argv)
{
    // CKim - Configure stdout sream buffer. _IONBF means no buffering. Each I/O operation is written as soon as possible. 
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // CKim - Associate 'signalHandler' function with interrupt signal (Ctrl+C key)
    signal(SIGINT,signalHandler);

    // CKim - Init ROS2 node. rclcpp stands for ROS Client Library for CPP. 
    // rclcpp provides the standard C++ API for interacting with ROS 2.
    rclcpp::init(argc, argv);

    // -----------------------------------------------------------------------------
    // CKim - Prepare memory for real time performance 
    // https://design.ros2.org/articles/realtime_background.html
    
    // CKim - Lock this processe's memory. Necessary for real time performance....
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Mlockall failed, check if you have sudo authority.");
        return -1;
    }

    
    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
    // -----------------------------------------------------------------------------

    // CKim - Initialize and launch EthercatLifeCycleNode
    ecat_lifecycle_node = std::make_unique<EthercatLifeCycleNode::EthercatLifeCycle>();
    rclcpp::spin(ecat_lifecycle_node->get_node_base_interface());
    
    // CKim - Terminate node
    rclcpp::shutdown();
    return 0;
}

