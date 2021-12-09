#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"


#include "DxlMaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class surgicalToolNode : public rclcpp::Node
{
  public:
    surgicalToolNode()
    : Node("surgicalToolNode")
    {
      dxl = std::make_unique<DxlMaster>();
      dxl->InitializeDriver(MODEMDEVICE,BAUDRATE,PROTOCOL_VERSION);
      dxl->GetDynamixelInfo(1);
      dxl->GetDynamixelInfo(2);
      dxl->SetWheelMode(1);
      dxl->SetWheelMode(2);

      printf("Enabling...\n");
      dxl->EnableTorque(1);
      dxl->EnableTorque(2);

      // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
      // are sent, to aid with recovery in the event of dropped messages.
      // "depth" specifies the size of this buffer.
      // In this example, we are optimizing for performance and limited resource usage (preventing
      // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      
      // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
      // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
      qos.best_effort();
      joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("Controller", qos, 
                                    std::bind(&surgicalToolNode::JstckCallbacks, this,std::placeholders::_1));
    }

  ~surgicalToolNode()
  {
    for(int i=1; i<2; i++)  {   dxl->DisableTorque(i);  }
    dxl->Disconnect();
  }

  private:
    // CKim - Subscriber for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joystick_subscriber_;

    // CKim - Call back for joystick messages
    void JstckCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Jstck messages %.2f, %.2f", msg->axes[0], msg->axes[3]);
      //controller_.left_x_axis_  = msg->axes[0];       //controller_.left_y_axis_  = msg->axes[1];
      //controller_.right_x_axis_ = msg->axes[3];       //controller_.right_y_axis_ = msg->axes[4];

      // MX28 0~2047 (0X7FF) can be used, and the unit is about 0.114rpm.
      // If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
      // If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.
      // That is, the 10th bit becomes the direction bit to control the direction.
      int vel[2] = {0,0};     float mag = 0.0;  
      
      // 0 is for tool rotation
      mag = fabs(msg->axes[3]);
      if(mag < 0.05)  {   mag = 0;  }
      else            {   mag -= 0.05;  }
      vel[0] = 600*mag;
      if(msg->axes[3] < 0)  {   vel[0] += 1024;   }   

      // 1 is for grasping. joystick input is 1 when released and -1 when pressed
      if((msg->axes[2] < 0.95) && (msg->axes[5] > 0.95))  {
        vel[1] = 500*fabs(msg->axes[2] - 1.0);    
      }
      if((msg->axes[2] > 0.95) && (msg->axes[5] < 0.95))  {
        vel[1] = 500*fabs(msg->axes[5] - 1.0) + 1024;
      }
      dxl->SetVelAll(vel);
    }

    std::unique_ptr<DxlMaster>  dxl;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<surgicalToolNode> pNode = std::make_shared<surgicalToolNode>();
  rclcpp::spin(pNode);
  printf("Spinning ended\n");
  rclcpp::shutdown();

  return 0;
}