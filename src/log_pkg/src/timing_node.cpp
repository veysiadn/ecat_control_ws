#include "timing_node.hpp"

using namespace std::chrono_literals;
using namespace Timing;

TimingNode::TimingNode()
  : Node("timing_node")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  received_data_sub_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", qos, 
                                   std::bind(&TimingNode::HandleReceivedDataCallback, this, std::placeholders::_1));
  statistics_timer_ = this->create_wall_timer(1s,std::bind(&TimingNode::PrintStatistics,this));
  received_data_ = std::make_shared<ecat_msgs::msg::DataReceived>();
}
TimingNode::~TimingNode()
{
  
}
void TimingNode::HandleReceivedDataCallback(const ecat_msgs::msg::DataReceived::SharedPtr msg)
{
  received_data_ = msg;
  CalculateStatistics();
}

void TimingNode::CalculateStatistics()
{
  if(received_data_->current_lifecycle_state==3u){
    if(!print_begin_){
      if(received_data_->period_ns > period_max_ns_)
        period_max_ns_ = received_data_->period_ns;
      if(received_data_->period_ns < period_min_ns_)
        period_min_ns_ = received_data_->period_ns;
      if(received_data_->exec_ns > exec_max_ns_)
        exec_max_ns_ = received_data_->exec_ns;
      if(received_data_->exec_ns < exec_min_ns_)
        exec_min_ns_ = received_data_->exec_ns;
      if(received_data_->jitter_ns > jitter_max_)
        jitter_max_ = received_data_->jitter_ns;
      if(received_data_->jitter_ns < jitter_min_)
        jitter_min_ = received_data_->jitter_ns;
    }
  }else{
    period_max_ns_ = 0 ;
    period_min_ns_ = 0xffffffff;
    exec_max_ns_ = 0;
    exec_min_ns_ = 0xffffffff;
    jitter_max_ = 0;
    jitter_min_ = 0xffffffff;
  }
}


void TimingNode::PrintStatistics()
{
    if(received_data_->current_lifecycle_state==3u){
      if(!print_begin_){
        RCLCPP_INFO(this->get_logger(), "Statistics:");
        RCLCPP_INFO(this->get_logger(), "  period_min_ns: %d", period_min_ns_);
        RCLCPP_INFO(this->get_logger(), "  period_max_ns: %d", period_max_ns_);
        RCLCPP_INFO(this->get_logger(), "  exec_min_ns: %d", exec_min_ns_);
        RCLCPP_INFO(this->get_logger(), "  exec_max_ns: %d", exec_max_ns_);
        RCLCPP_INFO(this->get_logger(), "  jitter_min: %d", jitter_min_);
        RCLCPP_INFO(this->get_logger(), "  jitter_max: %d", jitter_max_);
      }else{
        print_begin_--;
      }
    }else{
      print_begin_ = 20;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimingNode>());
  rclcpp::shutdown();
  return 0;
}
