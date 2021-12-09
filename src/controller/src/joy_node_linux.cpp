/**
 * @file joy_node_linux.cpp
 * @author Veysi ADIN (veysi.adin@kist.re.kr)
 * @brief This file contains joystick data publisher as a ROS2 node.
 *  Original source code for this node : https://github.com/ros2/joystick_drivers/releases/tag/2.3.2
 *  This code has been downloaded from mentioned address and modified for specific purposes.
 * @version 0.1
 * @date 2021-04-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
// #include <diagnostic_updater/diagnostic_updater.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>   // /// Completely static memory allocation strategy for messages.
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */

#include <rttest/rttest.h>   // To get number of allocation, statistics related memory allocation

#include <tlsf_cpp/tlsf.hpp>   // C++ wrapper for Miguel Masmano Tello's implementation of the TLSF memory allocator
// Implements the allocator_traits template
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

using namespace std::chrono_literals;

///\brief Opens, reads from and publishes joystick events
class Controller
{
private:
  bool open_;
  bool sticky_buttons_;
  bool default_trig_val_;
  std::string joy_dev_;
  std::string joy_dev_name_;
  double deadzone_;
  double autorepeat_rate_;   // in Hz.  0 for no repeat.
  double coalesce_interval_; // Defaults to 100 Hz rate limit.

  int event_count_;
  int pub_count_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  double lastDiagTime_;

  /*! \brief Returns the device path of the first joystick that matches joy_name.
   *         If no match is found, an empty string is returned.
   */
  std::string get_dev_by_joy_name(const std::string& joy_name, rclcpp::Logger logger)
  {
    const char path[] = "/dev/input"; // no trailing / here
    struct dirent *entry;
    struct stat stat_buf;

    DIR *dev_dir = opendir(path);
    if (dev_dir == NULL)
    {
      RCLCPP_ERROR(logger, "Couldn't open %s. Error %i: %s.", path, errno, strerror(errno));
      return "";
    }

    while ((entry = readdir(dev_dir)) != NULL)
    {
      // filter entries
      if (strncmp(entry->d_name, "js0", 3) != 0) // skip device if it's not a joystick
        continue;
      std::string current_path = std::string(path) + "/" + entry->d_name;
      if (stat(current_path.c_str(), &stat_buf) == -1)
        continue;
      if (!S_ISCHR(stat_buf.st_mode)) // input devices are character devices, skip other
        continue;

      // get joystick name
      int joy_fd = open(current_path.c_str(), O_RDONLY);
      if (joy_fd == -1)
        continue;

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));

      close(joy_fd);

      RCLCPP_INFO(logger, "Found joystick: %s (%s).", current_joy_name, current_path.c_str());

      if (strcmp(current_joy_name, joy_name.c_str()) == 0)
      {
          closedir(dev_dir);
          return current_path;
      }
    }

    closedir(dev_dir);
    return "";
  }

public:
  Controller()
  {}

  ///\brief Opens joystick port, reads from port and publishes while node is active
  int main(int argc, char **argv)
  {
    // diagnostic_.add("Joystick Driver Status", this, &Joystick::diagnostics);
    // diagnostic_.setHardwareID("none");
    (void)argc;
    (void)argv;
    auto qos = rclcpp::QoS(
    // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
    // are sent, to aid with recovery in the event of dropped messages.
    // "depth" specifies the size of this buffer.
    // In this example, we are optimizing for performance and limited resource usage (preventing
    // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
    rclcpp::KeepLast(1)
  );
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos.best_effort();
    auto node = std::make_shared<rclcpp::Node>("joy_node");

    // Parameters
    pub_ = node->create_publisher<sensor_msgs::msg::Joy>("Controller", qos);

    joy_dev_ = node->declare_parameter("dev", std::string("/dev/input/js0"));
    joy_dev_name_ = node->declare_parameter("dev_name", std::string(""));
    deadzone_ = node->declare_parameter("deadzone", 0.05);
    autorepeat_rate_ = node->declare_parameter("autorepeat_rate", 30.0);
    coalesce_interval_ = node->declare_parameter("coalesce_intervale", 0.001);
    default_trig_val_ = node->declare_parameter("default_trig_val", false);
    sticky_buttons_ = node->declare_parameter("sticky_buttons", false);

    // Checks on parameters
    if (!joy_dev_name_.empty())
    {
        std::string joy_dev_path = get_dev_by_joy_name(joy_dev_name_, node->get_logger());
        if (joy_dev_path.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "Couldn't find a joystick with name %s. Falling back to default device.", joy_dev_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Using %s as joystick device.", joy_dev_path.c_str());
            joy_dev_ = joy_dev_path;
        }
    }

    if (autorepeat_rate_ > 1 / coalesce_interval_)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);
    }

    if (deadzone_ >= 1)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0)
    {
      RCLCPP_WARN(node->get_logger(), "joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
      coalesce_interval_ = 0;
    }

    // Parameter conversions
    double autorepeat_interval = 1 / autorepeat_rate_;
    double scale = -1. / (1. - deadzone_) / 32767.;
    //double scale = 1. - deadzone_;
    double unscaled_deadzone = 32767. * deadzone_;

    js_event event;
    struct timeval tv;
    fd_set set;
    int joy_fd;

    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = node->now().seconds();

    // Big while loop opens, publishes
    while (rclcpp::ok())
    {
      open_ = false;
      // diagnostic_.force_update();
      bool first_fault = true;
      while (true)
      {
        // In the first iteration of this loop, first_fault is true so we just
        // want to check for rclcpp work and not block.  If it turns out that
        // we cannot open the joystick device immediately, then in subsequent
        // iterations we block for up to a second in rclcpp before attempting
        // to open the joystick device again.  The dummy promise and future
        // are used to accomplish this 1 second wait.
        std::promise<void> dummy_promise;
        std::shared_future<void> dummy_future(dummy_promise.get_future());
        std::chrono::duration<int64_t, std::milli> timeout;
        if (first_fault)
          timeout = std::chrono::milliseconds(0);
        else
          timeout = std::chrono::milliseconds(1000);
        rclcpp::spin_until_future_complete(node, dummy_future, timeout);
        if (!rclcpp::ok())
          goto cleanup;
        joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        if (joy_fd != -1)
        {
          // There seems to be a bug in the driver or something where the
          // initial events that are to define the initial state of the
          // joystick are not the values of the joystick when it was opened
          // but rather the values of the joystick when it was last closed.
          // Opening then closing and opening again is a hack to get more
          // accurate initial state data.
          close(joy_fd);
          joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        }
        if (joy_fd != -1)
          break;
        if (first_fault)
        {
          RCLCPP_ERROR(node->get_logger(), "Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
          first_fault = false;
        }
        // diagnostic_.update();
      }
      RCLCPP_INFO(node->get_logger(), "Opened joystick: %s. deadzone_: %f.", joy_dev_.c_str(), deadzone_);
      open_ = true;
      // diagnostic_.force_update();

      bool tv_set = false;
      bool publication_pending = false;
      tv.tv_sec = 1;
      tv.tv_usec = 0;

      // Here because we want to reset it on device close.
      auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
      double val; //Temporary variable to hold event values
      auto last_published_joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
      auto sticky_buttons_joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
      joy_msg->header.frame_id = "joy";

      while (rclcpp::ok())
      {
        rclcpp::spin_some(node);

        bool publish_now = false;
        bool publish_soon = false;
        FD_ZERO(&set);
        FD_SET(joy_fd, &set);

        //RCLCPP_INFO(node->get_logger(), "Select...");
        int select_out = select(joy_fd+1, &set, NULL, NULL, &tv);
        //RCLCPP_INFO(node->get_logger(), "Tick...");
        if (select_out == -1)
        {
          tv.tv_sec = 0;
          tv.tv_usec = 0;
          //RCLCPP_INFO(node->get_logger(), "Select returned negative. %i", ros::isShuttingDown());
          continue;
          //break; // Joystick is probably closed. Not sure if this case is useful.
        }

        if (FD_ISSET(joy_fd, &set))
        {
          if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
            break; // Joystick is probably closed. Definitely occurs.

          //RCLCPP_INFO(node->get_logger(), "Read data...");
          joy_msg->header.stamp = clock->now();
          event_count_++;
          switch(event.type)
          {
          case JS_EVENT_BUTTON:
          case JS_EVENT_BUTTON | JS_EVENT_INIT:
            if(event.number >= joy_msg->buttons.size())
            {
              int old_size = joy_msg->buttons.size();
              joy_msg->buttons.resize(event.number+1);
              last_published_joy_msg->buttons.resize(event.number+1);
              sticky_buttons_joy_msg->buttons.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg->buttons.size();i++){
                joy_msg->buttons[i] = 0.0;
                last_published_joy_msg->buttons[i] = 0.0;
                sticky_buttons_joy_msg->buttons[i] = 0.0;
              }
            }
            joy_msg->buttons[event.number] = (event.value ? 1 : 0);
            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT))
              publish_now = true;
            else
              publish_soon = true;
            break;
          case JS_EVENT_AXIS:
          case JS_EVENT_AXIS | JS_EVENT_INIT:
            val = event.value;
            if(event.number >= joy_msg->axes.size())
            {
              int old_size = joy_msg->axes.size();
              joy_msg->axes.resize(event.number+1);
              last_published_joy_msg->axes.resize(event.number+1);
              sticky_buttons_joy_msg->axes.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg->axes.size();i++){
                joy_msg->axes[i] = 0.0;
                last_published_joy_msg->axes[i] = 0.0;
                sticky_buttons_joy_msg->axes[i] = 0.0;
              }
            }
            if(default_trig_val_){
              // Allows deadzone to be "smooth"
              if (val > unscaled_deadzone)
                val -= unscaled_deadzone;
              else if (val < -unscaled_deadzone)
                val += unscaled_deadzone;
              else
                val = 0;
              joy_msg->axes[event.number] = val * scale;
              // Will wait a bit before sending to try to combine events.
              publish_soon = true;
              break;
            }
            else
            {
              if (!(event.type & JS_EVENT_INIT))
              {
                val = event.value;
                if(val > unscaled_deadzone)
                  val -= unscaled_deadzone;
                else if(val < -unscaled_deadzone)
                  val += unscaled_deadzone;
                else
                  val=0;
                joy_msg->axes[event.number]= val * scale;
              }

              publish_soon = true;
              break;
              default:
              RCLCPP_WARN(node->get_logger(), "joy_node: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
              break;
            }
          }
        }
        else if (tv_set) // Assume that the timer has expired.
        {
          joy_msg->header.stamp = clock->now();
          publish_now = true;
        }

        if (publish_now) {
          // Assume that all the JS_EVENT_INIT messages have arrived already.
          // This should be the case as the kernel sends them along as soon as
          // the device opens.
          //RCLCPP_INFO(node->get_logger(), "Publish...");
          joy_msg->header.stamp = clock->now();
          pub_->publish(*joy_msg);
         
          publish_now = false;
          tv_set = false;
          publication_pending = false;
          publish_soon = false;
          pub_count_++;
        }

        // If an axis event occurred, start a timer to combine with other
        // events.
        if (!publication_pending && publish_soon)
        {
          tv.tv_sec = trunc(coalesce_interval_);
          tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
          publication_pending = true;
          tv_set = true;
          //RCLCPP_INFO(node->get_logger(), "Pub pending...");
        }

        // If nothing is going on, start a timer to do autorepeat.
        if (!tv_set && autorepeat_rate_ > 0)
        {
          tv.tv_sec = trunc(autorepeat_interval);
          tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
          tv_set = true;
          //RCLCPP_INFO(node->get_logger(), "Autorepeat pending... %li %li", tv.tv_sec, tv.tv_usec);
        }

        if (!tv_set)
        {
          tv.tv_sec = 1;
          tv.tv_usec = 0;
        }

        // diagnostic_.update();
      } // End of joystick open loop.

      close(joy_fd);
      rclcpp::spin_some(node);
      if (rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Connection to joystick device lost unexpectedly. Will reopen.");
      }
    }

  cleanup:
    RCLCPP_INFO(node->get_logger(), "joy_node shut down.");

    return 0;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Controller j;
  return j.main(argc, argv);
}
