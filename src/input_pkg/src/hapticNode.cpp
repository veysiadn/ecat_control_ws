#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

// CKim - Socket Headers
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ecat_msgs/msg/haptic_cmd.hpp"  // CKim - Header for custom message

#include "hapticNode.hpp"

using namespace std::chrono_literals;

// -------------------------------------------------------- //
// CKim - This ROS node reads HapticDevice command from
// network and publishes it
// -------------------------------------------------------- //

HapticNode::HapticNode(char* argv[]) : rclcpp::Node("HapticNode")
{
  // CKim - Set IP and Port
  m_IP = argv[1];
  m_Port = argv[2];

  auto qos = rclcpp::QoS(
      // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
      // are sent, to aid with recovery in the event of dropped messages.
      // "depth" specifies the size of this buffer.
      // In this example, we are optimizing for performance and limited resource usage (preventing
      // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
      rclcpp::KeepLast(1));
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos.best_effort();
  // CKim - Initialize publisher
  haptic_publisher_ = this->create_publisher<ecat_msgs::msg::HapticCmd>("HapticInput", qos);

  // CKim - Launch thread that will constantly read socket and publish data
  future_ = exit_signal_.get_future();
  comm_thread_ = std::thread(&HapticNode::commThread, this);
}

void HapticNode::commThread()
{
  RCLCPP_INFO(get_logger(), "Starting Haptic Node");

  // CKim - Initialize socket and timeout
  int sock = socket(PF_INET, SOCK_STREAM, 0);
  fcntl(sock, F_SETFL, O_NONBLOCK);
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  fd_set fdset;
  FD_ZERO(&fdset);
  FD_SET(sock, &fdset);

  // CKim - Set up address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));  // Initialize to zero
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(m_IP.c_str());
  server_addr.sin_port = htons(atoi(m_Port.c_str()));

  // CKim - Connect. socket is set to be non blocking, use select to wait for result
  // and return upon error
  RCLCPP_INFO(get_logger(), "Connecting to server at %s through port %s\n", m_IP.c_str(), m_Port.c_str());
  connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
  if (select(sock + 1, NULL, &fdset, NULL, &tv) == 1)
  {
    int so_error;
    socklen_t len = sizeof(so_error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if (so_error == 0)
    {
      RCLCPP_INFO(get_logger(), "Connected to Server!");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Error during connection! Press Ctrl+C to terminate.");
      close(sock);
      return;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Timeout in select()! Press Ctrl+C to terminate.");
    close(sock);
    return;
  }

  // CKim - Read from socket and publish. Socket must be set back to blocking mode
  // for the code below to work
  fcntl(sock, F_SETFL, fcntl(sock, F_GETFL) & ~O_NONBLOCK);
  std::future_status status;
  status = future_.wait_for(std::chrono::seconds(0));
  RCLCPP_INFO(get_logger(), "Entering communication loop!");
  while (status == std::future_status::timeout)
  {
    // ----------------------------------------------------------- //
    // CKim - This code is using no protocol. Just 7 doubles
    // ----------------------------------------------------------- //
    // CKim - Allocate memory for 7 doubles
    int leng = 7 * sizeof(double);  //+ 2*sizeof(int);
    char* str2 = new char[leng];    //길이 만큼 배열 동적할당
    if (str2 == (char*)0)
    {
      RCLCPP_INFO(get_logger(), "Memory error!");
      break;
    }
    memset(str2, 0, leng);

    // CKim - Read and check the size of data received
    int real_recv_len, real_recv_byte;
    real_recv_len = 0;
    real_recv_byte = 0;

    while (real_recv_len < leng)
    {
      real_recv_byte = read(sock, &str2[real_recv_len], leng - real_recv_len);
      real_recv_len += real_recv_byte;
    }

    // CKim - Convert received bytes to doubles and ints
    double* val = (double*)str2;
    // int* btn = (int*) (str2+6*sizeof(double));

    // CKim - Fill the messag and publish data
    // First three element X, Y, Z increment. In mm
    // X: -/+ Left Right 		// Y: -/+ Down Up 		// Z: -/+ In / Out
    // Last three element Roll (Z) Pitch (X) Yaw (Y). In degree
    for (int i = 0; i < 3; i++)
    {
      hapticMsg.array[i] = val[i];
    }
    for (int i = 3; i < 6; i++)
    {
      hapticMsg.array[i] = val[i];
    }  //*2000.0;			}
    hapticMsg.array[6] = val[6];
    hapticMsg.btn[0] = 0;  // btn[1];
    hapticMsg.btn[1] = 0;  // btn[1];

    haptic_publisher_->publish(hapticMsg);

    status = future_.wait_for(std::chrono::seconds(0));
  }

  // CKim - End communication
  RCLCPP_INFO(get_logger(), "Leaving communication loop!");
  close(sock);
  return;
}

HapticNode::~HapticNode()
{
  // CKim - Trigger exit signal to stop the thread
  exit_signal_.set_value();
  comm_thread_.join();
}

int main(int argc, char* argv[])
{
  // CKim - Node is launched with argument specifying IP and port
  if (argc < 3)
  {
    printf("Usage : %s <IP> <port> \n", "ros2 run input_pkg hapticNode");
    exit(1);
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HapticNode>(argv));
  printf("Spinning ended\n");
  rclcpp::shutdown();
  // for(int i=1; i<3; i++)  {   dxl.DisableTorque(i);  }
  // dxl.Disconnect();
  return 0;
}

// -------------------------------

// -------------------------------------------------------- //
// CKim - This ROS node reads HapticDevice command from
// network and publishes it
// -------------------------------------------------------- //

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <string.h>
// #include <arpa/inet.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <netinet/in.h>
// #include <string>
// #include <time.h>
// #include "ros/ros.h" // ROS 기본 헤더 파일
// #include <SkillMate/HapticCommand.h>        // CKim - Automatically generated message Header

// #include <iostream>
// #include <fstream>

// using namespace std;

// int main(int argc, char* argv[])
// {
// 	int sock;
// 	struct sockaddr_in server_addr;

//     if(argc<3)
// 	{
//         printf("Usage : %s <IP> <port> \n", argv[3]);
// 		exit(1);
// 	 }

//     // Client Socket
// 	sock = socket(PF_INET, SOCK_STREAM, 0);

// 	//=======
//     ros::init(argc, argv, "HapticCmdPublisher");    //노드 초기화
//     ros::NodeHandle nh; //노드 핸들 선언

//     ros::Publisher haptic_pub =	nh.advertise<SkillMate::HapticCommand>("HapticCmd", 100);
//     ROS_INFO("Starting Haptic Node");
//     //====

// 	// Connect to address
// 	memset(&server_addr, 0, sizeof(server_addr));//서버 주소 초기화
//     server_addr.sin_family = AF_INET;
//     server_addr.sin_addr.s_addr = inet_addr(argv[1]);
//     server_addr.sin_port = htons(atoi(argv[2]));

//     ROS_INFO("Connecting to Server!\n");
// 	if(connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1)
// 	{
// 		printf("connect() error\n");
// 		close(sock);
// 		exit(1);
// 	}
//     ROS_INFO("Connected to Server!\n");

// 	//ros::Rate loop_rate(10);
// 	//ros::WallTime currTime = ros::WallTime::now();
// 	while(ros::ok())
// 	{
// 		// ----------------------------------------------------------- //
// 		// CKim - This code is using no protocol. Just 6 doubles
// 		// ----------------------------------------------------------- //
// 		int leng, real_recv_len, real_recv_byte;

//         leng = 6*sizeof(double) + 2*sizeof(int);
// 		char* str2 = new char[leng]; //길이 만큼 배열 동적할당

// 		if (str2 == (char *) 0) {
// 			printf("memory error!\n");
// 			exit(1);
// 		}

// 		memset(str2, 0, leng);
// 		real_recv_len = 0;
// 		real_recv_byte = 0;

// 		//받는 방법 2 : 받는 바이트 확인하며 받기
// 		while (real_recv_len < leng) {
// 			real_recv_byte = read(sock, &str2[real_recv_len],
// 					leng - real_recv_len);
// 			real_recv_len += real_recv_byte;
// 		}

// 		double* val = (double*) str2;
//         int* btn = (int*) (str2+6*sizeof(double));

//         SkillMate::HapticCommand msg;

// 		// CKim - First three element X, Y, Z increment. In mm
// 		// X: -/+ Left Right
// 		// Y: -/+ Down Up
// 		// Z: -/+ In / Out
// 		for(int i=0; i<3; i++)	{
// 			msg.array[i] = val[i];			}

// 		// CKim - Last three element Roll (Z) Pitch (X) Yaw (Y). In degree
// 		for(int i=3; i<6; i++)	{
//             msg.array[i] = val[i];  }//*2000.0;			}
// 		msg.btn[0] = btn[0];
// 		msg.btn[1] = btn[1];

// 		// ----------------------------------------------------------- //

// 		// 메시지를 퍼블리시 한다.
// 		haptic_pub.publish(msg);

// 		//loop_rate.sleep();
// 	}
// 	close(sock);
// 	return 0;
// }

// © 2022 GitHub, Inc.
// Terms
