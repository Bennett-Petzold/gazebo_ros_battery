#include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/BatteryState.h>
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

// Custom Callback Queue
//#include <ros/callback_queue.h>
//#include <ros/advertise_options.h>

// Boost
//#include <boost/thread.hpp>
#include <mutex>
#include <thread>
//#include <boost/bind.hpp>

// Services
// #include <gazebo_ros_battery/SetTemperature.h>
// #include <gazebo_ros_battery/SetCharge.h>
// #include <gazebo_ros_battery/Reset.h>
#include "gazebo_ros_battery/srv/set_temperature.hpp"
#include "gazebo_ros_battery/srv/set_charge.hpp"
#include "gazebo_ros_battery/srv/reset.hpp"
// For std::unique_ptr, could be removed
#include <memory>



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TEMPERATURE CLIENT");
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_temp_client");
  rclcpp::Client<gazebo_ros_battery::srv::SetTemperature>::SharedPtr client =
    node->create_client<gazebo_ros_battery::srv::SetTemperature>("battery_plugin/set_temperature");

  auto request = std::make_shared<gazebo_ros_battery::srv::SetTemperature::Request>();
  request->temperature.data = 30;


  while (!client->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dun");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set temperature");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "!!CHARGE CLIENT!!");

  std::shared_ptr<rclcpp::Node> nodetwo = rclcpp::Node::make_shared("set_charge_client");
  rclcpp::Client<gazebo_ros_battery::srv::SetCharge>::SharedPtr clienttwo =
    nodetwo->create_client<gazebo_ros_battery::srv::SetCharge>("battery_plugin/set_charge");

  auto requesttwo = std::make_shared<gazebo_ros_battery::srv::SetCharge::Request>();
  requesttwo->charge.data = 10;


  while (!clienttwo->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto resulttwo = clienttwo->async_send_request(requesttwo);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(nodetwo, resulttwo) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dun2");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set temperature");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "!!RESET CLIENT!!");

  std::shared_ptr<rclcpp::Node> nodethree = rclcpp::Node::make_shared("reset_client");
  rclcpp::Client<gazebo_ros_battery::srv::Reset>::SharedPtr clientthree =
    nodethree->create_client<gazebo_ros_battery::srv::Reset>("battery_plugin/reset");

  auto requestthree = std::make_shared<gazebo_ros_battery::srv::Reset::Request>();


  while (!clientthree->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto resultthree = clientthree->async_send_request(requestthree);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(nodethree, resultthree) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dun2");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set temperature");
  }

  rclcpp::shutdown();
  return 0;
}
