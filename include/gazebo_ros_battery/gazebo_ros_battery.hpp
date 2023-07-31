#ifndef GAZEBO_ROS_BATTERY_HPP_
#define GAZEBO_ROS_BATTERY_HPP_

#include <gazebo/common/Plugin.hh>

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
//#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
// #include <ros/ros.h>
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

namespace gazebo_plugins
{
// Forward declaration of private data class.
class GazeboRosBatteryPrivate;

/// Example ROS-powered Gazebo plugin with some useful boilerplate.
/// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
/// System, Visual, GUI, World, Sensor, etc.
class GazeboRosBattery : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosBattery();

  /// Destructor
  virtual ~GazeboRosBattery();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;
  void Reset();
  gazebo::physics::ModelPtr model_ptr;


protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

  //virtual void FiniChild();
  rclcpp::Service<gazebo_ros_battery::srv::SetTemperature>::SharedPtr set_temperature;
  rclcpp::Service<gazebo_ros_battery::srv::SetCharge>::SharedPtr set_charge_state{nullptr};
  rclcpp::Service<gazebo_ros_battery::srv::Reset>::SharedPtr reset_model;
  

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboRosBatteryPrivate> impl_;

      
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_ROS_BATTERY_HPP_
