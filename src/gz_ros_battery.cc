#include <gazebo/gazebo.hh>
#include <assert.h>
#include <sdf/sdf.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros_battery/gazebo_ros_battery.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace gazebo_plugins
{

// Finalize the controller
// void GazeboRosBattery::FiniChild() {
// 	  int pass;
//     //do nothing for now
//     //executor_->cancel();
//     //executor_thread_.join();
//     //gazebo_ros_->node()->shutdown(); //maybe replace with new way?
// }
/// Class to hold private data members (PIMPL pattern)

class GazeboRosBatteryPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  //GazeboRosPtr gazebo_ros_;
  gazebo::physics::ModelPtr parent;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_publisher_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> current_draw_subscribers_;
  sensor_msgs::msg::BatteryState battery_state_;
  gazebo::common::Time last_update_time_;
  std::string battery_topic_;
  std::string consumer_topic_;
  std::string battery_voltage_topic_;
  std::string frame_id_;
  std::string plugin_name_;
  std::vector<double> drawn_currents_;
  // common parameters
  bool publish_voltage_;
  int technology_;
  int num_of_consumers_;
  int cell_count_;
  double update_rate_;
  double update_period_;
  double design_capacity_;
  double current_drawn_;
  double nominal_voltage_;
  double constant_voltage_;
  double cut_off_voltage_;
  double internal_resistance_;
  double lpf_tau_;
  // linear model params
  double lin_discharge_coeff_;
  bool use_nonlinear_model_;
  // nonlinear model params
  double polarization_constant_; // polarization constant [V/Ah] or pol. resistance [Ohm]
  double exponential_voltage_;   // exponential voltage [V]
  double exponential_capacity_;  // exponential capacity [1/(Ah)]
  double characteristic_time_;   // characteristic time [s] for charge-dependence
  double design_temperature_;         // Design temperature where pol. const is unchanged and voltages are nominal.
  double arrhenius_rate_polarization_; // Arrhenius rate of polarization constant [K]
  double capacity_temp_coeff_;      // Temperature dependence of capacity [Ah/K]
  double reversible_voltage_temp_;  // Linear temperature dependant voltage shift [V/K]
  // internal variables
  bool model_initialised_;
  bool internal_cutt_off_;
  bool battery_empty_;
  double voltage_;
  double charge_;
  double charge_memory_;
  double current_;
  double discharge_;
  double capacity_;
  double temperature_;
  double temp_set_;
  double temp_lpf_tau_;
  double current_lpf_;
  std::mutex service_lock;

  // model update
  void linear_discharge_voltage_update();
  void nonlinear_discharge_voltage_update();

  // Services
  void SetCharge(const std::shared_ptr<gazebo_ros_battery::srv::SetCharge::Request> req, 
                                        std::shared_ptr<gazebo_ros_battery::srv::SetCharge::Response> res);

  void ResetModel(const std::shared_ptr<gazebo_ros_battery::srv::Reset::Request> req,
                                        std::shared_ptr<gazebo_ros_battery::srv::Reset::Response> res);


  void SetTemperature(const std::shared_ptr<gazebo_ros_battery::srv::SetTemperature::Request> req,
                                        std::shared_ptr<gazebo_ros_battery::srv::SetTemperature::Response> res);
                    
  // Callback Queue
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread callback_queue_thread_;
  std::mutex lock_;
  void QueueThread();
  void currentCallback(const std_msgs::msg::Float32::SharedPtr current, int consumer_id);
};

GazeboRosBattery::GazeboRosBattery()
: impl_(std::make_unique<GazeboRosBatteryPrivate>())
{
}

GazeboRosBattery::~GazeboRosBattery()
{
}

void GazeboRosBattery::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf)
{

	// Create a GazeboRos node instead of a common ROS node.
	// Pass it SDF parameters so common options like namespace and remapping
	// can be handled.
	this->model_ptr = model;
	impl_->plugin_name_ = "battery_plugin";

	impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);


	// Create a connection so the OnUpdate function is called at every simulation
	// iteration. Remove this call, the connection and the callback if not needed.
	impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
	std::bind(&GazeboRosBattery::OnUpdate, this));

	impl_->parent = model;
	const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

	// topic params
	impl_->num_of_consumers_ = _sdf->Get<int>("num_of_consumers", 2).first;

	impl_->consumer_topic_ = _sdf->Get<std::string>("consumer_topic", "/battery/consumer").first;
	impl_->frame_id_ = _sdf->Get<std::string>("frame_id", "battery" ).first;
	impl_->battery_topic_ = _sdf->Get<std::string>("battery_topic", "/battery/status" ).first;

	impl_->publish_voltage_ = _sdf->Get<bool>("publish_voltage", true).first;
	impl_->battery_voltage_topic_ = "none";
	if (impl_->publish_voltage_){
		impl_->battery_voltage_topic_ = _sdf->Get<std::string>("battery_voltage_topic", "/battery/voltage").first;
	}

	// common parameters
	impl_->technology_ = _sdf->Get<int>("technology", sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION ).first;
	impl_->cell_count_ = _sdf->Get<int>("number_of_cells", 8 ).first;
	impl_->update_rate_ = _sdf->Get<double>("update_rate", 10.0 ).first;
	impl_->design_capacity_ = _sdf->Get<double>("design_capacity", 4.0 ).first;
	impl_->nominal_voltage_ = _sdf->Get<double>("nominal_voltage", 24.0 ).first;
	impl_->cut_off_voltage_ = _sdf->Get<double>("cut_off_voltage", 18.0 ).first;
	impl_->constant_voltage_ =  _sdf->Get<double>("full_charge_voltage", 24.2 ).first;
	impl_->lpf_tau_ = _sdf->Get<double>("current_filter_tau", 1.0 ).first;
	impl_->temp_lpf_tau_ = _sdf->Get<double>("temperature_response_tau", 0.5 ).first;
	impl_->internal_resistance_ = _sdf->Get<double>("internal_resistance", 0.05 ).first;

	// model specific params
	impl_->use_nonlinear_model_ = _sdf->Get<bool>("use_nonlinear_model", true ).first;
	if (!impl_->use_nonlinear_model_) {
		impl_->lin_discharge_coeff_ = _sdf->Get<double>("linear_discharge_coeff", -1.0 ).first;
	}
	else {
	impl_->polarization_constant_ = _sdf->Get<double>("polarization_constant", 0.07 ).first;
	impl_->exponential_voltage_ = _sdf->Get<double>("exponential_voltage", 0.7 ).first;
	impl_->exponential_capacity_ = _sdf->Get<double>("exponential_capacity", 3.0 ).first;
	impl_->characteristic_time_ = _sdf->Get<double>("characteristic_time", 200.0 ).first;
	impl_->design_temperature_ = _sdf->Get<double>("design_temperature", 25.0 ).first;
	impl_->arrhenius_rate_polarization_ = _sdf->Get<double>("arrhenius_rate_polarization", 500.0 ).first;
	impl_->reversible_voltage_temp_ = _sdf->Get<double>("reversible_voltage_temp", 0.05 ).first;
	impl_->capacity_temp_coeff_ = _sdf->Get<double>("capacity_temp_coeff", 0.01 ).first;
	}

	impl_->battery_state_.header.frame_id = impl_->frame_id_;
	impl_->battery_state_.power_supply_technology = impl_->technology_;
	impl_->battery_state_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
	impl_->battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	impl_->battery_state_.design_capacity = impl_->design_capacity_;
	impl_->battery_state_.capacity = impl_->design_capacity_;
	// battery_state_.capacity   = last_full_capacity_;
	impl_->battery_state_.present = true;
	impl_->battery_state_.temperature = impl_->temperature_ = impl_->design_temperature_;
	// start
	if ( impl_->update_rate_ > 0.0 ) impl_->update_period_ = 1.0 / impl_->update_rate_; else impl_->update_period_ = 0.0;
	impl_->last_update_time_ = impl_->parent->GetWorld()->SimTime();

	// subscribers
	//std::vector<rclcpp::SubscriptionOptions> subscribe_options;
	//ROS_INFO_NAMED(plugin_name_, "%s: Creating %d consumer subscribers:", gazebo_ros_->info(), num_of_consumers_);
	//callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	RCLCPP_INFO(impl_->ros_node_->get_logger(), "%s: Creating %d consumer subscribers: %s", impl_->plugin_name_.c_str(), impl_->num_of_consumers_, impl_->consumer_topic_.c_str());
	//works till here

	for (int i=0; i<impl_->num_of_consumers_; i++)
	{
	//rclcpp::SubscriptionOptions subscription_options;
	//subscription_options.callback_group = callback_group_; possibly this needs to be added back and assigned to each subscription.. but suppossedly it is internally handled now

	
		std::function<void(const std_msgs::msg::Float32::SharedPtr)> bound_callback_func = std::bind(&GazeboRosBatteryPrivate::currentCallback, impl_.get(), std::placeholders::_1, i); //workaround to allow extra arguments to callback function


		impl_->current_draw_subscribers_.push_back(impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(impl_->consumer_topic_ + "/c" + std::to_string(i), qos.get_subscription_qos(impl_->consumer_topic_ + "/c" + std::to_string(i),rclcpp::QoS(1)),bound_callback_func));	
	//impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(impl_->consumer_topic_ + "/c" + std::to_string(0),qos.get_subscription_qos(impl_->consumer_topic_ + "/c" + std::to_string(i),rclcpp::QoS(1)),std::bind(&GazeboRosBatteryPrivate::currentCallback, impl_.get(), std::placeholders::_1));																			
	//std::string consumer_topic_i = impl_->consumer_topic_ + "/" + std::to_string(0);
	//RCLCPP_INFO(impl_->ros_node_->get_logger(), "Yea %s", consumer_topic_i.c_str());
	//impl_->ros_node_->create_subscription<std_msgs::msg::Float32>("something/something/s0",qos.get_subscription_qos("something/something/s0",rclcpp::QoS(1)),std::bind(&GazeboRosBatteryPrivate::currentCallback, impl_.get(), std::placeholders::_1));																			
	
	  //impl_->current_draw_subscribers_.push_back(impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(impl_->consumer_topic_ + "/" + std::to_string(0), qos.get_subscription_qos(impl_->consumer_topic_ + "/" + std::to_string(i), rclcpp::QoS(1)),bound_callback_func));
												  //impl_->ros_node_->create_subscription<std_msgs::msg::Float32>("cmd_vel", 									  qos.get_subscription_qos("cmd_vel",                                       rclcpp::QoS(1)),std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

	
	
	

	//current_draw_subscribers_.push_back(impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(consumer_topic_ + "/" + std::to_string(0),qos.get_subscription_qos(consumer_topic_ + "/" + std::to_string(0), rclcpp::QoS(1)), std::bind(&GazeboRosBattery::currentCallback, impl_.get(), std::placeholders::_1, 0)));
									//create_subscription<std_msgs::msg::Empty>(trigger_topic, qos.get_subscription_qos(trigger_topic, rclcpp::QoS(1)),std::bind(&GazeboRosCamera::OnTrigger, this, std::placeholders::_1));
	//RCLCPP_INFO(get_logger(), "%s: Listening to consumer on: %s", plugin_name_.c_str(), (consumer_topic_ + "/" + std::to_string(i)).c_str()); replace with gzmsg maybe
		impl_->drawn_currents_.push_back(0);
	}

	// publishers
	//battery_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::BatteryState>(battery_topic_, 1);
	impl_->battery_state_publisher_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(impl_->battery_topic_, 1);

	RCLCPP_INFO(impl_->ros_node_->get_logger(), "%s: Advertising battery state on %s ", impl_->plugin_name_.c_str(), impl_->battery_topic_.c_str());
	if (impl_->publish_voltage_){
		impl_->battery_voltage_publisher_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(impl_->battery_voltage_topic_, 1);
		RCLCPP_INFO(impl_->ros_node_->get_logger(),"%s: Advertising battery voltage on %s ", impl_->plugin_name_.c_str(), impl_->battery_voltage_topic_.c_str());
	}

	// // start custom queue
	//this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosBattery::QueueThread, this ) );
	//executor_->add_node(this);
	//executor_thread_ = std::thread(std::bind(&GazeboRosBattery::QueueThread, this));

	// services
	//this->set_charge_state = impl_->ros_node_->create_service<gazebo_ros_battery::srv::SetCharge>(plugin_name_ + "/set_charge", std::bind(&GazeboRosBattery::SetCharge, this, std::placeholders::_1, std::placeholders::_2));os_node_->create_service<gazebo_ros_battery::srv::SetCharge>(plugin_name_ + "/set_charge", std::bind(&GazeboRosBattery::SetCharge, this, std::placeholders::_1, std::placeholders::_2));
	//impl_->ros_node_->create_service<gazebo_ros_battery::srv::SetCharge>("batteryplugin/set_charge", &GazeboRosBattery::SetCharge);
	//impl_->ros_node_->create_service<gazebo_ros_battery::srv::SetCharge>(plugin_name_ + "/set_charge", std::bind(&GazeboRosBattery::SetCharge, impl_->ros_node_, std::placeholders::_1, std::placeholders::_2));
	this->set_charge_state = impl_->ros_node_->create_service<gazebo_ros_battery::srv::SetCharge>(impl_->plugin_name_ + "/set_charge",           std::bind(&GazeboRosBatteryPrivate::SetCharge, impl_.get(),std::placeholders::_1, std::placeholders::_2));
	this->set_temperature =  impl_->ros_node_->create_service<gazebo_ros_battery::srv::SetTemperature>(impl_->plugin_name_ + "/set_temperature", std::bind(&GazeboRosBatteryPrivate::SetTemperature, impl_.get(), std::placeholders::_1, std::placeholders::_2));
	this->reset_model =      impl_->ros_node_->create_service<gazebo_ros_battery::srv::Reset>(impl_->plugin_name_ + "/reset",                    std::bind(&GazeboRosBatteryPrivate::ResetModel, impl_.get(), std::placeholders::_1, std::placeholders::_2));

	impl_->model_initialised_ = false;
	Reset();
}


void GazeboRosBatteryPrivate::SetTemperature(const std::shared_ptr<gazebo_ros_battery::srv::SetTemperature::Request> req, std::shared_ptr<gazebo_ros_battery::srv::SetTemperature::Response> res)
{
  service_lock.lock();
  this->temp_set_ = req->temperature.data;
	//ROS_WARN_NAMED(plugin_name_, "%s: Temperature set: %f", gazebo_ros_->info(), temp_set_);
  service_lock.unlock();
  //return true;
}

void GazeboRosBatteryPrivate::SetCharge(const std::shared_ptr<gazebo_ros_battery::srv::SetCharge::Request> req, std::shared_ptr<gazebo_ros_battery::srv::SetCharge::Response> res)
{
  service_lock.lock();
  this->discharge_ = req->charge.data;
  RCLCPP_WARN_STREAM(this->ros_node_->get_logger(), this->plugin_name_ << ": Charge set: "<< this->design_capacity_ + this->discharge_);
  
	//ROS_WARN_NAMED(plugin_name_, "%s: Charge set: %f", gazebo_ros_->info(), design_capacity_ + discharge_);
  service_lock.unlock();
  //return true;
}

void GazeboRosBatteryPrivate::ResetModel(const std::shared_ptr<gazebo_ros_battery::srv::Reset::Request> req, std::shared_ptr<gazebo_ros_battery::srv::Reset::Response> res)
{
	service_lock.lock();

	this->last_update_time_ = parent->GetWorld()->SimTime();
	this->charge_ = design_capacity_;
	this->voltage_ = constant_voltage_;
	this->temperature_ = design_temperature_;
	this->temp_set_ = design_temperature_;
	this->discharge_ = 0;
	this->current_drawn_ = 0;
	this->battery_empty_ = false;
	this->internal_cutt_off_ = false;
	this->model_initialised_ = true;

	service_lock.unlock();
	//return true;
}

void GazeboRosBattery::Reset() {
  	impl_->last_update_time_ = this->model_ptr->GetWorld()->SimTime();
	impl_->charge_ = impl_->design_capacity_;
	impl_->voltage_ = impl_->constant_voltage_;
	impl_->temperature_ = impl_->design_temperature_;
	impl_->temp_set_ = impl_->design_temperature_;
	impl_->discharge_ = 0;
	impl_->current_drawn_ = 0;
	impl_->battery_empty_ = false;
	impl_->internal_cutt_off_ = false;
	impl_->model_initialised_ = true;
	RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), impl_->plugin_name_ << ": Battery model reset.");
	RCLCPP_INFO(impl_->ros_node_->get_logger(), "model_initialised: %s", impl_->model_initialised_?"true":"false"
	);

	
}





// simple linear discharge model
void GazeboRosBatteryPrivate::linear_discharge_voltage_update() {
	RCLCPP_INFO(this->ros_node_->get_logger(), "then here 1");
	this->voltage_ = this->constant_voltage_ + this->lin_discharge_coeff_ * (1 - this->discharge_ / this->design_capacity_) - this->internal_resistance_ * this->current_lpf_;
	RCLCPP_INFO(this->ros_node_->get_logger(), "voltage here 1 %f", this->voltage_);

}


void GazeboRosBatteryPrivate::nonlinear_discharge_voltage_update() {
	RCLCPP_INFO(this->ros_node_->get_logger(), "then here 2 vltg: %f ",this->voltage_);
	double t = this->temperature_+273.15;
	double t0 = this->design_temperature_+273.15;
	double E0T = this->constant_voltage_ + this->reversible_voltage_temp_*(t-t0); // TODO: Don't increase for t>t0 ?
	double QT = this->design_capacity_ + this->capacity_temp_coeff_*(t-t0);       // TODO: Don't increase for t>t0 ?
	double KT = this->polarization_constant_ * exp(this->arrhenius_rate_polarization_*(1/t-1/t0));
    this->voltage_ = E0T
             - KT * QT/(QT + this->discharge_) * (this->current_lpf_*(this->characteristic_time_/3600.0) - this->discharge_)
             + this->exponential_voltage_*exp(-this->exponential_capacity_*-this->discharge_);
	RCLCPP_INFO(this->ros_node_->get_logger(), "voltage here 2 %f", this->voltage_);

}

// Temperature dependent parameters:
//    QT = Q + dQdT*(t-t0)
//    KT = K * np.exp(Kalpha*(1/t-1/t0))
//    E0T = E0 + Tshift*(t-t0)
// v(i) = E0T - KT*QT/(QT-it)*(i*Ctime/3600.0+it) + A*np.exp(-B*it)
//
//  where
//  E0: constant voltage [V]
//  K:  polarization constant [V/Ah] or pol. resistance [Ohm]
//  Q:  maximum capacity [Ah]
//  A:  exponential voltage [V]
//  B:  exponential capacity [A/h]
//  it: already extracted capacity [Ah] (= - discharge)
//  id: current * characteristic time [Ah]
//  i:  battery current [A]
//  T0: design temperature [Celsius]
// Tpol: name of the Vulkan first officer aboard Enterprise NX-01 [Celsius]
// Tshift: temperature offset [Celsius]


void GazeboRosBattery::OnUpdate()
{
  // Do something every simulation iteration
	//RCLCPP_INFO(impl_->ros_node_->get_logger(), "up");
	RCLCPP_INFO(impl_->ros_node_->get_logger(), "voltage here update %f", impl_->voltage_);

    gazebo::common::Time current_time = this->model_ptr->GetWorld()->SimTime();
    double dt = ( current_time - impl_->last_update_time_ ).Double();
	//RCLCPP_INFO(impl_->ros_node_->get_logger(), "dt: %f model_initialised: %s", dt, impl_->model_initialised_);
    if ( dt > impl_->update_period_ && impl_->model_initialised_ ) {
				RCLCPP_INFO(impl_->ros_node_->get_logger(), "here");

				double n = dt / impl_->temp_lpf_tau_;
				impl_->temperature_ = impl_->temperature_ + n * (impl_->temp_set_ - impl_->temperature_);

				if (!impl_->battery_empty_) {
			    	// LPF filter on current
					double k = dt / impl_->lpf_tau_;
					impl_->current_lpf_ = impl_->current_lpf_ + k * (impl_->current_drawn_ - impl_->current_lpf_);
					RCLCPP_INFO(impl_->ros_node_->get_logger(), "current_lpf: %f", impl_->current_lpf_);
					
					// Accumulate discharge (negative: 0 to design capacity)
					impl_->discharge_ = impl_->discharge_ - GZ_SEC_TO_HOUR(dt * impl_->current_lpf_);
					if (!impl_->internal_cutt_off_) {
						if (impl_->use_nonlinear_model_){
							impl_->nonlinear_discharge_voltage_update();
						}
						else {
							impl_->linear_discharge_voltage_update();
						}
						RCLCPP_INFO(impl_->ros_node_->get_logger(), "voltage here after discharge: %f", impl_->voltage_);

						impl_->charge_ = impl_->design_capacity_ + impl_->discharge_; // discharge is negative
						impl_->charge_memory_ = impl_->charge_;
					}
					if (impl_->voltage_<=impl_->cut_off_voltage_ && !impl_->internal_cutt_off_) {
						impl_->discharge_ = 0;
						impl_->voltage_ = 0;
						impl_->internal_cutt_off_ = true;
						impl_->charge_ = impl_->charge_memory_;
						RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), impl_->plugin_name_ << ": Battery voltage cut off.");
					}
				}

				if (!impl_->battery_empty_ && impl_->charge_<=0) {
					impl_->discharge_ = 0;
					impl_->current_lpf_ = 0;
					impl_->voltage_ = 0;
					impl_->battery_empty_ = true;
					RCLCPP_WARN_STREAM(impl_->ros_node_->get_logger(), impl_->plugin_name_ << ": Battery discharged.");
				}
				// state update
				impl_->battery_state_.header.stamp = impl_->ros_node_->get_clock()->now();
				impl_->battery_state_.voltage    = impl_->voltage_;
				impl_->battery_state_.current    = impl_->current_lpf_;
				impl_->battery_state_.charge     = impl_->charge_;
				impl_->battery_state_.percentage = (impl_->charge_/impl_->design_capacity_)*100;
				impl_->battery_state_.temperature = impl_->temperature_;
				impl_->battery_state_publisher_->publish(impl_->battery_state_ );
				std_msgs::msg::Float32 battery_voltage_;
				battery_voltage_.data = impl_->voltage_;
				if (impl_->publish_voltage_) impl_->battery_voltage_publisher_->publish(battery_voltage_);
				impl_->last_update_time_+= gazebo::common::Time(impl_->update_period_);
    }
}

void GazeboRosBatteryPrivate::currentCallback(const std_msgs::msg::Float32::SharedPtr current, int consumer_id) {
	// int consumer_id = 0;
	current_drawn_ = 0;
	drawn_currents_[consumer_id]=current->data;
	for (int i=0; i<num_of_consumers_; i++) current_drawn_+=drawn_currents_[i];
	// ROS_WARN("Total current drawn: %f", current_drawn_);

	// temporary solution for simple charging
	if (current_drawn_ <=0){
		battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		if (internal_cutt_off_) voltage_ = cut_off_voltage_ + 0.05;
		internal_cutt_off_ = false;
	}
	else {
		battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

//possibly needs to be re-added later.
// void GazeboRosBattery::QueueThread()  {
//     static const double timeout = 0.01;
//     while (rclcpp::ok())
//     {
//       executor_->spin_some();
//       std::this_thread::sleep_for(std::chrono::duration<double>(timeout));
//     }
//   }

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosBattery)
}  // namespace gazebo_plugins