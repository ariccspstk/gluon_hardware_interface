#include "gluon_hardware_interface/gluon_hardware_interface.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace gluon_hardware_interface
{

namespace
{
constexpr char kLogger[] = "GluonSystemHardware";
constexpr double kTwoPi = 2.0 * M_PI;

// SDK units <-> ros2_control (SI) units.
inline double revolutions_to_radians(double rev) { return rev * kTwoPi; }
inline double radians_to_revolutions(double rad) { return rad / kTwoPi; }
inline double rpm_to_rad_per_sec(double rpm) { return rpm * kTwoPi / 60.0; }
}  // namespace

hardware_interface::CallbackReturn GluonSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Shared LAN address for the arm (INNFOS addresses actuators as an
  // (id, ipAddress) UnifiedID pair). Optional - "" is the SDK's own default,
  // which seems to work for a single Ethernet group.
  auto ip_it = info_.hardware_parameters.find("ip_address");
  ip_address_ = (ip_it != info_.hardware_parameters.end()) ? ip_it->second : "";

  const auto n_joints = info_.joints.size();
  hw_positions_.assign(n_joints, 0.0);
  hw_velocities_.assign(n_joints, 0.0);
  hw_efforts_.assign(n_joints, 0.0);
  hw_position_commands_.assign(n_joints, 0.0);
  actuator_ids_.assign(n_joints, 0);

  for (size_t i = 0; i < n_joints; ++i)
  {
    const auto & joint = info_.joints[i];

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' must have exactly one command interface: position.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto it = joint.parameters.find("actuator_id");
    if (it == joint.parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' is missing the required <param name=\"actuator_id\"> in the "
        "ros2_control URDF tag.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    try
    {
      actuator_ids_[i] = static_cast<uint8_t>(std::stoi(it->second));
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' has an invalid actuator_id '%s': %s",
        joint.name.c_str(), it->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ActuatorController is a singleton that must be explicitly initialized
  // before getInstance()/any other call is valid.
  controller_ = ActuatorController::initController();
  if (controller_ == nullptr)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "ActuatorController::initController() failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // No explicit teardown call is exposed in ActuatorController.h (the
  // singleton is torn down by its own internal GC on process exit).
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!innfos_lookup_and_enable())
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Failed to enable INNFOS actuators.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Let auto-refresh populate at least one round of cached values before we
  // seed commands from them, so the first write() doesn't jerk the arm.
  ActuatorController::processEvents();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double position = 0.0, velocity = 0.0, effort = 0.0;
    if (innfos_read_state(actuator_ids_[i], position, velocity, effort))
    {
      hw_positions_[i] = position;
      hw_velocities_[i] = velocity;
      hw_efforts_[i] = effort;
      hw_position_commands_[i] = position;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Gluon hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GluonSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  innfos_disable_all();
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Gluon hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GluonSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & name = info_.joints[i].name;
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GluonSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type GluonSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Pumps the SDK's internal event/callback queue - required for the
  // auto-refresh values (and any async callbacks) to actually update.
  ActuatorController::processEvents();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (!innfos_read_state(actuator_ids_[i], hw_positions_[i], hw_velocities_[i], hw_efforts_[i]))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "Actuator %u (joint '%s') is offline.",
        actuator_ids_[i], info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GluonSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    innfos_write_position_command(actuator_ids_[i], hw_position_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// innfos-cpp-sdk adapter implementations, against the real ActuatorController.h.
// ---------------------------------------------------------------------------

bool GluonSystemHardware::innfos_lookup_and_enable()
{
  Actuator::ErrorsDefine ec = Actuator::ERR_NONE;
  const auto discovered = controller_->lookupActuators(ec);

  for (auto id : actuator_ids_)
  {
    bool found = false;
    for (const auto & unified_id : discovered)
    {
      if (unified_id.actuatorID == id && unified_id.ipAddress == ip_address_)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger),
        "Actuator ID %u at '%s' was not found by lookupActuators() "
        "(error code 0x%x; 0x0 = ERR_NONE).",
        id, ip_address_.c_str(), static_cast<unsigned>(ec));
      return false;
    }

    if (!controller_->enableActuator(id, ip_address_))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "enableActuator(%u) failed.", id);
      return false;
    }

    // Profile position mode: confirmed as Mode_Profile_Pos in actuatordefine.h
    // (position control with an accel/decel ramp, as opposed to the plain
    // Mode_Pos). ActuatorMode is a plain enum in the Actuator namespace (not
    // an enum class), so it's Actuator::Mode_Profile_Pos, not
    // Actuator::ActuatorMode::Mode_Profile_Pos. activateActuatorMode()
    // returns void, so there's nothing to check here.
    controller_->activateActuatorMode(id, Actuator::Mode_Profile_Pos, ip_address_);

    // Turn on the SDK's own background polling of current/velocity/position,
    // so read() can take cached values (bRefresh=false) instead of blocking.
    // Default refresh interval is 1000ms per the SDK's own doc comment - far
    // too slow for a 100Hz control loop (see gluon_controllers.yaml's
    // update_rate) - bring it down to match, or read() will keep returning
    // the same stale value for ~100 cycles between actual updates.
    controller_->switchAutoRefresh(id, true, ip_address_);
    controller_->setAutoRefreshInterval(id, 10, ip_address_);  // ms, matches 100Hz
  }
  return true;
}

bool GluonSystemHardware::innfos_disable_all()
{
  bool all_ok = true;
  for (auto id : actuator_ids_)
  {
    controller_->switchAutoRefresh(id, false, ip_address_);
    if (!controller_->disableActuator(id, ip_address_))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "disableActuator(%u) failed.", id);
      all_ok = false;
    }
  }
  return all_ok;
}

bool GluonSystemHardware::innfos_read_state(
  uint8_t id, double & position, double & velocity, double & effort)
{
  if (!controller_->isOnline(id, ip_address_))
  {
    return false;
  }

  // bRefresh=false: read the SDK's cached value (kept current by the
  // switchAutoRefresh() enabled in innfos_lookup_and_enable()) rather than
  // blocking this real-time call on a network round trip.
  const double position_rev = controller_->getPosition(id, false, ip_address_);
  const double velocity_rpm = controller_->getVelocity(id, false, ip_address_);
  const double current_a = controller_->getCurrent(id, false, ip_address_);

  position = revolutions_to_radians(position_rev);
  velocity = rpm_to_rad_per_sec(velocity_rpm);
  effort = current_a;  // raw motor current in Amps; no torque constant available
  return true;
}

void GluonSystemHardware::innfos_write_position_command(uint8_t id, double position)
{
  controller_->setPosition(id, radians_to_revolutions(position), ip_address_);
}

}  // namespace gluon_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  gluon_hardware_interface::GluonSystemHardware, hardware_interface::SystemInterface)