#include "gluon_hardware_interface/gluon_hardware_interface.hpp"

#include <cmath>
#include <chrono>
#include <string>
#include <thread>
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

// Motor-shaft revolutions (what the INNFOS SDK's getPosition()/setPosition()
// actually report) vs. output-shaft revolutions (what the URDF/joint angles
// are defined in terms of). Confirmed from the legacy ROS 1 gluon_hw_interface
// (STEERING_GEAR_RATIO = 36, RAD_TO_POS/POS_TO_RAD macros) - the current code
// was previously missing this factor entirely, which is why raw SDK readings
// looked like multiple full revolutions instead of plausible joint angles.
// NOTE: applies the same ratio to all 6 joints, matching what the legacy code
// did. If any individual joint actually uses a different gear ratio, this
// needs to become a per-joint value instead of one shared constant.
constexpr double kGearRatio = 36.0;

// SDK units <-> ros2_control (SI) units.
inline double revolutions_to_radians(double rev) { return rev * kTwoPi / kGearRatio; }
inline double radians_to_revolutions(double rad) { return rad * kGearRatio / kTwoPi; }
inline double rpm_to_rad_per_sec(double rpm) { return rpm * kTwoPi / (60.0 * kGearRatio); }
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
  // read current position for the homing check below.
  ActuatorController::processEvents();

  if (!innfos_home_all())
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Homing failed; refusing to activate.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state and commands from the now-homed actuators, so the first
  // write() in the control loop doesn't jerk the arm.
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

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Gluon hardware activated and homed.");
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

  //UnifiedID is a structure composed of the actuator ID (actuatorID) and IP(ipAddress) of ECB(ECU)
  std::vector<ActuatorController::UnifiedID> uIDArray = discovered;

  //If the size of the uIDArray is greater than zero, the connected actuators have been found
  if(uIDArray.size() > 0)
  {
      for(auto uID : uIDArray)
      {
          cout << "Actuator ID: "<<(int)uID.actuatorID << " IP address: " << uID.ipAddress.c_str() << endl;
      }
  }
  else
  {
      //ec=0x803 Communication with ECB(ECU) failed
      //ec=0x802 Communication with actuator failed
      cout << "Connected error code:" << hex << ec << endl;
  }

  // First pass: confirm every actuator_ids_ entry was discovered on
  // ip_address_, and build the UnifiedID list to hand to
  // enableActuatorInBatch() in one shot below.
  std::vector<ActuatorController::UnifiedID> to_enable;
  to_enable.reserve(actuator_ids_.size());

  for (auto id : actuator_ids_)
  {
    bool found = false;
    for (const auto & unified_id : discovered)
    {
      if (unified_id.actuatorID == id && unified_id.ipAddress == ip_address_)
      {
        found = true;
        to_enable.push_back(unified_id);
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
  }

  // Batch-enable everything in a single SDK call instead of one
  // enableActuator() round trip per joint.
  if (!controller_->enableActuatorInBatch(to_enable))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "enableActuatorInBatch() failed.");
    return false;
  }

  // Second pass: per-actuator mode activation and auto-refresh setup, which
  // have no batch equivalents in the SDK.
  for (const auto & unified_id : to_enable)
  {
    // Profile position mode: confirmed as Mode_Profile_Pos in actuatordefine.h
    // (position control with an accel/decel ramp, as opposed to the plain
    // Mode_Pos). ActuatorMode is a plain enum in the Actuator namespace (not
    // an enum class), so it's Actuator::Mode_Profile_Pos, not
    // Actuator::ActuatorMode::Mode_Profile_Pos. activateActuatorMode()
    // returns void, so there's nothing to check here.
    controller_->activateActuatorMode(
      unified_id.actuatorID, Actuator::Mode_Profile_Pos, unified_id.ipAddress);

    // Turn on the SDK's own background polling of current/velocity/position,
    // so read() can take cached values (bRefresh=false) instead of blocking.
    // Default refresh interval is 1000ms per the SDK's own doc comment - far
    // too slow for a 100Hz control loop (see gluon_controllers.yaml's
    // update_rate) - bring it down to match, or read() will keep returning
    // the same stale value for ~100 cycles between actual updates.
    controller_->switchAutoRefresh(unified_id.actuatorID, true, unified_id.ipAddress);
    controller_->setAutoRefreshInterval(unified_id.actuatorID, 10, unified_id.ipAddress);  // ms, matches 100Hz
  }

  return true;
}

bool GluonSystemHardware::innfos_disable_all()
{
  // Turn off auto-refresh per actuator first (no batch equivalent exposed),
  // then flush the event queue before issuing the disable.
  for (auto id : actuator_ids_)
  {
    controller_->switchAutoRefresh(id, false, ip_address_);
  }
  ActuatorController::processEvents();

  // A known-working standalone tool disables successfully by re-running
  // lookupActuators() immediately beforehand, on a controller instance that
  // was never streaming continuous auto-refresh/setPosition traffic. Doing
  // the same re-lookup here, now that auto-refresh is off, to test whether
  // stale/queued traffic from the long-running session was the reason
  // disableAllActuators() alone wasn't taking effect in this process.
  Actuator::ErrorsDefine ec = Actuator::ERR_NONE;
  controller_->lookupActuators(ec);
  ActuatorController::processEvents();

  // NOTE: disableAllActuators() disables every actuator the controller has
  // discovered, not just the ones in actuator_ids_. Fine as long as this
  // ActuatorController instance is dedicated to this arm - if it's ever
  // shared with actuators outside actuator_ids_, this will disable those
  // too.
  controller_->disableAllActuators();
  ActuatorController::processEvents();

  // Don't just trust the return value - verify every actuator we actually
  // care about considers itself disabled. Retry a few times before giving
  // up, since this matters for safety (a "disabled" arm that's still
  // powered is a real hazard, not just a log nuisance).
  constexpr int kMaxAttempts = 5;
  int attempt = 1;
  auto any_still_enabled = [this]()
  {
    for (auto id : actuator_ids_)
    {
      if (controller_->isEnable(id, ip_address_)) return true;
    }
    return false;
  };

  // Give the disable command real time to take effect before checking -
  // the legacy ROS1 interface for this arm did the same (disableAllActuators()
  // followed by a flat 200ms sleep), suggesting disabling isn't instantaneous.
  // Re-calling disableAllActuators() again too quickly may just interrupt an
  // already in-flight disable sequence rather than help it along, so each
  // attempt gets a full settle window (with processEvents() pumped
  // throughout, since our isEnable()/heartbeat state needs that to update)
  // before we decide whether to retry.
  constexpr auto kSettleWindow = std::chrono::milliseconds(250);
  constexpr auto kPumpInterval = std::chrono::milliseconds(20);

  auto settle_and_check = [kSettleWindow, kPumpInterval, &any_still_enabled]()
  {
    const auto deadline = std::chrono::steady_clock::now() + kSettleWindow;
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(kPumpInterval);
      ActuatorController::processEvents();
    }
    return any_still_enabled();
  };

  while (settle_and_check() && attempt < kMaxAttempts)
  {
    // Log *why*, not just that it failed - if disable keeps getting
    // rejected, the actuator's own error code / current mode should tell us
    // whether it's refusing because it's still actively holding a Profile
    // Position setpoint, faulted, or something else entirely.
    for (auto id : actuator_ids_)
    {
      if (controller_->isEnable(id, ip_address_))
      {
        const auto mode = controller_->getActuatorMode(id, ip_address_);
        const auto err = controller_->getErrorCode(id, ip_address_);
        RCLCPP_WARN(
          rclcpp::get_logger(kLogger),
          "  actuator %u still enabled: mode=%d, error=0x%x",
          id, static_cast<int>(mode), static_cast<unsigned>(err));
      }
    }

    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "disableAllActuators() attempt %d did not take effect for all actuators after a "
      "%ld ms settle window; retrying.",
      attempt, static_cast<long>(kSettleWindow.count()));
    controller_->disableAllActuators();
    ActuatorController::processEvents();
    ++attempt;
  }

  bool all_ok = true;
  for (auto id : actuator_ids_)
  {
    if (controller_->isEnable(id, ip_address_))
    {
      // Escalate hard: this means the arm may still be powered and holding
      // position/torque after what the rest of the system believes is a
      // clean deactivation. Whoever is operating the arm needs to know.
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Actuator %u COULD NOT BE DISABLED after %d attempts - it may still be "
        "powered/enabled. Treat the arm as live until confirmed otherwise.",
        id, kMaxAttempts);
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

bool GluonSystemHardware::innfos_home_all()
{
  static constexpr double kHomeToleranceRad = 0.02;      // ~1.1 deg, per-joint "close enough"
  static constexpr auto kHomeTimeout = std::chrono::seconds(15);
  static constexpr auto kPumpInterval = std::chrono::milliseconds(20);

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Homing all actuators to zero position...");

  // Already in Mode_Profile_Pos (set during innfos_lookup_and_enable), so this
  // ramps smoothly using each actuator's profile accel/decel/max-velocity
  // settings rather than snapping instantly to 0.
  for (auto id : actuator_ids_)
  {
    innfos_write_position_command(id, 0.0);
  }

  const auto deadline = std::chrono::steady_clock::now() + kHomeTimeout;
  bool all_homed = false;

  while (std::chrono::steady_clock::now() < deadline)
  {
    ActuatorController::processEvents();

    all_homed = true;
    for (auto id : actuator_ids_)
    {
      double position = 0.0, velocity = 0.0, effort = 0.0;
      if (!innfos_read_state(id, position, velocity, effort))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger(kLogger),
          "Actuator %u went offline while homing.", id);
        return false;
      }
      if (std::abs(position) > kHomeToleranceRad)
      {
        all_homed = false;
      }
    }

    if (all_homed)
    {
      break;
    }
    std::this_thread::sleep_for(kPumpInterval);
  }

  if (!all_homed)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "Homing did not complete within %ld seconds.",
      static_cast<long>(kHomeTimeout.count()));
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "All actuators homed.");
  return true;
}

}

// namespace gluon_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  gluon_hardware_interface::GluonSystemHardware, hardware_interface::SystemInterface)