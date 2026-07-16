#ifndef GLUON_HARDWARE_INTERFACE__GLUON_HARDWARE_INTERFACE_HPP_
#define GLUON_HARDWARE_INTERFACE__GLUON_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

// innfos-cpp-sdk
#include "actuatorcontroller.h"

namespace gluon_hardware_interface
{

/// ros2_control SystemInterface for the INNFOS Gluon arm, backed by innfos-cpp-sdk.
///
/// Expects, in the ros2_control URDF tag:
///  - a hardware-level <param name="ip_address">...</param> (the Gluon's LAN
///    address; INNFOS's UnifiedID model addresses actuators as an
///    (id, ipAddress) pair, defaults to "" if omitted, matching the SDK's own
///    default parameter),
///  - one <joint> per actuator, each with <param name="actuator_id">N</param>
///    matching its physical INNFOS actuator ID, e.g.:
///      <joint name="joint1">
///        <param name="actuator_id">1</param>
///        <command_interface name="position"/>
///        <state_interface name="position"/>
///        <state_interface name="velocity"/>
///        <state_interface name="effort"/>
///      </joint>
class GluonSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint state/command storage, indexed the same as info_.joints.
  // Exposed to ros2_control in SI units (rad, rad/s); converted to/from the
  // SDK's native units (revolutions, RPM, Amps) at the read()/write() boundary.
  std::vector<double> hw_positions_;      // rad
  std::vector<double> hw_velocities_;     // rad/s
  std::vector<double> hw_efforts_;        // A (raw motor current - no torque
                                           // constant available to convert to Nm)
  std::vector<double> hw_position_commands_;  // rad

  // joint index -> INNFOS actuator ID, read from each joint's "actuator_id" param.
  std::vector<uint8_t> actuator_ids_;

  // Shared LAN address for the whole arm (INNFOS's UnifiedID = id + ipAddress).
  // Read from the hardware-level "ip_address" param; defaults to "".
  std::string ip_address_;

  // Owned by ActuatorController's internal singleton lifecycle - not deleted
  // by us. Set once in on_configure() via ActuatorController::initController().
  ActuatorController * controller_ = nullptr;

  // ---------------------------------------------------------------------------
  // innfos-cpp-sdk adapter.
  //
  // Every direct call into libActuatorController.so is contained in these four
  // methods, based on the actual ActuatorController.h and actuatordefine.h
  // headers:
  //  - lookupActuators()/enableActuator()/disableActuator() return real
  //    status (bool / vector<UnifiedID>); ErrorsDefine::ERR_NONE == 0.
  //  - activateActuatorMode() returns void - no failure to detect. Uses
  //    Actuator::ActuatorMode::Mode_Profile_Pos (accel/decel-ramped position
  //    control, as opposed to the unramped Mode_Pos).
  //  - requestCVPValue() is a fire-and-forget async trigger, not a getter;
  //    we instead enable the SDK's own switchAutoRefresh() once and read
  //    cached values via getPosition/getVelocity/getCurrent(id, false, ip)
  //    each cycle, which doesn't block on a network round trip.
  // ---------------------------------------------------------------------------

  /// Looks up connected actuators, enables the ones we need, puts them into
  /// position-control mode, and turns on background auto-refresh of their
  /// current/velocity/position. Called from on_activate().
  bool innfos_lookup_and_enable();

  /// Disables all actuators we enabled. Called from on_deactivate().
  bool innfos_disable_all();

  /// Reads the SDK's cached current/velocity/position for one actuator
  /// (no network round trip - see switchAutoRefresh above), converting units.
  /// Called from read().
  bool innfos_read_state(uint8_t id, double & position, double & velocity, double & effort);

  /// Sends a target position (radians, converted to revolutions) to one
  /// actuator. Called from write().
  void innfos_write_position_command(uint8_t id, double position);

  // All six joints home simultaneously.
  bool innfos_home_all();
};

}  // namespace gluon_hardware_interface

#endif  // GLUON_HARDWARE_INTERFACE__GLUON_HARDWARE_INTERFACE_HPP_
