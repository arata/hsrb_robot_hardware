/*
Copyright (c) 2018 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_
#define HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <transmission_interface/simple_transmission.hpp>

#include "joint_communication.hpp"
#include "joint_parameters.hpp"

namespace hsrb_robot_hardware {
// TODO(Takeshita) Error handling
// TODO(Takeshita) Diagnostics
// TODO(Takeshita) Servo lock/unlock
// TODO(Takeshita) Alignment

struct JointValues {
  using Ptr = std::shared_ptr<JointValues>;

  std::string name;
  double pos;
  double vel;
  double eff;

  explicit JointValues(const std::string& _name) : name(_name), pos(0.0), vel(0.0), eff(0.0) {}

  template<typename TYPE>
  TYPE GeneratePositionHandle() {
    return TYPE(name, hardware_interface::HW_IF_POSITION, &pos);
  }
  template<typename TYPE>
  TYPE GenerateVelocityHandle() {
    return TYPE(name, hardware_interface::HW_IF_VELOCITY, &vel);
  }
  template<typename TYPE>
  TYPE GenerateEffortHandle() {
    return TYPE(name, hardware_interface::HW_IF_EFFORT, &eff);
  }
};

// ros2_control can only exchange double type, so define a util class that handles bool type
class BoolValue {
 public:
  BoolValue() { Set(false); }

  template<typename TYPE>
  TYPE GenerateHandle(const std::string& name, const std::string& interface) {
    return TYPE(name, interface, &value_);
  }

  bool Get() const { return value_ > 0; }
  void Set(bool value) {
    if (value) {
      value_ = 1.0;
    } else {
      value_ = -1.0;
    }
  }
  void Set(double value) { Set(value > 0.5); }

 private:
  double value_;
};

class ActiveJoint {
 public:
  using Ptr = std::shared_ptr<ActiveJoint>;

  ActiveJoint(const JointCommunication::Ptr& comm, const JointParameters& params);
  virtual ~ActiveJoint() = default;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces();
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  virtual void start();
  virtual void read();
  virtual void write();

 protected:
  JointCommunication::Ptr comm_;
  JointParameters params_;

  // Int is preferable because there is switch statement depending on the drive_mode
  // However, since StateInterface receives a double pointer, a double is also prepared
  int8_t drive_mode_;
  double drive_mode_out_;
  // If command_drive_mode is a negative value, assume no command value
  // It would be nice if we could use optional, but it can't be passed to CommandInterface
  double command_drive_mode_;

  JointValues::Ptr jnt_curr_;
  JointValues::Ptr act_curr_;
  std::shared_ptr<transmission_interface::Transmission> curr_transmission_;

  JointValues::Ptr jnt_cmd_;
  JointValues::Ptr act_cmd_;
  std::shared_ptr<transmission_interface::Transmission> cmd_transmission_;
};

class GripperActiveJoint : public ActiveJoint {
 public:
  using Ptr = std::shared_ptr<GripperActiveJoint>;

  GripperActiveJoint(const GripperCommunication::Ptr& comm, const JointParameters& params,
                     const GripperJointParameters& gripper_params);
  virtual ~GripperActiveJoint() = default;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void read() override;
  void write() override;

 private:
  GripperCommunication::Ptr gripper_comm_;
  GripperJointParameters gripper_params_;

  JointValues::Ptr jnt_curr_left_;
  JointValues::Ptr jnt_curr_right_;

  double force_cmd_;
  double effort_cmd_;

  BoolValue grasping_flag_curr_;
  BoolValue grasping_flag_cmd_;
};

}  // namespace hsrb_robot_hardware

#endif/*HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_*/
