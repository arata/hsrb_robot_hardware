/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_
#define HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_

#include <memory>
#include <string>
#include <vector>

// #include <hardware_interface/base_interface.hpp>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>

#include "hsrb_hw_joint.hpp"

namespace hsrb_robot_hardware {
// TODO(Takeshita) Parameterization of servo communication settings
// TODO(Takeshita) Control table version check
// TODO(Takeshita) Test that it can be loaded as a plugin

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
// class HsrbHwBase : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
class HsrbHwBase : public hardware_interface::SystemInterface {
 public:
  virtual ~HsrbHwBase() = default;

  //hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type start() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

  // When it stops, it is time to kill the node, so it is not implemented
  hardware_interface::return_type stop() override { return hardware_interface::return_type::OK; }

 protected:
  // It's placed in an easy-to-reach place for testing.，It's not very good.
  boost::shared_ptr<hsrb_servomotor_protocol::INetwork> network_;
  boost::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> protocol_;

  std::vector<std::shared_ptr<ActiveJoint>> active_joints_;
};

class HsrbHW : public HsrbHwBase<hsrb_servomotor_protocol::ExxxNetwork, hsrb_servomotor_protocol::ExxxProtocol,
                                 JointCommunication, GripperCommunication, ActiveJoint, GripperActiveJoint> {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HsrbHW);
};

}  // namespace hsrb_robot_hardware

#endif/*HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_*/
