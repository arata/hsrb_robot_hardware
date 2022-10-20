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
#include "hsrb_hw_joint.hpp"

#include <hardware_interface/handle.hpp>

#include <hsrb_servomotor_protocol/exxx_common.hpp>

namespace {
}  // unnamed namespace

namespace hsrb_robot_hardware {

std::shared_ptr<transmission_interface::SimpleTransmission> ConfigureTransmission(
    double reduction, double offset, JointValues::Ptr jnt, JointValues::Ptr act) {
  auto transmission = std::make_shared<transmission_interface::SimpleTransmission>(reduction, offset);
  transmission->configure({jnt->GeneratePositionHandle<transmission_interface::JointHandle>(),
                           jnt->GenerateVelocityHandle<transmission_interface::JointHandle>(),
                           jnt->GenerateEffortHandle<transmission_interface::JointHandle>()},
                          {act->GeneratePositionHandle<transmission_interface::ActuatorHandle>(),
                           act->GenerateVelocityHandle<transmission_interface::ActuatorHandle>(),
                           act->GenerateEffortHandle<transmission_interface::ActuatorHandle>()});
  return transmission;
}

void ResetCommand(uint8_t drive_mode, JointCommunication::Ptr& comm, JointValues& act,
                  transmission_interface::Transmission& trans) {
  switch (drive_mode) {
    case hsrb_servomotor_protocol::kDriveModePosition:
    case hsrb_servomotor_protocol::kDriveModeActPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeImpedance: {
      comm->GetCurrentPosition(kMotorAxis, act.pos);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity: {
      comm->GetCurrentPosition(kOutputAxis, act.pos);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeVelocity: {
      act.vel = 0.0;
      break;
    }
    default:
      break;
  }
  trans.actuator_to_joint();
}


ActiveJoint::ActiveJoint(const JointCommunication::Ptr& comm,
                         const JointParameters& params)
    : comm_(comm), params_(params), drive_mode_(0), drive_mode_out_(0.0), command_drive_mode_(-1.0) {
  jnt_curr_ = std::make_shared<JointValues>(params.joint_name);
  act_curr_ = std::make_shared<JointValues>(params.joint_name);
  curr_transmission_ = ConfigureTransmission(
      params.reduction, params.position_offset, jnt_curr_, act_curr_);

  jnt_cmd_ = std::make_shared<JointValues>(params.joint_name);
  act_cmd_ = std::make_shared<JointValues>(params.joint_name);
  cmd_transmission_ = ConfigureTransmission(
      params.reduction, params.position_offset, jnt_cmd_, act_cmd_);
}

std::vector<hardware_interface::StateInterface> ActiveJoint::export_state_interfaces() {
  // TODO(Takeshita) Use info_.state_interfaces[j].name
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(jnt_curr_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(params_.joint_name, "current_drive_mode", &drive_mode_out_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActiveJoint::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(jnt_cmd_->GeneratePositionHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(jnt_cmd_->GenerateVelocityHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_drive_mode", &command_drive_mode_));
  return command_interfaces;
}

void ActiveJoint::start() {
  comm_->ResetAlarm();

  drive_mode_ = params_.default_drive_mode;
  drive_mode_out_ = static_cast<double>(drive_mode_);
  comm_->ResetDriveMode(drive_mode_);

  comm_->ResetVelocityLimit(params_.velocity_limit);
  ResetCommand(drive_mode_, comm_, *act_cmd_, *cmd_transmission_);
  if (params_.IsZeroReset()) {
    comm_->ResetPosition();
  }
}

void ActiveJoint::read() {
  JointCommunication::ReadValues values;
  const auto error = comm_->Read(values);
  if (error && error.category() == boost::system::system_category()) {
    return;
  }

  drive_mode_ = static_cast<uint8_t>(values.drive_mode);
  drive_mode_out_ = static_cast<double>(drive_mode_);

  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
      act_curr_->pos = values.joint_calc_outaxis_correct_position;
      act_curr_->vel = values.motor_outaxis_velocity;
      break;
    default:
      act_curr_->pos = values.motor_outaxis_position;
      act_curr_->vel = values.motor_outaxis_velocity;
      break;
  }
  act_curr_->eff = values.current / params_.motor_to_joint_gear_ratio * params_.torque_constant;

  curr_transmission_->actuator_to_joint();

  jnt_cmd_->vel = 0.0;
}

void ActiveJoint::write() {
  cmd_transmission_->joint_to_actuator();

  boost::system::error_code error;
  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntVelocity: {
      error = comm_->WriteCommandVelocity(act_cmd_->vel);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModePosition:
    case hsrb_servomotor_protocol::kDriveModeActPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeImpedance: {
      error = comm_->WriteCommandPosition(act_cmd_->pos);
      break;
    }
    default: {
      break;
    }
  }

  auto command_drive_mode = static_cast<int8_t>(command_drive_mode_);
  if (command_drive_mode >= 0) {
    error = comm_->SetDriveMode(static_cast<uint8_t>(command_drive_mode));
    command_drive_mode_ = -1.0;
  }
}


GripperActiveJoint::GripperActiveJoint(const GripperCommunication::Ptr& comm,
                                       const JointParameters& params,
                                       const GripperJointParameters& gripper_params)
    : ActiveJoint(comm, params), gripper_comm_(comm), gripper_params_(gripper_params) {
  jnt_curr_left_ = std::make_shared<JointValues>(gripper_params.left_spring_joint);
  jnt_curr_right_ = std::make_shared<JointValues>(gripper_params.right_spring_joint);
}

std::vector<hardware_interface::StateInterface> GripperActiveJoint::export_state_interfaces() {
  auto state_interfaces = ActiveJoint::export_state_interfaces();
  state_interfaces.emplace_back(jnt_curr_left_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_left_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_left_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(grasping_flag_curr_.GenerateHandle<hardware_interface::StateInterface>(
      params_.joint_name, "current_grasping_flag"));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperActiveJoint::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(jnt_cmd_->GeneratePositionHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_drive_mode", &command_drive_mode_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, hardware_interface::HW_IF_EFFORT, &effort_cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_force", &force_cmd_));
  command_interfaces.emplace_back(grasping_flag_cmd_.GenerateHandle<hardware_interface::CommandInterface>(
      params_.joint_name, "command_grasping_flag"));
  return command_interfaces;
}

void GripperActiveJoint::read() {
  GripperCommunication::GripperValues values;
  const auto error = gripper_comm_->Read(values);
  if (error && error.category() == boost::system::system_category()) {
    return;
  }

  drive_mode_ = static_cast<uint8_t>(values.drive_mode);
  drive_mode_out_ = static_cast<double>(drive_mode_);
  grasping_flag_curr_.Set(values.hand_grasping_flag);

  act_curr_->pos = values.hand_motor_position;
  act_curr_->vel = values.hand_motor_velocity;
  act_curr_->eff = values.current / params_.motor_to_joint_gear_ratio * params_.torque_constant;

  jnt_curr_left_->pos = values.hand_left_position - act_curr_->pos;
  jnt_curr_left_->vel = values.hand_left_velocity;
  jnt_curr_left_->eff = values.hand_left_force;

  jnt_curr_right_->pos = values.hand_right_position - act_curr_->pos;
  jnt_curr_right_->vel = values.hand_right_velocity;
  jnt_curr_right_->eff = values.hand_right_force;

  curr_transmission_->actuator_to_joint();
}

void GripperActiveJoint::write() {
  cmd_transmission_->joint_to_actuator();

  boost::system::error_code error;
  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeHandGrasp: {
      if (grasping_flag_cmd_.Get() && !grasping_flag_curr_.Get()) {
        error = gripper_comm_->WriteGraspingFlag(true);
      }
      if (error) {
        break;
      }
      error = gripper_comm_->WriteCommandEffort(effort_cmd_);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeHandPosition: {
      error = comm_->WriteCommandPosition(act_cmd_->pos);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeHandSE: {
      error = gripper_comm_->WriteCommandForce(force_cmd_);
      break;
    }
    default: {
      break;
    }
  }

  auto command_drive_mode = static_cast<int8_t>(command_drive_mode_);
  if (command_drive_mode >= 0) {
    error = comm_->SetDriveMode(static_cast<uint8_t>(command_drive_mode));
    command_drive_mode_ = -1.0;
  }
}
}  // namespace hsrb_robot_hardware
