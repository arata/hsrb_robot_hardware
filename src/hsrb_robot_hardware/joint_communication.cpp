/*
Copyright (c) 2021 TOYOTA MOTOR CORPORATION
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
#include "joint_communication.hpp"

#include <algorithm>
#include <iomanip>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_servomotor_protocol/load_control_table.hpp>

namespace {
const uint32_t kRetryCount = 3;

typedef boost::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> Protocol;
typedef hsrb_servomotor_protocol::ControlTableItemDescriptor TableItem;

// エラー発生時にROS_ERRORを送出
void StreamError(const boost::system::error_code& error, uint8_t id, const std::string& sentence) {
  if (error && error.category() == boost::system::system_category()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("JointCommunication"),
                        sentence << "[" << static_cast<int32_t>(id) << "]: "
                        << error.value() << " " << error.message());
  }
}

// エラー発生時にROS_FATALを送出しexit
void StreamFatal(const boost::system::error_code& error, uint8_t id, const std::string& sentence) {
  if (error && error.category() == boost::system::system_category()) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("JointCommunication"),
                        sentence << "[" << static_cast<int32_t>(id) << "]: "
                        << error.value() << " " << error.message());
    exit(EXIT_FAILURE);
  }
}

/// バージョンの文字列を生成する
/// @param[in] hash バージョン情報
/// @return 文字列への変換結果
std::string HashToString(const std::vector<uint8_t>& hash) {
  std::stringstream buffer;
  for (size_t i = 0; i < hash.size(); ++i) {
    buffer << std::setw(2) << std::setfill('0') << std::hex
           << static_cast<int32_t>(hash[i]);
  }
  return buffer.str();
}

/// itemが示す場所にdouble(MKS)で指定した値を変換して書き込む
/// @param[in] protocol 通信プロトコル
/// @param[in] id nodeID
/// @param[in] item コントロールテーブルのitem
/// @param[in] value 書き込む値
/// @return エラーコード
boost::system::error_code WriteControlTableItem(
    const Protocol& protocol,
    uint8_t id,
    const TableItem::ConstPtr& item,
    double value) {
  std::vector<uint8_t> data(item->num_bytes());
  if (!item->ConvertToBytes(value, data)) {
    return boost::system::error_code(
        boost::system::errc::value_too_large,
        boost::system::system_category());
  }
  return protocol->WriteBlock(id, item->initial_address(),
                              data.size(), &data[0]);
}

/// itemが示す場所からdouble(MKS)で指定した値を変換して読み込む
/// @param[in] protocol 通信プロトコル
/// @param[in] id nodeID
/// @param[in] item コントロールテーブルのitem
/// @param[in] value_out 読み込んだ値
/// @return エラーコード
boost::system::error_code ReadControlTableItem(
    const Protocol& protocol,
    uint8_t id,
    const TableItem::ConstPtr& item,
    double& value_out) {
  std::vector<uint8_t> data(item->num_bytes());
  boost::system::error_code error;
  error = protocol->ReadBlock(id, item->initial_address(),
                              data.size(), &data[0]);
  if (!item->ConvertToMKS(data, value_out)) {
    return boost::system::error_code(
        boost::system::errc::illegal_byte_sequence,
        boost::system::system_category());
  }
  return error;
}

/// itemが示す場所にdouble(MKS)で指定した値を変換して読み込む
/// @param[in] protocol 通信プロトコル
/// @param[in] id 書き込むnodeID
/// @param[in] items コントロールテーブルのitem列
/// @param[out] values_out 取得した値 itemsの順番に並ぶ
/// @return エラーコード
boost::system::error_code ReadControlTableItemArray(
    const Protocol& protocol,
    uint8_t id,
    const std::vector<TableItem::ConstPtr>& items,
    std::vector<double>& values_out) {
  if (items.empty()) {
    return boost::system::error_code(
        boost::system::errc::success,
        boost::system::system_category());
  }
  values_out.resize(items.size());
  boost::system::error_code error;
  uint16_t block_initial_address = items[0]->initial_address();
  uint16_t block_final_address = items[0]->final_address();

  for (size_t i = 0; i < items.size(); ++i) {
    block_initial_address =
        std::min(block_initial_address, items[i]->initial_address());
    block_final_address =
        std::max(block_final_address, items[i]->final_address());
  }
  assert(block_initial_address <= block_final_address);
  uint32_t size = block_final_address - block_initial_address + 1;
  std::vector<uint8_t> data(size);

  error = protocol->ReadBlock(id, block_initial_address, size, &data[0]);

  for (size_t i = 0; i < items.size(); ++i) {
    if (!items[i]->ConvertToMKS(
            &data[items[i]->initial_address() - block_initial_address],
            values_out[i])) {
      return boost::system::error_code(
          boost::system::errc::illegal_byte_sequence,
          boost::system::system_category());
    }
  }
  return error;
}

/// itemが示す場所にdouble(MKS)で指定した値を変換して読み込む
/// @param[in] protocol 通信プロトコル
/// @param[in] id 書き込むnodeID
/// @param[in] items コントロールテーブルのitem列
/// @param[out] values_out 取得したitemと値のマップ
/// @return エラーコード
boost::system::error_code ReadControlTableItemArray(
    const Protocol& protocol,
    uint8_t id,
    const std::vector<TableItem::ConstPtr>& items,
    std::map<TableItem::ConstPtr, double>& values_out) {
  std::vector<double> values;
  boost::system::error_code error =
      ReadControlTableItemArray(protocol, id, items, values);
  if (error) {
    if (error.category() == boost::system::system_category()) {
      return error;
    }
  }

  values_out.clear();
  for (uint32_t i = 0; i < items.size(); ++i) {
    values_out[items[i]] = values[i];
  }
  return error;
}
}  // namespace

namespace hsrb_robot_hardware {

/// 成功するまで指定された回数実行する
/// @param[in] func 実行する関数
/// @param[in] retry リトライ回数(0だと1回だけ実行する)
/// @return エラーコード
boost::system::error_code TryWithRetry(
    boost::function<boost::system::error_code()> func,
    uint32_t retry) {
  boost::system::error_code error;
  for (uint32_t count = 0; count <= retry; ++count) {
    error = func();
    if (!error) {
      break;
    }
  }
  return error;
}

bool ReadHash(
    const boost::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol,
    uint8_t id, std::vector<uint8_t>& control_table_hash_out, std::vector<uint8_t>& firmware_hash_out) {
  boost::system::error_code error;
  error = TryWithRetry(
      boost::bind(&hsrb_servomotor_protocol::IDynamixelishProtocol::ReadHash,
                  protocol, id, boost::ref(control_table_hash_out), boost::ref(firmware_hash_out)),
      kRetryCount);
  if (error) {
    if (error.category() == boost::system::system_category()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("JointCommunication"),
                          "Failed to read hash joint " << static_cast<int32_t>(id) << ": " << error.message());
      return false;
    }
  }
  return true;
}

bool LoadControlTable(
    const boost::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol, uint8_t id,
    const std::string& package_path, boost::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table_out) {
  std::vector<uint8_t> control_table_hash;
  std::vector<uint8_t> firmware_hash;
  if (!ReadHash(protocol, id, control_table_hash, firmware_hash)) {
    return false;
  }

  std::string table_path;
  if (hsrb_servomotor_protocol::LoadControlTable(
          package_path, control_table_hash, *table_out, table_path)) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("JointCommunication"), "control_table_path : " << table_path);
    return true;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("JointCommunication"), "Cannot find right control table");
    return false;
  }
}

JointCommunication::JointCommunication(uint8_t act_id,
                   const boost::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol,
                   const boost::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table)
    : act_id_(act_id), protocol_(protocol), table_(table), has_safety_alarm_(true) {
  InitItemDescriptors();
  InitItemToRead();
  ValidateControlTableVersion();
}

bool JointCommunication::ReadHash(
    std::vector<uint8_t>& control_table_hash_out, std::vector<uint8_t>& firmware_hash_out) {
  return hsrb_robot_hardware::ReadHash(protocol_, act_id_, control_table_hash_out, firmware_hash_out);
}

bool JointCommunication::ReadHash(
    std::string& control_table_hash_out, std::string& firmware_hash_out) {
  std::vector<uint8_t> control_table_hash;
  std::vector<uint8_t> firmware_hash;
  if (!ReadHash(control_table_hash, firmware_hash)) {
    return false;
  }
  control_table_hash_out = HashToString(control_table_hash);
  firmware_hash_out = HashToString(firmware_hash);
  return true;
}

void JointCommunication::ResetAlarm() {
  boost::system::error_code error;
  error = TryWithRetry(
      boost::bind(&hsrb_servomotor_protocol::IDynamixelishProtocol::WriteUInt16,
                  protocol_, act_id_, items_["present_alarm_status"]->initial_address(), 0),
      kRetryCount);
  StreamError(error, act_id_, "Failed to clear AlarmStatus");

  if (has_safety_alarm_) {
    error = TryWithRetry(
        boost::bind(&hsrb_servomotor_protocol::IDynamixelishProtocol::WriteUInt32,
                    protocol_, act_id_, items_["present_safety_alarm_status"]->initial_address(), 0),
        kRetryCount);
    StreamError(error, act_id_, "Failed to clear SafetyAlarmStatus");
  }

  error = TryWithRetry(
      boost::bind(&hsrb_servomotor_protocol::IDynamixelishProtocol::WriteUInt16,
                  protocol_, act_id_, items_["present_warning"]->initial_address(), 0),
      kRetryCount);
  StreamError(error, act_id_, "Failed to clear Warning");
}

void JointCommunication::ResetDriveMode(uint8_t drive_mode) {
  boost::system::error_code error = TryWithRetry(
      boost::bind(&JointCommunication::SetDriveMode, this, drive_mode), kRetryCount);
  StreamFatal(error, act_id_, "Failed to write DriveMode");
}

void JointCommunication::ResetVelocityLimit(double limit) {
  boost::system::error_code error = TryWithRetry(
      boost::bind(&JointCommunication::WriteControlTableItem, this,
                  "ref_position_control_vmax", limit),
      kRetryCount);
  StreamFatal(error, act_id_, "Failed to write Max velocity");
}

void JointCommunication::GetCurrentPosition(JointAxis joint_axis, double& position_out) {
  TableItem::ConstPtr item;
  if (joint_axis == kOutputAxis) {
    item = items_["_present_joint_calc_outaxis_correct_position"];
  } else {
    item = items_["_present_motor_outaxis_position"];
  }
  boost::system::error_code error = TryWithRetry(
      boost::bind(::ReadControlTableItem, protocol_, act_id_, item, boost::ref(position_out)),
      kRetryCount);
  StreamFatal(error, act_id_, "Failed to get first current position");
}

void JointCommunication::ResetPosition() {
  boost::system::error_code error = TryWithRetry(
      boost::bind(::WriteControlTableItem, protocol_, act_id_, items_["present_motor_position"], 0.0),
      kRetryCount);
  StreamFatal(error, act_id_, "Failed to write wheel position");
}

boost::system::error_code JointCommunication::Read(ReadValues& read_values_out) {
  std::map<TableItem::ConstPtr, double> values;
  boost::system::error_code error = ReadControlTableItemArray(
      protocol_, act_id_, items_to_read_, values);
  if (!error || error.category() != boost::system::system_category()) {
      read_values_out.temperature = values[items_["_present_temperature"]];
      read_values_out.motor_outaxis_position = values[items_["_present_motor_outaxis_position"]];
      read_values_out.motor_outaxis_velocity = values[items_["_present_motor_outaxis_velocity"]];
      read_values_out.current = values[items_["_present_current"]];
      read_values_out.joint_calc_outaxis_correct_position =
          values[items_["_present_joint_calc_outaxis_correct_position"]];
      read_values_out.drive_mode = values[items_["_present_drive_mode"]];
  }
  return error;
}

boost::system::error_code JointCommunication::ReadAlarm(double& alarm_out) {
  return ::ReadControlTableItem(protocol_, act_id_, items_["present_alarm_status"], alarm_out);
}

boost::system::error_code JointCommunication::ReadSafetyAlarm(double& alarm_out) {
  if (has_safety_alarm_) {
    return ::ReadControlTableItem(protocol_, act_id_, items_["present_safety_alarm_status"], alarm_out);
  } else {
    alarm_out = 0.0;
    return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }
}

boost::system::error_code JointCommunication::WriteCommandPosition(double value) {
  return ::WriteControlTableItem(protocol_, act_id_, items_["ref_position_goal_outaxis"], value);
}

boost::system::error_code JointCommunication::WriteCommandVelocity(double value) {
  return ::WriteControlTableItem(protocol_, act_id_, items_["ref_velocity_goal_outaxis"], value);
}

boost::system::error_code JointCommunication::SetDriveMode(uint8_t drive_mode) {
  return protocol_->WriteUInt8(act_id_, items_["command_drive_mode"]->initial_address(), drive_mode);
}

boost::system::error_code JointCommunication::WriteControlTableItem(const std::string& name, double value) {
  return WriteControlTableItem(name, value, false);
}

boost::system::error_code JointCommunication::WriteControlTableItem(
    const std::string& name, double value, bool do_sync) {
  TableItem::ConstPtr item = table_->ReferItemDescriptor(name);
  if (!item) {
    return boost::system::error_code(boost::system::errc::invalid_argument,
                                     boost::system::system_category());
  }
  boost::system::error_code error = ::WriteControlTableItem(protocol_, act_id_, item, value);
  if (!do_sync || error) {
    return error;
  } else {
    return protocol_->SyncParam(act_id_);
  }
}

boost::system::error_code JointCommunication::ReadControlTableItem(const std::string& name, double& value) {
  TableItem::ConstPtr item = table_->ReferItemDescriptor(name);
  if (item) {
    return ::ReadControlTableItem(protocol_, act_id_, item, value);
  } else {
    return boost::system::error_code(boost::system::errc::invalid_argument,
                                     boost::system::system_category());
  }
}

boost::system::error_code JointCommunication::ResetOutputEncoder() {
  return protocol_->AvagoAvePos(act_id_);
}

void JointCommunication::ValidateControlTableVersion() {
  std::vector<uint8_t> control_table_hash;
  std::vector<uint8_t> firmware_hash;
  if (!ReadHash(control_table_hash, firmware_hash)) {
    exit(EXIT_FAILURE);
  }

  if (control_table_hash != table_->GetMd5Sum()) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("JointCommunication"),
                        "Md5 mismatch joint " << static_cast<int32_t>(act_id_) << "\n"
                        << HashToString(control_table_hash) << " vs "
                        << HashToString(table_->GetMd5Sum()));
    exit(EXIT_FAILURE);
  }
}

// 通信に必要なItemDescriptorを取得する
void JointCommunication::InitItemDescriptors() {
  items_.clear();
  InsertItemDescriptor("present_alarm_status");
  InsertItemDescriptor("present_warning");
  InsertItemDescriptor("command_drive_mode");

  InsertItemDescriptor("_present_temperature");
  InsertItemDescriptor("_present_motor_outaxis_position");
  InsertItemDescriptor("_present_motor_outaxis_velocity");
  InsertItemDescriptor("_present_current");
  InsertItemDescriptor("_present_joint_calc_outaxis_correct_position");
  InsertItemDescriptor("_present_drive_mode");

  InsertItemDescriptor("ref_position_goal_outaxis");
  InsertItemDescriptor("ref_velocity_goal_outaxis");

  InsertItemDescriptor("present_motor_position");

  TableItem::ConstPtr item = table_->ReferItemDescriptor("present_safety_alarm_status");
  if (item) {
    items_.insert(std::pair<std::string, TableItem::ConstPtr>("present_safety_alarm_status", item));
    has_safety_alarm_ = true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("JointCommunication"), "Control table does not have present_safety_alarm_status");
    has_safety_alarm_ = false;
  }
}

// ItemDescriptorを取得する
void JointCommunication::InsertItemDescriptor(const std::string& name) {
  TableItem::ConstPtr item = table_->ReferItemDescriptor(name);
  if (!item) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("JointCommunication"), "Cannot get " <<  name << " from control table");
    exit(EXIT_FAILURE);
  }
  items_.insert(std::pair<std::string, TableItem::ConstPtr>(name, item));
}

// 読み込みを行うitemの配列を初期化する
void JointCommunication::InitItemToRead() {
  // Read()において読み込みに使うcontrol_tableのitemを配列にしておく
  // ここに入れたものはブロック読み込みされるので一塊に
  // なっていないと通信の負荷が大きくなる
  items_to_read_.clear();
  items_to_read_.push_back(items_["_present_temperature"]);
  items_to_read_.push_back(items_["_present_motor_outaxis_position"]);
  items_to_read_.push_back(items_["_present_motor_outaxis_velocity"]);
  items_to_read_.push_back(items_["_present_current"]);
  items_to_read_.push_back(items_["_present_joint_calc_outaxis_correct_position"]);
  items_to_read_.push_back(items_["_present_drive_mode"]);
}


boost::system::error_code GripperCommunication::Read(GripperValues& read_values_out) {
  std::map<TableItem::ConstPtr, double> values;
  boost::system::error_code error = ReadControlTableItemArray(
      protocol_, act_id_, items_to_read_, values);
  if (!error || error.category() != boost::system::system_category()) {
    read_values_out.temperature = values[items_["_present_temperature"]];
    read_values_out.current = values[items_["_present_current"]];
    read_values_out.drive_mode = values[items_["_present_drive_mode"]];

    read_values_out.hand_motor_position = values[items_["_present_hand_motor_position"]];
    read_values_out.hand_left_position = values[items_["_present_hand_left_position"]];
    read_values_out.hand_right_position = values[items_["_present_hand_right_position"]];
    read_values_out.hand_motor_velocity = values[items_["_present_hand_motor_velocity"]];
    read_values_out.hand_left_velocity = values[items_["_present_hand_left_velocity"]];
    read_values_out.hand_right_velocity = values[items_["_present_hand_right_velocity"]];
    read_values_out.hand_left_force = values[items_["_present_hand_left_force"]];
    read_values_out.hand_right_force = values[items_["_present_hand_right_force"]];
    read_values_out.hand_grasping_flag = values[items_["_present_hand_grasping_flag"]];
    read_values_out.effort = values[items_["_present_effort"]];
    read_values_out.supply_voltage = values[items_["_present_supply_voltage"]];
  }
  return error;
}

boost::system::error_code GripperCommunication::WriteCommandForce(double value) {
  return ::WriteControlTableItem(protocol_, act_id_, items_["ref_hand_force_goal"], value);
}

boost::system::error_code GripperCommunication::WriteCommandEffort(double value) {
  return ::WriteControlTableItem(protocol_, act_id_, items_["ref_effort_goal"], value);
}

boost::system::error_code GripperCommunication::WriteGraspingFlag(bool grasping_flag) {
  return protocol_->WriteUInt8(act_id_, items_["ref_hand_grasping_flag"]->initial_address(), grasping_flag);
}

// 通信に必要なItemDescriptorを取得する
void GripperCommunication::InitItemDescriptors() {
  InsertItemDescriptor("present_alarm_status");
  InsertItemDescriptor("present_warning");
  InsertItemDescriptor("command_drive_mode");

  InsertItemDescriptor("_present_temperature");
  InsertItemDescriptor("_present_current");
  InsertItemDescriptor("_present_drive_mode");

  InsertItemDescriptor("ref_position_goal_outaxis");
  InsertItemDescriptor("ref_velocity_goal_outaxis");
  InsertItemDescriptor("ref_effort_goal");
  InsertItemDescriptor("ref_hand_force_goal");
  InsertItemDescriptor("ref_hand_grasping_flag");

  InsertItemDescriptor("_present_hand_motor_position");
  InsertItemDescriptor("_present_hand_left_position");
  InsertItemDescriptor("_present_hand_right_position");
  InsertItemDescriptor("_present_hand_motor_velocity");
  InsertItemDescriptor("_present_hand_left_velocity");
  InsertItemDescriptor("_present_hand_right_velocity");
  InsertItemDescriptor("_present_hand_left_force");
  InsertItemDescriptor("_present_hand_right_force");
  InsertItemDescriptor("_present_hand_grasping_flag");
  InsertItemDescriptor("_present_effort");
  InsertItemDescriptor("_present_supply_voltage");

  TableItem::ConstPtr item = table_->ReferItemDescriptor("present_safety_alarm_status");
  if (item) {
    items_.insert(std::pair<std::string, TableItem::ConstPtr>("present_safety_alarm_status", item));
    has_safety_alarm_ = true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("JointCommunication"), "Control table does not have present_safety_alarm_status");
    has_safety_alarm_ = false;
  }
}

// 読み込みを行うitemの配列を初期化する
void GripperCommunication::InitItemToRead() {
  // Read()において読み込みに使うcontrol_tableのitemを配列にしておく
  // ここに入れたものはブロック読み込みされるので一塊に
  // なっていないと通信の負荷が大きくなる
  items_to_read_.clear();
  items_to_read_.push_back(items_["_present_hand_motor_position"]);
  items_to_read_.push_back(items_["_present_hand_left_position"]);
  items_to_read_.push_back(items_["_present_hand_right_position"]);
  items_to_read_.push_back(items_["_present_hand_motor_velocity"]);
  items_to_read_.push_back(items_["_present_hand_left_velocity"]);
  items_to_read_.push_back(items_["_present_hand_right_velocity"]);
  items_to_read_.push_back(items_["_present_hand_left_force"]);
  items_to_read_.push_back(items_["_present_hand_right_force"]);
  items_to_read_.push_back(items_["_present_hand_grasping_flag"]);
  items_to_read_.push_back(items_["_present_effort"]);
  items_to_read_.push_back(items_["_present_supply_voltage"]);
  items_to_read_.push_back(items_["_present_temperature"]);
  items_to_read_.push_back(items_["_present_current"]);
  items_to_read_.push_back(items_["_present_drive_mode"]);
}
}  // namespace hsrb_robot_hardware
