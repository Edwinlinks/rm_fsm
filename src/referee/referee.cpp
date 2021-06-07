//
// Created by peter on 2021/5/17.
//
#include "rm_fsm/referee/referee.h"

namespace rm_fsm {
void Referee::init() {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  try {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
  } catch (std::exception &e) {
    ROS_ERROR(
        "Cannot set serial port of referee system, check whether the serial library is installed and try to use catkin clean");
    return;
  }
  if (!serial_.isOpen()) {
    try {
      serial_.open();
    } catch (serial::IOException &e) {
      ROS_ERROR("Cannot open referee port");
    }
  }
  if (is_online_) ROS_INFO("Referee system connect successfully");
}

// read data from referee
void Referee::read() {
  std::vector<uint8_t> rx_buffer;
  uint8_t temp_buffer[k_unpack_buffer_length_] = {0};
  int rx_len = 0, frame_len;

  if (ros::Time::now() - last_get_referee_data_ > ros::Duration(0.1)) is_online_ = false;
  try {
    if (serial_.waitReadable()) {
      rx_len = serial_.available();
      serial_.read(rx_buffer, rx_len);
    }
  } catch (serial::IOException &e) {
    ROS_ERROR("Referee system disconnect, cannot read referee data");
    is_online_ = false;
    return;
  }
  if (rx_len < k_unpack_buffer_length_) {
    for (int kI = 0; kI < k_unpack_buffer_length_ - rx_len; ++kI) temp_buffer[kI] = unpack_buffer_[kI + rx_len];
    for (int kI = 0; kI < rx_len; ++kI) temp_buffer[kI + k_unpack_buffer_length_ - rx_len] = rx_buffer[kI];
    for (int kI = 0; kI < k_unpack_buffer_length_; ++kI) unpack_buffer_[kI] = temp_buffer[kI];
  }
  for (int kI = 0; kI < k_unpack_buffer_length_ - k_frame_length_; ++kI) {
    if (unpack_buffer_[kI] == 0xA5) {
      frame_len = unpack(&unpack_buffer_[kI]);
      if (frame_len != -1) kI += frame_len;
    }
  }
  super_capacitor_.read(rx_buffer);
  getRobotId();
  publishData();
}

int Referee::unpack(uint8_t *rx_data) {
  uint16_t cmd_id;
  int frame_len;
  FrameHeaderStruct frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (verifyCRC8CheckSum(rx_data, k_header_length_) == true) {
    frame_len = frame_header.data_length_ + k_header_length_ + k_cmd_id_length_ + k_tail_length_;;
    if (verifyCRC16CheckSum(rx_data, frame_len) == true) {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id) {
        case GAME_STATUS_CMD:memcpy(&referee_data_.game_status_, rx_data + 7, sizeof(GameStatus));
          break;
        case GAME_RESULT_CMD:memcpy(&referee_data_.game_result_, rx_data + 7, sizeof(GameResult));
          break;
        case GAME_ROBOT_HP_CMD:memcpy(&referee_data_.game_robot_hp_, rx_data + 7, sizeof(GameRobotHp));
          break;
        case DART_STATUS_CMD:memcpy(&referee_data_.dart_status_, rx_data + 7, sizeof(DartStatus));
          break;
        case ICRA_ZONE_STATUS_CMD:
          memcpy(&referee_data_.icra_buff_debuff_zone_status, rx_data + 7,
                 sizeof(IcraBuffDebuffZoneStatus));
          break;
        case FIELD_EVENTS_CMD:memcpy(&referee_data_.event_data_, rx_data + 7, sizeof(EventData));
          break;
        case SUPPLY_PROJECTILE_ACTION_CMD:
          memcpy(&referee_data_.supply_projectile_action_, rx_data + 7,
                 sizeof(SupplyProjectileAction));
          break;
        case REFEREE_WARNING_CMD:memcpy(&referee_data_.referee_warning_, rx_data + 7, sizeof(RefereeWarning));
          break;
        case DART_REMAINING_CMD:memcpy(&referee_data_.dart_remaining_time_, rx_data + 7, sizeof(DartRemainingTime));
          break;
        case ROBOT_STATUS_CMD:memcpy(&referee_data_.game_robot_status_, rx_data + 7, sizeof(GameRobotStatus));
          break;
        case POWER_HEAT_DATA_CMD:memcpy(&referee_data_.power_heat_data_, rx_data + 7, sizeof(PowerHeatData));
          referee_data_.power_heat_data_.chassis_volt_ *= 0.001; //mV->V
          referee_data_.power_heat_data_.chassis_current_ *= 0.001; //mA->A
          break;
        case ROBOT_POS_CMD:memcpy(&referee_data_.game_robot_pos_, rx_data + 7, sizeof(GameRobotPos));
          break;
        case BUFF_CMD:memcpy(&referee_data_.buff_, rx_data + 7, sizeof(Buff));
          break;
        case AERIAL_ROBOT_ENERGY_CMD:
          memcpy(&referee_data_.aerial_robot_energy_, rx_data + 7,
                 sizeof(AerialRobotEnergy));
          break;
        case ROBOT_HURT_CMD:memcpy(&referee_data_.robot_hurt_, rx_data + 7, sizeof(RobotHurt));
          break;
        case SHOOT_DATA_CMD:memcpy(&referee_data_.shoot_data_, rx_data + 7, sizeof(ShootData));
          break;
        case BULLET_REMAINING_CMD:memcpy(&referee_data_.bullet_remaining_, rx_data + 7, sizeof(BulletRemaining));
          break;
        case ROBOT_RFID_STATUS_CMD:memcpy(&referee_data_.rfid_status_, rx_data + 7, sizeof(RfidStatus));
          break;
        case DART_CLIENT_CMD:memcpy(&referee_data_.dart_client_cmd_, rx_data + 7, sizeof(DartClientCmd));
          break;
        case STUDENT_INTERACTIVE_DATA_CMD:
          memcpy(&referee_data_.student_interactive_data_, rx_data + 7,
                 sizeof(InteractiveData));
          break;
        default:ROS_WARN("Referee command ID not found.");
          break;
      }
      is_online_ = true;
      last_get_referee_data_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

void Referee::getRobotId() {
  robot_id_ = referee_data_.game_robot_status_.robot_id_;
  if (robot_id_ != BLUE_SENTRY && robot_id_ != RED_SENTRY) {
    switch (robot_id_) {
      case BLUE_HERO:client_id_ = BLUE_HERO_CLIENT;
        break;
      case BLUE_ENGINEER:client_id_ = BLUE_ENGINEER_CLIENT;
        break;
      case BLUE_STANDARD_1:client_id_ = BLUE_STANDARD_1_CLIENT;
        break;
      case BLUE_STANDARD_2:client_id_ = BLUE_STANDARD_2_CLIENT;
        break;
      case BLUE_STANDARD_3:client_id_ = BLUE_STANDARD_3_CLIENT;
        break;
      case RED_HERO:client_id_ = RED_HERO_CLIENT;
        break;
      case RED_ENGINEER:client_id_ = RED_ENGINEER_CLIENT;
        break;
      case RED_STANDARD_1:client_id_ = RED_STANDARD_1_CLIENT;
        break;
      case RED_STANDARD_2:client_id_ = RED_STANDARD_2_CLIENT;
        break;
      case RED_STANDARD_3:client_id_ = RED_STANDARD_3_CLIENT;
        break;
    }
  }
}

void Referee::publishData() {
  if (robot_id_ == RED_HERO || robot_id_ == BLUE_HERO) {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_;
  } else {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_;
  }
  referee_pub_data_.chassis_volt = referee_data_.power_heat_data_.chassis_volt_;
  referee_pub_data_.chassis_current = referee_data_.power_heat_data_.chassis_current_;
  referee_pub_data_.chassis_power = referee_data_.power_heat_data_.chassis_power_;
  referee_pub_data_.chassis_power_buffer = referee_data_.power_heat_data_.chassis_power_buffer_;
  referee_pub_data_.robot_hp = referee_data_.game_robot_status_.remain_hp_;
  referee_pub_data_.hurt_armor_id = referee_data_.robot_hurt_.armor_id_;
  referee_pub_data_.hurt_type = referee_data_.robot_hurt_.hurt_type_;
  referee_pub_data_.bullet_speed = referee_data_.shoot_data_.bullet_speed_;
  referee_pub_data_.stamp = last_get_referee_data_;

  super_capacitor_pub_data_.capacity = super_capacitor_.parameters[3] * 100;
  super_capacitor_pub_data_.chassis_power_buffer = super_capacitor_.parameters[2];
  super_capacitor_pub_data_.limit_power = super_capacitor_.parameters[1];
  super_capacitor_pub_data_.chassis_power = super_capacitor_.parameters[0];
  super_capacitor_pub_data_.stamp = super_capacitor_.last_get_capacitor_data_;

  referee_pub_.publish(referee_pub_data_);
  super_capacitor_pub_.publish(super_capacitor_pub_data_);
}

void Referee::drawCircle(int center_x, int center_y, int radius, int picture_id,
                         GraphicColorType color, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientGraphicData)] = {0};
  auto client_graph_data = (ClientGraphicData *) tx_data;
  int tx_len = k_header_length_ + k_cmd_id_length_ + sizeof(ClientGraphicData) + k_tail_length_;

  client_graph_data->student_interactive_header_data_.data_cmd_id_ = CLIENT_GRAPH_SINGLE_CMD;
  client_graph_data->student_interactive_header_data_.sender_id_ = robot_id_;
  client_graph_data->student_interactive_header_data_.receiver_id_ = client_id_;
  client_graph_data->graphic_data_struct_.graphic_name_[0] = (uint8_t) (picture_id & 0xff);
  client_graph_data->graphic_data_struct_.graphic_name_[1] = (uint8_t) ((picture_id >> 8) & 0xff);
  client_graph_data->graphic_data_struct_.graphic_name_[2] = (uint8_t) ((picture_id >> 16) & 0xff);
  client_graph_data->graphic_data_struct_.start_x_ = center_x;
  client_graph_data->graphic_data_struct_.start_y_ = center_y;
  client_graph_data->graphic_data_struct_.radius_ = radius;
  client_graph_data->graphic_data_struct_.operate_type_ = operate_type;
  client_graph_data->graphic_data_struct_.graphic_type_ = 2; // circle
  client_graph_data->graphic_data_struct_.layer_ = 0;
  client_graph_data->graphic_data_struct_.color_ = color;
  client_graph_data->graphic_data_struct_.width_ = 3;
  pack(tx_buffer, tx_data, STUDENT_INTERACTIVE_DATA_CMD, sizeof(ClientGraphicData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to draw UI");
  }
}

void Referee::drawString(int x, int y, int picture_id, std::string data,
                         GraphicColorType color, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientCharData)] = {0};
  auto client_char_data = (ClientCharData *) tx_data;
  int tx_len = k_header_length_ + k_cmd_id_length_ + sizeof(ClientCharData) + k_tail_length_;

  client_char_data->student_interactive_header_data_.data_cmd_id_ = CLIENT_CHARACTER_CMD;
  client_char_data->student_interactive_header_data_.sender_id_ = robot_id_;
  client_char_data->student_interactive_header_data_.receiver_id_ = client_id_;
  client_char_data->graphic_data_struct_.graphic_name_[0] = (uint8_t) (picture_id & 0xff);
  client_char_data->graphic_data_struct_.graphic_name_[1] = (uint8_t) ((picture_id >> 8) & 0xff);
  client_char_data->graphic_data_struct_.graphic_name_[2] = (uint8_t) ((picture_id >> 16) & 0xff);
  client_char_data->graphic_data_struct_.start_x_ = x;
  client_char_data->graphic_data_struct_.start_y_ = y;
  client_char_data->graphic_data_struct_.graphic_type_ = 7; // char
  client_char_data->graphic_data_struct_.operate_type_ = operate_type;
  client_char_data->graphic_data_struct_.start_angle_ = 20; // char size
  client_char_data->graphic_data_struct_.end_angle_ = (int) data.size(); // string length
  client_char_data->graphic_data_struct_.width_ = 5; // line width
  client_char_data->graphic_data_struct_.layer_ = 0;
  client_char_data->graphic_data_struct_.color_ = color;
  for (int kI = 0; kI < 30; ++kI) {
    if (kI < (int) data.size())
      client_char_data->data_[kI] = data[kI];
    else
      client_char_data->data_[kI] = ' ';
  }
  pack(tx_buffer, tx_data, STUDENT_INTERACTIVE_DATA_CMD, sizeof(ClientCharData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to draw UI");
  }
}

void Referee::sendInteractiveData(int data_cmd_id, int receiver_id, uint8_t data) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(InteractiveData)] = {0};
  auto student_interactive_data = (InteractiveData *) tx_data;
  int tx_len = k_header_length_ + k_cmd_id_length_ + sizeof(InteractiveData) + k_tail_length_;

  student_interactive_data->student_interactive_header_data_.data_cmd_id_ = data_cmd_id;
  student_interactive_data->student_interactive_header_data_.sender_id_ = robot_id_;
  student_interactive_data->student_interactive_header_data_.receiver_id_ = receiver_id;
  student_interactive_data->data_ = data;
  pack(tx_buffer, tx_data, STUDENT_INTERACTIVE_DATA_CMD, sizeof(InteractiveData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to send command to sentry");
  }
}

void Referee::pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len) {
  memset(tx_buffer, 0, k_frame_length_);
  auto *frame_header = (FrameHeaderStruct *) tx_buffer;

  frame_header->sof_ = 0xA5;
  frame_header->data_length_ = len;
  memcpy(&tx_buffer[k_header_length_], (uint8_t *) &cmd_id, k_cmd_id_length_);
  appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, len);
  appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + len + k_tail_length_);
}

uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char uc_crc_8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = uc_crc_8 ^ (*pch_message++);
    uc_crc_8 = kCrc8Table[uc_index];
  }
  return (uc_crc_8);
}

uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char uc_expected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return 0;
  uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, kCrc8Init);
  return (uc_expected == pch_message[dw_length - 1]);
}

void appendCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char uc_crc = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return;
  uc_crc = getCRC8CheckSum((unsigned char *) pch_message, dw_length - 1, kCrc8Init);
  pch_message[dw_length - 1] = uc_crc;
}

uint16_t getCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc) {
  uint8_t chData;
  if (pch_message == nullptr) return 0xFFFF;
  while (dw_length--) {
    chData = *pch_message++;
    (w_crc) = ((uint16_t) (w_crc) >> 8) ^ wCRC_table[((uint16_t) (w_crc) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return w_crc;
}

uint32_t verifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length) {
  uint16_t w_expected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return 0;
  w_expected = getCRC16CheckSum(pch_message, dw_length - 2, kCrc16Init);
  return ((w_expected & 0xff) == pch_message[dw_length - 2]
      && ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
}

void appendCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length) {
  uint16_t wCRC = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return;
  wCRC = getCRC16CheckSum((uint8_t *) pch_message, dw_length - 2, kCrc16Init);
  pch_message[dw_length - 2] = (uint8_t) (wCRC & 0x00ff);
  pch_message[dw_length - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}

void SuperCapacitor::read(const std::vector<uint8_t> &rx_buffer) {
  int count = 0;
  memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
  memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
  receive_buf_counter_ = 0;
  for (unsigned char kI : rx_buffer) {
    dtpReceivedCallBack(kI);
    count++;
    if (count >= (int) sizeof(receive_buffer_)) {
      memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
      memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
      receive_buf_counter_ = 0;
    }
  }
  if (parameters[0] >= 120) parameters[0] = 120;
  if (parameters[0] <= 0) parameters[0] = 0;
  if (parameters[2] >= 25) parameters[2] = 25;
  if (parameters[2] <= 0) parameters[2] = 0;
  if (parameters[3] >= 1) parameters[3] = 1;
  if (parameters[3] <= 0) parameters[3] = 0;
  if (ros::Time::now() - last_get_capacitor_data_ > ros::Duration(0.1)) is_online_ = false;
}

void SuperCapacitor::receiveCallBack(unsigned char package_id, unsigned char *data) {
  if (package_id == 0) {
    last_get_capacitor_data_ = ros::Time::now();
    parameters[0] = int16ToFloat((data[0] << 8) | data[1]);
    parameters[1] = int16ToFloat((data[2] << 8) | data[3]);
    parameters[2] = int16ToFloat((data[4] << 8) | data[5]);
    parameters[3] = int16ToFloat((data[6] << 8) | data[7]);
  }
}

void SuperCapacitor::dtpReceivedCallBack(unsigned char receive_byte) {
  unsigned char check_flag;
  unsigned int sof_pos, eof_pos, check_counter;

  receive_buffer_[receive_buf_counter_] = receive_byte;
  receive_buf_counter_ = receive_buf_counter_ + 1;
  check_flag = 0;
  sof_pos = 0;
  eof_pos = 0;
  check_counter = 0;
  while (true) {
    if (check_flag == 0 && receive_buffer_[check_counter] == 0xff) {
      check_flag = 1;
      sof_pos = check_counter;
    } else if (check_flag == 1 && receive_buffer_[check_counter] == 0xff) {
      eof_pos = check_counter;
      break;
    }
    if (check_counter >= (receive_buf_counter_ - 1)) break;
    else check_counter++;
  }                                                           // Find Package In Buffer

  if ((eof_pos - sof_pos) == 11) {
    unsigned int temp_var;
    unsigned char data_buffer[8] = {0};
    unsigned char valid_buffer[12] = {0};

    for (temp_var = 0; temp_var < 12; temp_var++)           // Copy Data To Another Buffer
      valid_buffer[temp_var] = receive_buffer_[sof_pos + temp_var];

    eof_pos++;
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_ - eof_pos; temp_var++)
      ping_pong_buffer_[temp_var] = receive_buffer_[eof_pos + temp_var];
    receive_buf_counter_ = receive_buf_counter_ - eof_pos;
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_; temp_var++)
      receive_buffer_[temp_var] = ping_pong_buffer_[temp_var];

    unsigned char pid_bit = valid_buffer[1] >> 4;           // Get The PID Bit
    if (pid_bit == ((~(valid_buffer[1] & 0x0f)) & 0x0f)) {   // PID Verify
      for (temp_var = 0; temp_var < 8; ++temp_var)
        data_buffer[temp_var] = valid_buffer[2 + temp_var];
      if (valid_buffer[10] != 0x00) {                   // Some Byte had been replace
        unsigned char temp_filter = 0x00;
        for (temp_var = 0; temp_var < 8; ++temp_var)
          if (((valid_buffer[10] & (temp_filter | (0x01 << temp_var))) >> temp_var)
              == 1)                                   // This Byte Need To Adjust
            data_buffer[temp_var] = 0xff;           // Adjust to 0xff
      }
      receiveCallBack(pid_bit, data_buffer);
    }
  } else if ((eof_pos - sof_pos) != 0 && eof_pos != 0) {
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    receive_buf_counter_ = 0;
  }
}

float SuperCapacitor::int16ToFloat(unsigned short data0) {
  if (data0 == 0) return 0;
  float *fp32;
  unsigned int fInt32 = ((data0 & 0x8000) << 16) |
      (((((data0 >> 10) & 0x1f) - 0x0f + 0x7f) & 0xff) << 23)
      | ((data0 & 0x03FF) << 13);
  fp32 = (float *) &fInt32;
  return *fp32;
}

} // namespace rm_fsm