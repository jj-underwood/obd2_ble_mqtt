#include "obd2_ble_mqtt.h"
#include "esphome/core/log.h"
#include "esp_task_wdt.h"
#include <regex>
#include <iostream>
#include <string>
#include <cctype>

#ifdef USE_ESP32

namespace esphome {
namespace obd2_ble_mqtt {

static const char *TAG = "obd2_ble_mqtt";

void OBD2BLEClient::setup() {
  ESP_LOGD(TAG, "Setting up OBD2 BLE Client...");
  string_to_uuid(this->service_uuid_str_, &this->service_uuid_);
  string_to_uuid(this->read_char_uuid_str_, &this->read_char_uuid_);
  string_to_uuid(this->write_char_uuid_str_, &this->write_char_uuid_);
  string_to_uuid(this->notify_char_uuid_str_, &this->notify_char_uuid_);
  
  for (auto it = init_commands_.rbegin(); it != init_commands_.rend(); ++it) {
    OBD2Task task;
    task.command = *it;
    task.can_id = "";
    task.mode = "";
    task.pid = "";
    task.bits = "";
    task.ignore_error = false;
    task_queue_.insert(task_queue_.begin(), std::move(task));
  }
  
//  dump_config();
}

void OBD2BLEClient::loop() {
  if (!mtu_configured_ || !notifications_ready_ || !descr_write_done_) return;
  
  if (current_task_index_ < task_queue_.size()) {
    OBD2Task &task = task_queue_[current_task_index_];
    process_task(task, command_delay_, command_wait_, response_wait_, publish_delay_, disconnect_delay_);
    if (task.status == DONE || (task.status == ERROR && task.ignore_error)) {
      current_task_index_++;
    }
  } else {
    this->disconnect();
  }
}

void OBD2BLEClient::dump_config() {
  ESP_LOGCONFIG(TAG, "OBD2 BLE Client:");
  ESP_LOGCONFIG(TAG, " Service UUID: %s", service_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, " Write Characteristic UUID: %s", write_char_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, " Notify Characteristic UUID: %s", notify_char_uuid_str_.c_str());
  std::string init_commands_str;
  for (const auto &cmd : init_commands_) {
    init_commands_str += cmd + " ";
  }
  if (!init_commands_str.empty()) init_commands_str.pop_back();
  ESP_LOGCONFIG(TAG, " OBD2 Initialization Commands: %s", init_commands_str.c_str());
  ESP_LOGCONFIG(TAG, " OBD2 Command Delay: %d ms", command_delay_);
  ESP_LOGCONFIG(TAG, " OBD2 Command Wait: %d ms", command_wait_);
  ESP_LOGCONFIG(TAG, " Task queue size: %d", task_queue_.size());
  for (auto &task : task_queue_) {
    std::string line = "  command: " + task.command;
    if (task.can_id != "") {
      line += ", CAN ID: " + task.can_id;
    }
    if (task.mode != "") {
      line += ", Mode: " + task.mode;
    }
    if (task.pid != "") {
      line += ", PID: " + task.pid;
    }
    if (task.bits != "") {
      line += ", Bits: " + task.bits;
    }
    if (task.sensor != nullptr) {
      line += ", Name: " + task.sensor->get_name();
    } else if (task.text_sensor != nullptr) {
      line += ", Name: " + task.text_sensor->get_name();
    }
    ESP_LOGCONFIG(TAG, "%s", line.c_str());
    for (int i = 0; i < task.codes.size(); ++i) {
      ESP_LOGCONFIG(TAG, "   code: %s, name: %s", task.codes[i].c_str(), task.binary_sensors[i]->get_name().c_str());
    }
  }
}

void OBD2BLEClient::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  ESP_LOGV(TAG, "GATT Event: %d", event);
  switch (event) {
    case ESP_GATTC_REG_EVT:
      ESP_LOGD(TAG, "GATT client is registered.");
      break;
    case ESP_GATTC_CONNECT_EVT:
      ESP_LOGD(TAG, "BLE physical connection is set up.");
      gattc_if_ = gattc_if;
      conn_id_ = param->connect.conn_id;
      ESP_LOGI(TAG, "Connected to device, gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
      break;
    case ESP_GATTC_OPEN_EVT:
      ESP_LOGD(TAG, "GATT virtual connection is set up.");
      break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
      ESP_LOGD(TAG, "BLE discover service completed.");
      break;
    case ESP_GATTC_SEARCH_RES_EVT:
      ESP_LOGD(TAG, "GATT service discovery result is got.");
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGD(TAG, "GATT Search Complete Event");
      auto *write_chr = this->parent_->get_characteristic(this->service_uuid_, this->write_char_uuid_);
      if (write_chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No write characteristic found at device.", this->parent_->address_str().c_str());
        break;
      }
      this->write_char = write_chr;

      auto *notify_chr = this->parent_->get_characteristic(this->service_uuid_, this->notify_char_uuid_);
      if (notify_chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No notify characteristic found at device.", this->parent_->address_str().c_str());
        break;
      }
      this->notify_char = notify_chr;

      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), notify_chr->handle);
      if (status) {
        ESP_LOGD(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT:
      ESP_LOGD(TAG, "Configuration of MTU completes, MTU size: %d.", param->cfg_mtu.mtu);
      mtu_configured_ = true;
      break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      ESP_LOGD(TAG, "Register for notification of a service completes.");
      notifications_ready_ = true;
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      ESP_LOGD(TAG, "GATT characteristic descriptor write completes.");
      descr_write_done_ = true;
      break;
    case ESP_GATTC_WRITE_CHAR_EVT:
      on_write();
      break;
    case ESP_GATTC_NOTIFY_EVT:
      this->on_notify(std::vector<uint8_t>(param->notify.value, param->notify.value + param->notify.value_len));
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGW(TAG, "Disconnected from OBD2 adapter, reason=%d", param->disconnect.reason);
      on_disconnect();
      break;
    case ESP_GATTC_CLOSE_EVT:
      ESP_LOGD(TAG, "GATT virtual connection is closed.");
      break;
    default:
      ESP_LOGD(TAG, "Unhandled GATT Event: %d", event);
      break;
  }
}

void OBD2BLEClient::add_task_for_sensor(sensor::Sensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid, const std::string &bits) {
  auto it = std::find_if(task_queue_.begin(), task_queue_.end(), [&mode, &pid, &bits](const OBD2Task& t) {
    if (bits.empty()) {
      return t.mode == mode && t.pid == pid && t.bits.empty();
    } else {
      return t.mode == mode && t.pid == pid && t.bits == bits;
    }
  });
  if (it != task_queue_.end()) {
    it->can_id = can_id;
    it->sensor = sensor;
  } else {
    OBD2Task task;
    task.command = mode + pid;
    task.can_id = can_id;
    task.mode = mode;
    task.pid = pid;
    task.bits = bits;
    task.sensor = sensor;
    task_queue_.emplace_back(std::move(task));
  }
}

void OBD2BLEClient::add_task_for_text_sensor(text_sensor::TextSensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid) {
  auto it = std::find_if(task_queue_.begin(), task_queue_.end(), [&mode, &pid](const OBD2Task& t) {
    if (pid.empty()) {
      return t.mode == mode && t.pid.empty();
    } else {
      return t.mode == mode && t.pid == pid;
    }
  });
  if (it != task_queue_.end()) {
    it->can_id = can_id;
    it->text_sensor = sensor;
  } else {
    OBD2Task task;
    if (pid.empty()) {
      task.command = mode;
    } else {
      task.command = mode + pid;
      task.pid = pid;
    }
    task.can_id = can_id;
    task.mode = mode;
    task.text_sensor = sensor;
    task_queue_.emplace_back(std::move(task));
  }
}

void OBD2BLEClient::add_task_for_binary_sensor(binary_sensor::BinarySensor *sensor, const std::string &can_id, const std::string &mode, const std::string &pid, const std::string &bits, const std::string &code) {
  if (bits.empty() && !code.empty()) {
    auto it = std::find_if(task_queue_.begin(), task_queue_.end(), [&mode, &pid](const OBD2Task& t) {
      if (pid.empty()) {
        return t.mode == mode && t.pid.empty();
      } else {
        return t.mode == mode && t.pid == pid;
      }
    });
    if (it != task_queue_.end()) {
      it->can_id = can_id;
      it->codes.push_back(code);
      it->binary_sensors.push_back(sensor);
    } else {
      OBD2Task task;
      if (pid.empty()) {
        task.command = mode;
      } else {
        task.command = mode + pid;
        task.pid = pid;
      }
      task.can_id = can_id;
      task.mode = mode;
      task.codes.push_back(code);
      task.binary_sensors.push_back(sensor);
      task_queue_.emplace_back(std::move(task));
    }
  } else if (!bits.empty() && code.empty()) {
    OBD2Task task;
    if (pid.empty()) {
      task.command = mode;
    } else {
      task.command = mode + pid;
      task.pid = pid;
    }
    task.can_id = can_id;
    task.mode = mode;
    task.bits = bits;
    task.binary_sensors.push_back(sensor);
    task_queue_.emplace_back(std::move(task));
  }
}

void OBD2BLEClient::string_to_uuid(const std::string &uuid_str, espbt::ESPBTUUID *uuid) {
  ESP_LOGD(TAG, "string_to_uuid: %s", uuid_str.c_str());
  if (uuid_str.length() == 4 || (uuid_str.length() == 6 && uuid_str.substr(0, 2) == "0x")) {
    // 16-bit UUID
    uint16_t uuid16 = std::stoi(uuid_str.substr(uuid_str.length() - 4), nullptr, 16);
    *uuid = espbt::ESPBTUUID::from_uint16(uuid16);
    ESP_LOGD(TAG, "Converted 16-bit UUID: %04X", uuid16);
  } else if (uuid_str.length() == 36) {
    // 128-bit UUID
    *uuid = espbt::ESPBTUUID::from_raw(uuid_str);
    ESP_LOGD(TAG, "Converted 128-bit UUID: %s", uuid_str.c_str());
  } else {
    ESP_LOGE(TAG, "Invalid UUID string length: %s", uuid_str.c_str());
  }
}

void OBD2BLEClient::process_task(OBD2Task &task, const int &command_delay, const int &command_wait, const int &response_wait, const int &publish_delay, const int &disconnect_delay) {
  unsigned long current_time = millis();
  if (current_time - last_command_time_ >= command_delay) {
    switch (task.status) {
      case PENDING:
        ESP_LOGD(TAG, "Sending command: %s", task.command.c_str());
        request_write(task.command);
        last_command_ = task.command;
        last_command_time_ = current_time;
        task.status = SENDING;
        break;
      case SENDING:
        if (current_time - last_command_time_ >= command_wait) {
          ESP_LOGW(TAG, "Confirmation timeout for command: %s", task.command.c_str());
          task.status = ERROR;
        }
        break;
      case SENT:
        if (current_time - last_command_time_ >= response_wait) {
          ESP_LOGW(TAG, "Response timeout for command: %s", task.command.c_str());
          task.status = ERROR;
        }
        break;
      case RECEIVED:
        ESP_LOGD(TAG, "Processing response for command: %s", task.command.c_str());
        if (parse_data(task)) {
          task.status = PUBLISHING;
        } else {
          task.status = ERROR;
        }
        break;
      case PUBLISHING:
        if (current_time - last_publish_time_ >= publish_delay) {
          if (this->is_connected()) {
            ESP_LOGD(TAG, "Publishing response for command: %s", task.command.c_str());
            publish_data(task);
            if (task.published) {
              task.status = DONE;
            } else {
              last_publish_time_ = current_time;
            }
          }
        }
        break;
      case DONE:
        ESP_LOGI(TAG, "DONE task for command: %s", task.command.c_str());
        break;
      case ERROR:
        if (!task.ignore_error) {
          if (current_time - last_command_time_ >= disconnect_delay) {
            ESP_LOGW(TAG, "Task cancelled. Disconnecting from OBD2 adapter...");
            this->disconnect();
          }
        }
        break;
      default:
        break;
    }
    esp_task_wdt_reset();
  }
}

void OBD2BLEClient::request_write(const std::string &command) {
  if (this->write_char == nullptr) {
    ESP_LOGE(TAG, "Write characteristic is null, cannot send command");
    return;
  }
  std::string command_with_cr = command + "\r";
  auto status = esp_ble_gattc_write_char(
      this->parent_->get_gattc_if(),
      this->parent_->get_conn_id(),
      this->write_char->handle,
      command_with_cr.size(),
      (uint8_t *)command_with_cr.c_str(),
      ESP_GATT_WRITE_TYPE_RSP,
      ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }
}

void OBD2BLEClient::request_read() {
  if (this->read_char == nullptr) {
    ESP_LOGE(TAG, "Read characteristic is null, cannot request read");
    return;
  }
  ESP_LOGD(TAG, "Request read data for %s", last_command_.c_str());
  auto status = esp_ble_gattc_read_char(
      this->parent()->get_gattc_if(),
      this->parent()->get_conn_id(),
      this->read_char->handle,
      ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_read_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }
}

void OBD2BLEClient::disconnect() {
  if (gattc_if_ != 0 && conn_id_ != std::numeric_limits<uint16_t>::max()) {
    ESP_LOGI(TAG, "Disconnecting from device, gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
    esp_ble_gattc_close(gattc_if_, conn_id_);
    cleanup();
  }
}

void OBD2BLEClient::on_write() {
  if (current_task_index_ < task_queue_.size()) {
    OBD2Task &task = task_queue_[current_task_index_];
    if (task.status == PENDING || task.status == SENDING) {
      task.status = SENT;
    }
    ESP_LOGD(TAG, "GATT characteristic write operation completes: %s.", task.command.c_str());
  } else {
    ESP_LOGW(TAG, "GATT write event received, but no current task available!");
  }
}

void OBD2BLEClient::on_notify(const std::vector<uint8_t> &data) {
  if (current_task_index_ < task_queue_.size()) {
    response_buffer_.insert(response_buffer_.end(), data.begin(), data.end());
    parse_response();
  } else {
    ESP_LOGW(TAG, "GATT notify event received, but no current task available!");
  }
}

void OBD2BLEClient::on_disconnect() {
  gattc_if_ = 0;
  conn_id_ = std::numeric_limits<uint16_t>::max();
  ESP_LOGW(TAG, "Disconnected from OBD2 adapter. gattc_if: %d, conn_id: %d", gattc_if_, conn_id_);
}

void OBD2BLEClient::parse_response() {
  if (!response_buffer_.empty() && response_buffer_.back() == 0x3E) {
    ESP_LOGD(TAG, "Received data in hex: %s", format_hex_pretty(response_buffer_).c_str());
    std::string response(response_buffer_.begin(), response_buffer_.end());
    std::replace(response.begin(), response.end(), '\r', ' ');
    std::replace(response.begin(), response.end(), '\0', ' ');
    ESP_LOGD(TAG, "Received data in str: %s", response.c_str());
    
    std::vector<std::vector<uint8_t>> lines;
    std::vector<uint8_t> line;
    for (auto byte : response_buffer_) {
      if (byte == 0x0D) {
        if (!line.empty()) {
          lines.push_back(line);
        }
        line.clear();
      } else {
        line.push_back(byte);
      }
    }
    OBD2Task &task = task_queue_[current_task_index_];
    task.data = std::move(lines);
    task.status = RECEIVED;
    response_buffer_.clear();
  }
}

bool OBD2BLEClient::parse_data(OBD2Task &task) {
  for (const auto &data : task.data) {
    std::string response_str(data.begin(), data.end());
    if (!is_ready(response_str, task.command)) {
      ESP_LOGE(TAG, "Task error for command: %s", task.command.c_str());
      return false;
    }
    if (is_message(task, response_str)) continue;
    
    std::string can_id_str;
    std::uint8_t pci;
    std::uint8_t length;
    std::string response_data_str;
    tie(can_id_str, pci, length, response_data_str) = split_data(response_str);
    if (response_data_str.empty()) response_data_str = response_str;
    ESP_LOGD(TAG, "CAN ID: %s, PCI: %s, Length: %s, Response data: %s", can_id_str.c_str(), format_hex_pretty(pci).c_str(), format_hex_pretty(length).c_str(), response_data_str.c_str());
    
    auto response_data = decode_data(response_data_str);
    if (response_data.empty()) continue;
    
    switch (frame_type(pci)) {
      case FrameType::SINGLE:
        ESP_LOGD(TAG, "Single frame");
        parse_payload(task, can_id_str, length, response_data);
        break;
      case FrameType::FIRST:
        handle_first_frame(task, can_id_str, length, response_data);
        break;
      case FrameType::CONSECUTIVE:
        handle_consecutive_frame(task, can_id_str, response_data);
        break;
      default:
        ESP_LOGW(TAG, "Unknown frame type: %s", format_hex_pretty(pci).c_str());
        break;
    }
  }
  return true;
}

bool OBD2BLEClient::is_ready(const std::string &response_str, const std::string &command) {
  if (response_str.find("UNABLE TO CONNECT") != std::string::npos) {
    ESP_LOGW(TAG, "ELM not available for command: %s", command.c_str());
    return false;
  } else {
    return true;
  }
}

bool OBD2BLEClient::is_message(OBD2Task &task, const std::string &response_str) {
  if (response_str.find("ELM327") != std::string::npos) {
    ESP_LOGD(TAG, "ELM327 available for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("SEARCHING") != std::string::npos) {
    ESP_LOGD(TAG, "Searching for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("OK") != std::string::npos) {
    ESP_LOGD(TAG, "Response OK for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("NO DATA") != std::string::npos) {
    ESP_LOGD(TAG, "No data for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("BUFFER FULL") != std::string::npos) {
    ESP_LOGD(TAG, "No enough buffer for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find(task.command) != std::string::npos) {
    ESP_LOGD(TAG, "Echo received for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "A0") {
    ESP_LOGD(TAG, "Response A0 for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "7") {
    ESP_LOGD(TAG, "Response 7 for command: %s", task.command.c_str());
    return true;
  } else if (response_str == "6") {
    ESP_LOGD(TAG, "Response 6 for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find(">") != std::string::npos) {
    ESP_LOGD(TAG, "Prompt '>' received for command: %s", task.command.c_str());
    return true;
  } else if (response_str.find("?") != std::string::npos) {
    ESP_LOGD(TAG, "Error '?' received for command: %s", task.command.c_str());
    return true;
  } else {
    std::regex atrv_regex(R"((\d+\.\d*)V)");
    std::smatch atrv_match;
    if (std::regex_search(response_str, atrv_match, atrv_regex)) {
      if (task.command == "ATRV") {
        task.value_f = std::stof(atrv_match[1]);
      }
      return true;
    } else {
      return false;
    }
  }
}

std::tuple<std::string, std::uint8_t, std::uint8_t, std::string> OBD2BLEClient::split_data(std::string &data_str) {
  std::uint8_t can_id_length = 0;
  std::string can_id_str;
  std::string pci_str;
  std::uint8_t pci = 0;
  std::string length_str;
  std::uint8_t length = 0;
  std::string response_data_str;
  if (data_str.size() > 5 && data_str[0] == '7' && data_str[1] == 'E') {
    can_id_length = 3;
  } else if (data_str.size() > 10 && (data_str[0] == '1' && (data_str[1] == '8' || data_str[1] == '9'))) {
    can_id_length = 8;
  } else {
    ESP_LOGD(TAG, "No valid CAN ID detected: %s", data_str.c_str());
    return std::make_tuple("", 0, 0, "");
  }
  
  can_id_str = data_str.substr(0, can_id_length);
  if (data_str.size() < can_id_length + 2) {
    ESP_LOGD(TAG, "Data string too short for PCI: %s", data_str.c_str());
    return std::make_tuple(can_id_str, 0, 0, "");
  }
  
  pci_str = data_str.substr(can_id_length, 2);
  pci = static_cast<std::uint8_t>(std::stoi(pci_str, nullptr, 16));
  if (pci <= 0x07) {
    length = pci;
    response_data_str = data_str.substr(can_id_length + 2);
  } else if (pci >= 0x10 && pci <= 0x1F) {
    if (data_str.size() < can_id_length + 4) {
      ESP_LOGD(TAG, "Data string too short for First Frame: %s", data_str.c_str());
      return std::make_tuple(can_id_str, pci, 0, "");
    }
    length_str = data_str.substr(can_id_length + 2, 2);
    length = static_cast<std::uint8_t>(std::stoi(length_str, nullptr, 16));
    response_data_str = data_str.substr(can_id_length + 4);
  } else if (pci >= 0x21 && pci <= 0x3F) {
    response_data_str = data_str.substr(can_id_length + 2);
  }
    
  return std::make_tuple(can_id_str, pci, length, response_data_str);
}

std::vector<uint8_t> OBD2BLEClient::decode_data(const std::string &response_data_str) {
  if (response_data_str.size() % 2 != 0) {
    ESP_LOGW(TAG, "Data size is not even, skipping decode: %s", response_data_str.c_str());
    return {};
  }
  if (!std::all_of(response_data_str.begin(), response_data_str.end(), ::isxdigit)) {
    ESP_LOGW(TAG, "Data is not in ASCII hex format: %s", response_data_str.c_str());
    return {};
  }
  return ascii_hex_to_bytes(std::vector<uint8_t>(response_data_str.begin(), response_data_str.end()));
}

std::vector<uint8_t> OBD2BLEClient::ascii_hex_to_bytes(const std::vector<uint8_t> &ascii_data) {
  std::vector<uint8_t> raw_data;
  for (size_t i = 0; i < ascii_data.size(); i += 2) {
    if (i + 1 >= ascii_data.size() || !isxdigit(ascii_data[i]) || !isxdigit(ascii_data[i + 1])) {
      break;
    }
    uint8_t byte = (std::stoi(std::string(1, ascii_data[i]), nullptr, 16) << 4) |
                   std::stoi(std::string(1, ascii_data[i + 1]), nullptr, 16);
    raw_data.push_back(byte);
  }
  return raw_data;
}

OBD2BLEClient::FrameType OBD2BLEClient::frame_type(uint8_t pci) {
  if (pci <= 0x07) return FrameType::SINGLE;
  if (pci >= 0x10 && pci <= 0x1F) return FrameType::FIRST;
  if (pci >= 0x21 && pci <= 0x3F) return FrameType::CONSECUTIVE;
  return FrameType::UNKNOWN;
}

void OBD2BLEClient::handle_first_frame(OBD2Task &task, std::string &can_id_str, uint8_t length, const std::vector<uint8_t> &response_data) {
  if (task.can_id_map.count(can_id_str)) {
    ESP_LOGW(TAG, "Duplicated first frame for CAN ID: %s", can_id_str.c_str());
    return;
  }
  task.can_id_map[can_id_str] = {length, response_data.size(), response_data};
  ESP_LOGD(TAG, "Stored first frame for CAN ID: %s", can_id_str.c_str());
}

void OBD2BLEClient::handle_consecutive_frame(OBD2Task &task, std::string &can_id_str, const std::vector<uint8_t> &response_data) {
  auto it = task.can_id_map.find(can_id_str);
  if (it == task.can_id_map.end()) {
    ESP_LOGW(TAG, "Consecutive frame received for unknown CAN ID: %s", can_id_str.c_str());
    return;
  }

  auto &entry = it->second;
  entry.concat_data.insert(entry.concat_data.end(), response_data.begin(), response_data.end());
  entry.received_count += response_data.size();

  if (entry.received_count >= entry.total_length) {
    ESP_LOGD(TAG, "Completed data for CAN ID: %s", can_id_str.c_str());
    if (entry.concat_data.size() > entry.total_length) {
      entry.concat_data.resize(entry.total_length);
      ESP_LOGD(TAG, "Final data after trimming: %s", format_hex_pretty(entry.concat_data).c_str());
    }
    parse_payload(task, can_id_str, entry.total_length, entry.concat_data);
    task.can_id_map.erase(it);
  }
}

void OBD2BLEClient::parse_payload(OBD2Task &task, std::string &can_id_str, uint8_t &length, std::vector<uint8_t> &response_data) {
  if (length != response_data.size()) {
    ESP_LOGE(TAG, "Unexpected data size, Length: %s, Data size: %s", format_hex_pretty(length).c_str(), format_hex_pretty(response_data.size()).c_str());
    return;
  }
  // Parsing OBD-II Modes
  switch (response_data[0]) {
    case 0x41:  // Mode 01
      if (response_data[1] == 0x00 || response_data[1] == 0x20 || response_data[1] == 0x40 || response_data[1] == 0x60 || response_data[1] == 0x80 || response_data[1] == 0xA0 || response_data[1] == 0xC0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_01(task, response_data);
        }
      }
      break;
    case 0x45:  // Mode 05
      break;
    case 0x46:  // Mode 06
      if (response_data[1] == 0x00 || response_data[1] == 0x20 || response_data[1] == 0x40 || response_data[1] == 0x60 || response_data[1] == 0x80 || response_data[1] == 0xA0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_05(task, response_data);
        }
      }
      break;
    case 0x43:  // Mode 03
    case 0x47:  // Mode 07
    case 0x4A:  // Mode 0A
      if (task.can_id == can_id_str) {
        handle_dtc_response(task, response_data);
        ESP_LOGD(TAG, "DTC codes returned: %s", task.value_s.c_str());
      }
      break;
    case 0x49:  // Mode 09
      if (response_data[1] == 0x00) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, false);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        if (task.can_id == can_id_str) {
          handle_mode_09(task, response_data);
        }
      }
      break;
    case 0x62:  // Mode 22
      if (response_data[2] == 0x00 || response_data[2] == 0x20 || response_data[2] == 0x40 || response_data[2] == 0x60 || response_data[2] == 0x80 || response_data[2] == 0xA0 || response_data[2] == 0xC0 || response_data[2] == 0xE0) {
        std::string pids_str;
        size_t pids_num;
        tie(pids_str, pids_num) = list_supported_pids(response_data, true);
        if (!pids_str.empty() || pids_num == 0) {
          ESP_LOGI(TAG, "CAN ID: %s, Mode: %s, Supported PIDs: %s, total %d", can_id_str.c_str(), format_hex_pretty(response_data[0]).c_str(), pids_str.c_str(), pids_num);
        }
      } else {
        handle_mode_22(task, response_data);
      }
    default:
      break;
  }
}

std::tuple<std::string, size_t> OBD2BLEClient::list_supported_pids(const std::vector<uint8_t> &response_data, bool is_two_byte_pid) {
  std::string supported_pids_str;
  std::vector<uint8_t> supported_pids_8;
  std::vector<uint16_t> supported_pids_16;
  size_t pids_num = 0;
  
  if (response_data.size() < 6) {
    ESP_LOGW(TAG, "Invalid response size for supported PIDs.");
    return std::make_tuple(supported_pids_str, pids_num);
  }

  uint8_t mode = response_data[0];
  
  if (mode != 0x41 && mode != 0x46 && mode != 0x49 && mode != 0x62) {
    ESP_LOGW(TAG, "Invalid response for supported PIDs.");
    return std::make_tuple(supported_pids_str, pids_num);
  }
  
  uint16_t requested_pid = is_two_byte_pid ? (response_data[1] << 8 | response_data[2]) : response_data[1];
  
  int pid_start = requested_pid + 1;
  size_t data_offset = is_two_byte_pid ? 3 : 2;
  
  for (size_t i = data_offset; i < response_data.size(); i++) {
    uint8_t byte_mask = response_data[i];
    
    for (int bit = 0; bit < 8; bit++) {
      if (byte_mask & (1 << (7 - bit))) {
        if (is_two_byte_pid) {
          uint16_t pid = pid_start + (i - data_offset) * 8 + bit;
          supported_pids_16.push_back(pid);
        } else {
          uint8_t pid = pid_start + (i - data_offset) * 8 + bit;
          supported_pids_8.push_back(pid);
        }
      }
    }
  }
  
  std::string delimiter = " ";
  if (is_two_byte_pid) {
    char buffer[5];
    for (size_t i = 0; i < supported_pids_16.size(); ++i) {
      if (i != 0) {
        supported_pids_str += delimiter;
      }
      snprintf(buffer, sizeof(buffer), "%04X", supported_pids_16[i]);
      supported_pids_str += buffer;
    }
    pids_num = supported_pids_16.size();
  } else {
    char buffer[3];
    for (size_t i = 0; i < supported_pids_8.size(); ++i) {
      if (i != 0) {
        supported_pids_str += delimiter;
      }
      snprintf(buffer, sizeof(buffer), "%02X", supported_pids_8[i]);
      supported_pids_str += buffer;
    }
    pids_num = supported_pids_8.size();
  }
  
  return std::make_tuple(supported_pids_str, pids_num);
}

void OBD2BLEClient::handle_mode_01(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0.0f;
  task.value_s = "";
  uint8_t pid = data[1];
  
  switch (pid) {
    case 0x01: task.value_s = get_monitor_status(data); break;
    case 0x03: task.value_s = get_fuel_system_status(data[2]); break;
    case 0x04: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x05: task.value_f = data[2] - 40.0f; break;
    case 0x06: task.value_f = (100.0f/128.0f) * data[2] - 100.0f; break;
    case 0x07: task.value_f = (100.0f/128.0f) * data[2] - 100.0f; break;
    case 0x08: task.value_f = (100.0f/128.0f) * data[2] - 100.0f; break;
    case 0x09: task.value_f = (100.0f/128.0f) * data[2] - 100.0f; break;
    case 0x0A: task.value_f = 3.0f * data[2]; break;
    case 0x0B: task.value_f = data[2]; break;
    case 0x0C: task.value_f = ((data[2] * 256.0f) + data[3]) / 4.0f; break;
    case 0x0D: task.value_f = data[2]; break;
    case 0x0E: task.value_f = (data[2]/2.0f) - 64.0f; break;
    case 0x0F: task.value_f = data[2] - 40.0f; break;
    case 0x10: task.value_f = ((data[2] * 256.0f) + data[3]) / 100.0f; break;
    case 0x11: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x13: task.value_s = get_o2_sensor_presence(data[2]); break;
    case 0x15: task.value_f = (100.0f/128.0f) * data[3] - 100.0f; break;
    case 0x1C: task.value_s = get_standard_name(data[2]); break;
    case 0x1F: task.value_f = (data[2] * 256.0f) + data[3]; break;
    case 0x21: task.value_f = (data[2] * 256.0f) + data[3]; break;
    case 0x22: task.value_f = 0.079f * ((data[2] * 256.0f) + data[3]); break;
    case 0x23: task.value_f = 10.0f * ((data[2] * 256.0f) + data[3]); break;
    case 0x24: task.value_f = (2.0f/65536.0f) * ((256.0f * data[2]) + data[3]); break;
    case 0x2C: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x2D: task.value_f = (100.0f/128.0f) * data[2] - 100.0; break;
    case 0x2E: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x2F: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x30: task.value_f = data[2]; break;
    case 0x31: task.value_f = (256.0f * data[2]) + data[3]; break;
    case 0x32: task.value_f = ((256.0f * data[2]) + data[3]) / 4.0f; break;
    case 0x33: task.value_f = data[2]; break;
    case 0x34: task.value_f = (2.0f/65536.0f) * ((256.0f * data[2]) + data[3]); break;
    case 0x3C: task.value_f = (((256.0f * data[2]) + data[3]) / 10.0f) - 40.0f; break;
    case 0x41: task.value_s = get_monitor_status(data); break;
    case 0x42: task.value_f = ((256.0f * data[2]) + data[3]) / 1000.0f; break;
    case 0x43: task.value_f = (100.0f/255.0f) * ((256.0f * data[2]) + data[3]); break;
    case 0x44: task.value_f = (2.0f/65536.0f) * ((256.0f * data[2]) + data[3]); break;
    case 0x45: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x46: task.value_f = data[2] - 40.0f; break;
    case 0x47: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x49: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x4A: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x4C: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x4D: task.value_f = (256.0f * data[2]) + data[3]; break;
    case 0x4E: task.value_f = (256.0f * data[2]) + data[3]; break;
    case 0x51: task.value_s = get_fuel_type(data[2]); break;
    case 0x52: task.value_f = (100.0f / 255.0f) * data[2]; break;
    case 0x53: task.value_f = ((256.0f * data[2]) + data[3]) / 200.0f; break;
    case 0x59: task.value_f = 10.0f * ((256.0f * data[2]) + data[3]); break;
    case 0x5A: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x5B: task.value_f = (100.0f/255.0f) * data[2]; break;
    case 0x5C: task.value_f = data[2] - 40.0f; break;
    case 0x5D: task.value_f = (((256.0f * data[2]) + data[3]) / 128.0f) - 210.0f; break;
    case 0x5E: task.value_f = ((256.0f * data[2]) + data[3]) / 20.0f; break;
    case 0x61: task.value_f = data[2] - 125.0f; break;
    case 0x62: task.value_f = data[2] - 125.0f; break;
    case 0x63: task.value_f = (256.0f * data[2]) + data[3]; break;
    case 0x65: task.value_s = get_auxiliary_io_supported(data); break;
    case 0x67: task.value_f = get_ect(task.bits, data); break;
    case 0x6D: task.value_f = get_fuel_pressure_control_system(task.bits, data); break;
    default:
      ESP_LOGD(TAG, "Unsupported PID: %02X", pid);
      break;
  }
  
  if (!std::isnan(task.value_f)) {
    ESP_LOGD(TAG, "Calculated value: %.2f", task.value_f);
  }
  if (!task.value_s.empty()) {
    ESP_LOGD(TAG, "Calculated value: %s", task.value_s.c_str());
  }
}

void OBD2BLEClient::handle_mode_05(OBD2Task &task, const std::vector<uint8_t> &data) {
  std::string tid = format_hex(data[1]);
  task.value_f = data[2];
  // Handle Mode 05 specific logic here
}

void OBD2BLEClient::handle_mode_06(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0.0f;
  task.value_s = "";
  uint8_t pid = data[1];
  
  switch (pid) {
    case 0x01: break;
    case 0x02: break;
    case 0x21: break;
    case 0x31: break;
    case 0x35: break;
    case 0x36: break;
    case 0x3C: break;
    case 0x3D: break;
    case 0x41: break;
    case 0x42: break;
    case 0xA1: break;
    case 0xA2: break;
    case 0xA3: break;
    case 0xA4: break;
    case 0xA5: break;
    default:
      ESP_LOGD(TAG, "Unsupported PID: %02X", pid);
      break;
  }
}

void OBD2BLEClient::handle_mode_09(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0.0f;
  task.value_s = "";
  uint8_t pid = data[1];
  uint8_t indicator = data[2];
  switch (data[1]) {
    case 0x02: task.value_s = get_vin(data); break;
    case 0x04: task.value_s = get_cid(data); break;
    case 0x06: task.value_s = get_cvn(data); break;
    case 0x08: task.value_f = get_ipt(task.bits, data); break;
    case 0x0A: task.value_s = get_ect_name(data); break;
    case 0x14: break;
    default: break;
  }
}

void OBD2BLEClient::handle_mode_22(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_f = 0.0f;
  uint16_t pid = (data[1] << 8) | data[2];
  switch (pid) {
    case 0x1001: break;
    case 0x1002: break;
    case 0x1003: break;
    case 0x1004: break;
    case 0x1005: break;
    case 0x1006: break;
    case 0x1007: break;
    case 0x1008: break;
    case 0x1009: break;
    case 0x100A: break;
    case 0x101D: break;
    case 0x101E: break;
    case 0x1022: break;
    case 0x1024: break;
    case 0x103C: break;
    case 0x103D: break;
    case 0x1061: break;
    case 0x1063: break;
    case 0x1088: break;
    case 0x1089: break;
    case 0x10A5: task.value_f = (data[3] - 128.0f) / 2.0f; break;
    case 0x10A6: task.value_f = data[3] * (100.0f/255.0f); break;
    case 0x10AE: break;
    case 0x10AF: break;
    case 0x10B0: break;
    case 0x10B1: break;
    case 0x10B2: task.value_f = data[3]; break;
    case 0x10B4: task.value_f = data[3] - 50.0f; break;
    case 0x10B5: task.value_f = data[3] - 50.0f; break;
    case 0x10B6: break;
    case 0x10B7: break;
    case 0x10B8: break;
    case 0x10B9: break;
    case 0x10BA: break;
    case 0x10BC: break;
    case 0x10BE: break;
    case 0x10C1: break;
    case 0x10CB: break;
    case 0x10CC: break;
    case 0x10DE: break;
    case 0x10DF: break;
    case 0x10E3: break;
    case 0x10E5: break;
    case 0x10EC: break;
    case 0x10ED: break;
    case 0x10EE: break;
    case 0x10EF: break;
    case 0x10F0: break;
    case 0x10F1: break;
    case 0x10F3: break;
    case 0x10F4: break;
    case 0x10F5: break;
    case 0x110B: break;
    case 0x110C: break;
    case 0x110D: break;
    case 0x110E: break;
    case 0x1114: break;
    case 0x1121: task.value_f = ((data[3] * 256.0f) + data[4]) / 4.0f; break;
    case 0x1134: break;
    case 0x1137: task.value_f = data[3]; break;
    case 0x1165: break;
    case 0x1166: break;
    case 0x11A9: break;
    case 0x11AA: break;
    case 0x11AB: break;
    case 0x11AC: break;
    case 0x11AD: break;
    case 0x11AE: break;
    case 0x11AF: break;
    case 0x11BE: break;
    case 0x11BF: break;
    case 0x11C4: break;
    case 0x11C5: break;
    case 0x11C6: break;
    case 0x11C8: break;
    case 0x11CA: break;
    case 0x11CC: break;
    case 0x11CE: break;
    case 0x11D0: break;
    case 0x11D2: break;
    case 0x11D3: break;
    case 0x11D4: break;
    case 0x11D5: break;
    case 0x11D6: break;
    case 0x11D7: break;
    case 0x11DA: break;
    case 0x11DC: break;
    case 0x11DD: break;
    case 0x11DE: break;
    case 0x11E3: break;
    case 0x11E4: break;
    case 0x11F3: break;
    case 0x11F4: break;
    case 0x11F5: break;
    case 0x11FA: break;
    case 0x122E: break;
    case 0x1231: task.value_f = data[3]; break;
    case 0x1232: task.value_f = data[3]; break;
    case 0x1233: task.value_f = data[3]; break;
    case 0x1234: task.value_f = data[3]; break;
    case 0x1249: break;
    case 0x124A: task.value_f = data[3]; break;
    case 0x124C: task.value_f = data[3]; break;
    case 0x1265: break;
    case 0x1266: break;
    case 0x126A: break;
    case 0x126F: break;
    case 0x1272: break;
    case 0x1273: break;
    case 0x1274: break;
    case 0x1275: break;
    case 0x1276: break;
    case 0x1277: break;
    case 0x1278: break;
    case 0x1279: break;
    case 0x127F: break;
    case 0x1285: break;
    case 0x1286: break;
    case 0x1287: break;
    case 0x1288: break;
    case 0x1289: break;
    case 0x128A: break;
    case 0x128B: break;
    case 0x128C: break;
    case 0x128D: break;
    case 0x128E: break;
    case 0x128F: break;
    case 0x1290: break;
    case 0x1291: break;
    case 0x1292: break;
    case 0x1293: break;
    case 0x1295: break;
    case 0x1296: break;
    case 0x1297: break;
    case 0x1298: break;
    case 0x1299: break;
    case 0x129A: break;
    case 0x129B: break;
    case 0x129D: break;
    case 0x12A1: break;
    case 0x12A2: break;
    case 0x12A3: break;
    case 0x12A4: break;
    case 0x12C2: break;
    case 0x12C3: break;
    case 0x12C4: break;
    case 0x12C5: break;
    case 0x12C6: break;
    case 0x12C7: break;
    case 0x12C8: break;
    case 0x12C9: break;
    case 0x12CA: break;
    case 0x12CB: break;
    case 0x12CC: break;
    case 0x12CD: break;
    case 0x12CE: break;
    case 0x12CF: break;
    case 0x12D0: break;
    case 0x12D1: break;
    case 0x12D2: break;
    case 0x12D3: break;
    case 0x12DE: break;
    case 0x12DF: break;
    case 0x12E1: break;
    case 0x12E2: break;
    case 0x12E3: break;
    case 0x12E4: break;
    case 0x12E5: break;
    case 0x12E6: break;
    case 0x12EF: break;
    case 0x12F0: break;
    case 0x12F1: break;
    case 0x12F2: break;
    case 0x12F3: break;
    case 0x12F4: break;
    case 0x12F6: break;
    case 0x12F7: break;
    case 0x12F8: break;
    case 0x12F9: break;
    case 0x12FB: break;
    case 0x12FC: break;
    case 0x12FD: break;
    case 0x12FE: break;
    case 0x1308: break;
    case 0x1309: break;
    case 0x130C: break;
    case 0x130D: break;
    case 0x1314: break;
    case 0x1315: break;
    case 0x1316: break;
    case 0x1317: break;
    case 0x131A: break;
    case 0x131B: break;
    case 0x131C: break;
    case 0x131D: break;
    case 0x131F: break;
    case 0x1321: break;
    case 0x1322: break;
    case 0x1323: break;
    case 0x1324: break;
    case 0x1325: break;
    case 0x1326: break;
    case 0x1327: break;
    case 0x1328: break;
    case 0x1329: break;
    case 0x132A: break;
    case 0x132B: break;
    case 0x132C: break;
    case 0x132D: break;
    case 0x132E: break;
    case 0x132F: break;
    case 0x1330: break;
    case 0x1331: break;
    case 0x1332: break;
    case 0x1333: break;
    case 0x1334: break;
    case 0x1335: break;
    case 0x1336: break;
    case 0x1337: break;
    case 0x1338: break;
    case 0x1339: break;
    case 0x133A: break;
    case 0x133C: break;
    case 0x133D: break;
    case 0x133E: break;
    case 0x133F: break;
    case 0x1341: break;
    case 0x1342: break;
    case 0x1343: break;
    case 0x1344: break;
    case 0x1347: break;
    case 0x1348: break;
    case 0x1349: break;
    case 0x134A: break;
    case 0x134D: break;
    case 0x134F: break;
    case 0x1351: break;
    case 0x1352: break;
    case 0x1353: break;
    case 0x1354: break;
    case 0x1355: break;
    case 0x1356: break;
    case 0x1357: break;
    case 0x1358: break;
    case 0x1359: break;
    case 0x135A: break;
    case 0x135B: break;
    case 0x135C: break;
    case 0x135D: break;
    case 0x135E: break;
    case 0x135F: break;
    case 0x1361: break;
    case 0x1362: break;
    case 0x1363: break;
    case 0x1364: break;
    case 0x1365: break;
    case 0x1366: break;
    case 0x1367: break;
    case 0x1368: break;
    case 0x1369: break;
    case 0x136A: break;
    case 0x136B: break;
    case 0x136C: break;
    case 0x136D: break;
    case 0x136E: break;
    case 0x136F: break;
    case 0x1370: break;
    case 0x1371: break;
    case 0x1372: break;
    case 0x1373: break;
    case 0x1374: break;
    case 0x1375: break;
    case 0x1376: break;
    case 0x1377: break;
    case 0x1379: break;
    case 0x137A: break;
    case 0x137C: break;
    case 0x137D: break;
    case 0x137E: break;
    case 0x137F: break;
    case 0x1382: break;
    case 0x1383: break;
    case 0x1384: break;
    case 0x1385: break;
    default: break;
  }
}

void OBD2BLEClient::handle_dtc_response(OBD2Task &task, const std::vector<uint8_t> &data) {
  task.value_s = "";
  if (data.size() <= 2) {
    ESP_LOGD(TAG, "No Data");
    task.value_s = "";
  } else {
    for (size_t i = 1; i + 1 < data.size(); i += 2) {
      uint8_t high_byte = data[i];
      uint8_t low_byte = data[i + 1];

      char dtc_type;
      switch (high_byte >> 4) {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3: dtc_type = 'P'; break; // Powertrain
        case 0x4:
        case 0x5: dtc_type = 'C'; break; // Chassis
        case 0x6:
        case 0x7: dtc_type = 'B'; break; // Body
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB: dtc_type = 'U'; break; // Network
        default: dtc_type = '?'; break; // Undefined
      }

      char dtc[6];
      snprintf(dtc, sizeof(dtc), "%c%01X%02X", dtc_type, high_byte & 0x0F, low_byte);
      ESP_LOGD(TAG, "Decoded DTC: %s", dtc);
      task.value_s += std::string(dtc) + " ";
    }
  }
}

std::string OBD2BLEClient::get_monitor_status(const std::vector<uint8_t> &data) {
  std::string value = "";
  if (data.size() < 6) {
    return value;
  }
  
  std::string delimiter = " ";
  
  if (data[1] == 0x01 && is_bit_set(data[2], 7)) add_string(value, "A7", delimiter);
  
  if (is_bit_set(data[3], 2)) add_string(value, "B2", delimiter);
  if (!is_bit_set(data[3], 6)) add_string(value, "B6", delimiter);
  
  if (is_bit_set(data[3], 1)) add_string(value, "B1", delimiter);
  if (!is_bit_set(data[3], 5)) add_string(value, "B5", delimiter);
  
  if (is_bit_set(data[3], 0)) add_string(value, "B0", delimiter);
  if (!is_bit_set(data[3], 4)) add_string(value, "B4", delimiter);
  
  if (is_bit_set(data[4], 7)) add_string(value, "C7", delimiter);
  if (!is_bit_set(data[5], 7)) add_string(value, "D7", delimiter);
  
  if (is_bit_set(data[4], 6)) add_string(value, "C6", delimiter);
  if (!is_bit_set(data[5], 6)) add_string(value, "D6", delimiter);
  
  if (is_bit_set(data[4], 5)) add_string(value, "C5", delimiter);
  if (!is_bit_set(data[5], 5)) add_string(value, "D5", delimiter);
  
  if (is_bit_set(data[4], 4)) add_string(value, "C4", delimiter);
  if (!is_bit_set(data[5], 4)) add_string(value, "D4", delimiter);
  
  if (is_bit_set(data[4], 3)) add_string(value, "C3", delimiter);
  if (!is_bit_set(data[5], 3)) add_string(value, "D3", delimiter);
  
  if (is_bit_set(data[4], 2)) add_string(value, "C2", delimiter);
  if (!is_bit_set(data[5], 2)) add_string(value, "D2", delimiter);
  
  if (is_bit_set(data[4], 1)) add_string(value, "C1", delimiter);
  if (!is_bit_set(data[5], 1)) add_string(value, "D1", delimiter);
  
  if (is_bit_set(data[4], 0)) add_string(value, "C0", delimiter);
  if (!is_bit_set(data[5], 0)) add_string(value, "D0", delimiter);
  
  return value;
}

std::string OBD2BLEClient::get_fuel_system_status(const uint8_t &num) {
  switch (num) {
    case 0x00: return "Motor is off";
    case 0x01: return "Open loop due to insufficient engine temp";
    case 0x02: return "Closed loop, using o2 sensor feedback";
    case 0x04: return "Open loop due to engine load OR fuel cut due to deceleration";
    case 0x08: return "Open loop due to system failure";
    case 0x10: return "Closed loop, using at least one o2 sensor but there is fault in feedback";
    default: return "Unknown";
  }
}

std::string OBD2BLEClient::get_o2_sensor_presence(const uint8_t &num) {
  std::string value;
  if (num != 0x00) {
    value = "";
    bool first = true; 
    for (int i = 0; i < 8; ++i) {
      if (num & (1 << i)) {
        if (!first) {
          value += " ";
        }
        first = false;
        if (i < 4) {
          value += "B1S" + std::to_string(i + 1);
        } else {
          value += "B2S" + std::to_string(i - 3);
        }
      }
    }
  } else {
    value = "No Oxygen Sensors";
  }
  return value;
}

std::string OBD2BLEClient::get_standard_name(const uint8_t &num) {
  switch (num) {
    case 0x01: return "OBD-II";
    case 0x02: return "OBD";
    case 0x03: return "OBD and OBD-II";
    case 0x04: return "OBD-I";
    case 0x05: return "Not OBD compliant";
    case 0x06: return "EOBD";
    case 0x07: return "EOBD and OBD-II";
    case 0x08: return "EOBD and OBD";
    case 0x09: return "EOBD, OBD and OBD II";
    case 0x0A: return "JOBD";
    case 0x0B: return "JOBD and OBD II";
    case 0x0C: return "JOBD and EOBD";
    case 0x0D: return "JOBD, EOBD, and OBD II";
    case 0x11: return "EMD";
    case 0x12: return "EMD+";
    case 0x13: return "HD OBD-C";
    case 0x14: return "HD OBD";
    case 0x15: return "WWH OBD";
    case 0x17: return "HD EOBD-I";
    case 0x18: return "HD EOBD-I N";
    case 0x19: return "HD EOBD-II";
    case 0x1A: return "HD EOBD-II N";
    case 0x1C: return "OBDBr-1";
    case 0x1D: return "OBDBr-2";
    case 0x1E: return "KOBD";
    case 0x1F: return "IOBD I";
    case 0x20: return "IOBD II";
    case 0x21: return "HD EOBD-IV";
    default:  return "Unknown";
  }
}

std::string OBD2BLEClient::get_fuel_type(const uint8_t &num) {
  switch (num) {
    case 0x01: return "Gasoline";
    case 0x02: return "Methanol";
    case 0x03: return "Ethanol";
    case 0x04: return "Diesel";
    case 0x05: return "LPG";
    case 0x06: return "CNG";
    case 0x07: return "Propane";
    case 0x08: return "Electric";
    case 0x09: return "Bifuel running Gasoline";
    case 0x0A: return "Bifuel running Methanol";
    case 0x0B: return "Bifuel running Ethanol";
    case 0x0C: return "Bifuel running LPG";
    case 0x0D: return "Bifuel running CNG";
    case 0x0E: return "Bifuel running Propane";
    case 0x0F: return "Bifuel running Electricity";
    case 0x10: return "Bifuel running electric and combustion engine";
    case 0x11: return "Hybrid gasoline";
    case 0x12: return "Hybrid Ethanol";
    case 0x13: return "Hybrid Diesel";
    case 0x14: return "Hybrid Electric";
    case 0x15: return "Hybrid running electric and combustion engine";
    case 0x16: return "Hybrid Regenerative";
    case 0x17: return "Bifuel running diesel";
    default:  return "Unknown";
  }
}

std::string OBD2BLEClient::get_auxiliary_io_supported(const std::vector<uint8_t> &data) {
  std::string value = "";
  if (data.size() < 4) {
    return value;
  }
  
  std::string delimiter = " ";
  
  if (is_bit_set(data[2], 3)) add_string(value, "A3", delimiter);
  if (!is_bit_set(data[3], 3)) add_string(value, "B3", delimiter);
  
  if (is_bit_set(data[2], 2)) add_string(value, "A2", delimiter);
  if (!is_bit_set(data[3], 2)) add_string(value, "B2", delimiter);
  
  if (is_bit_set(data[2], 1)) add_string(value, "A1", delimiter);
  if (!is_bit_set(data[3], 1)) add_string(value, "B1", delimiter);
  
  if (is_bit_set(data[2], 0)) add_string(value, "A0", delimiter);
  if (!is_bit_set(data[3], 0)) add_string(value, "B0", delimiter);
  
  return value;
}

float OBD2BLEClient::get_ect(const std::string &bits, const std::vector<uint8_t> &data) {
  float value = 0.0f;
  if (data.size() < 5) {
    return value;
  }
  
  if (bits == "B" && is_bit_set(data[2], 0)) {
    value = data[3] - 40.0;
  } else if (bits == "C" && is_bit_set(data[2], 1)) {
    value = data[4] - 40.0;
  }
  
  return value;
}

float OBD2BLEClient::get_fuel_pressure_control_system(const std::string &bits, const std::vector<uint8_t> &data) {
  float value = 0.0f;
  if (data.size() < 13) {
    return value;
  }
  
  if (bits == "K" && is_bit_set(data[2], 5)) {
    value = data[12] - 40.0f;
  } else if (bits == "IJ" && is_bit_set(data[2], 4)) {
    value = ((256.0f * data[10]) + data[11]) * 10.0f;
  } else if (bits == "GH" && is_bit_set(data[2], 3)) {
    value = ((256.0f * data[8]) + data[9]) * 10.0f;
  } else if (bits == "F" && is_bit_set(data[2], 2)) {
    value = data[7] - 40.0f;
  } else if (bits == "DE" && is_bit_set(data[2], 1)) {
    value = ((256.0f * data[5]) + data[6]) * 10.0f;
  } else if (bits == "BC" && is_bit_set(data[2], 0)) {
    value = ((256.0f * data[3]) + data[4]) * 10.0f;
  }
  
  return value;
}

std::string OBD2BLEClient::get_vin(const std::vector<uint8_t> &data) {
  std::string value = "";
  for (size_t i = 3; i < data.size(); ++i) {
    value += static_cast<char>(data[i]);
  }
  return value;
}

std::string OBD2BLEClient::get_cid(const std::vector<uint8_t> &data) {
  std::string value(data.begin() + 3, data.end());
  return value;
}

std::string OBD2BLEClient::get_cvn(const std::vector<uint8_t> &data) {
  std::string value = "";
  for (size_t i = 3; i < data.size(); ++i) {
    value += format_hex_pretty(data[i]);
  }
  return value;
}

float OBD2BLEClient::get_ipt(const std::string &bits, const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "In-use tracking data: %s", format_hex_pretty(data).c_str());
  std::vector<uint8_t> items(data.begin() + 3, data.end());
  uint16_t value;
  if (bits == "AA" && items.size() >= 2) value = (items[0] << 8) | items[1];
  else if (bits == "BB" && items.size() >= 4) value = (items[2] << 8) | items[3];
  else if (bits == "CC" && items.size() >= 6) value = (items[4] << 8) | items[5];
  else if (bits == "DD" && items.size() >= 8) value = (items[6] << 8) | items[7];
  else if (bits == "EE" && items.size() >= 10) value = (items[8] << 8) | items[9];
  else if (bits == "FF" && items.size() >= 12) value = (items[10] << 8) | items[11];
  else if (bits == "GG" && items.size() >= 14) value = (items[12] << 8) | items[13];
  else if (bits == "HH" && items.size() >= 16) value = (items[14] << 8) | items[15];
  else if (bits == "II" && items.size() >= 18) value = (items[16] << 8) | items[17];
  else if (bits == "JJ" && items.size() >= 20) value = (items[18] << 8) | items[19];
  else if (bits == "KK" && items.size() >= 22) value = (items[20] << 8) | items[21];
  else if (bits == "LL" && items.size() >= 24) value = (items[22] << 8) | items[23];
  else if (bits == "MM" && items.size() >= 26) value = (items[24] << 8) | items[25];
  else if (bits == "NN" && items.size() >= 28) value = (items[26] << 8) | items[27];
  else if (bits == "OO" && items.size() >= 30) value = (items[28] << 8) | items[29];
  else if (bits == "PP" && items.size() >= 32) value = (items[30] << 8) | items[31];
  else if (bits == "QQ" && items.size() >= 34) value = (items[32] << 8) | items[33];
  else if (bits == "RR" && items.size() >= 36) value = (items[34] << 8) | items[35];
  else if (bits == "SS" && items.size() >= 38) value = (items[36] << 8) | items[37];
  else if (bits == "TT" && items.size() >= 40) value = (items[38] << 8) | items[39];
  else value = 0;
  return static_cast<float>(value);
}

std::string OBD2BLEClient::get_ect_name(const std::vector<uint8_t> &data) {
  std::string value(data.begin() + 3, data.end());
  return value;
}

bool OBD2BLEClient::is_bit_set(const uint8_t &value, const int &position) {
  uint8_t mask = 1 << position;
  return (value & mask) != 0;
}

std::string OBD2BLEClient::get_ipt_name(const size_t &num) {
  switch (num) {
    case 0: return "OBDCOND";
    case 2: return "IGNCNTR";
    case 4: return "CATCOMP1";
    case 6: return "CATCOND1";
    case 8: return "CATCOMP2";
    case 10: return "CATCOND2";
    case 12: return "O2COMP1";
    case 14: return "O2COND1";
    case 16: return "O2COMP2";
    case 18: return "O2COND2";
    case 20: return "EGRCOMP";
    case 22: return "EGRCOND";
    case 24: return "AIRCOMP";
    case 26: return "AIRCOND";
    case 28: return "EVAPCOMP";
    case 30: return "EVAPCOND";
    case 32: return "SO2COMP1";
    case 34: return "SO2COND1";
    case 36: return "SO2COMP2";
    case 38: return "SO2COND2";
    default: return "Unknown";
  }
}

void OBD2BLEClient::add_string(std::string &base_str, const std::string &add_str, const std::string &delimiter) {
  if (base_str.empty()) {
    base_str += add_str;
  } else {
    base_str += delimiter + add_str;
  }
}

void OBD2BLEClient::publish_data(OBD2Task &task) {
  if (!std::isnan(task.value_f) && task.sensor != nullptr) {
    std::string sensor_name = transform_string(task.sensor->get_name());
    std::map<std::string, OBD2Data> obd2_data = {
      {sensor_name, OBD2Data(task.value_f)},
    };
    publish_message(topic_, obd2_data);
    task.published = true;
  } else if (task.text_sensor != nullptr && task.codes.empty() && task.bits.empty()) {
    std::string sensor_name = transform_string(task.text_sensor->get_name());
    std::map<std::string, OBD2Data> obd2_data = {
      {sensor_name, OBD2Data(task.value_s)},
    };
    publish_message(topic_, obd2_data);
    task.published = true;
  } else if (task.text_sensor != nullptr && !task.codes.empty()) {
    if (current_code_index_ < task.codes.size()) {
      bool code_state = false;
      if (!task.value_s.empty()) {
        code_state = (task.value_s.find(task.codes[current_code_index_]) != std::string::npos);
        if (code_state) {
          ESP_LOGI(TAG, "DTC code %s found in response", task.codes[current_code_index_].c_str());
        }
      }
      std::string code_name = task.binary_sensors[current_code_index_]->get_name();
      std::string sensor_name = transform_string(task.binary_sensors[current_code_index_]->get_name());
      if (code_name.find(task.codes[current_code_index_])) {
        std::map<std::string, OBD2Data> obd2_data = {
          {sensor_name, OBD2Data(code_state)},
        };
        publish_message(topic_, obd2_data);
        ESP_LOGD(TAG, "Published Code: %s", task.codes[current_code_index_].c_str());
      } else {
        ESP_LOGW(TAG, "No match, Code: %s, Sensor: %s", task.codes[current_code_index_].c_str(), task.binary_sensors[current_code_index_]->get_name().c_str());
      }
      current_code_index_++;
    } else {
      ESP_LOGD(TAG, "Code publishing completed");
      std::string sensor_name = transform_string(task.text_sensor->get_name());
      std::map<std::string, OBD2Data> obd2_data = {
        {sensor_name, OBD2Data(task.value_s)},
      };
      publish_message(topic_, obd2_data);
      task.published = true;
    }
  } else if (!task.binary_sensors.empty() && !task.bits.empty()) {
    ESP_LOGI(TAG, "Text %s, Bits %s", task.value_s.c_str(), task.bits.c_str());
    bool bits_state = false;
    bits_state = (task.value_s.find(task.bits) != std::string::npos);
    if (bits_state) {
      ESP_LOGI(TAG, "Bits %s found in response", task.bits.c_str());
    }
    if (task.binary_sensors.size() == 1) {
      std::string sensor_name = transform_string(task.binary_sensors[0]->get_name());
      std::map<std::string, OBD2Data> obd2_data = {
        {sensor_name, OBD2Data(bits_state)},
      };
      publish_message(topic_, obd2_data);
      ESP_LOGD(TAG, "Published Bits: %s", task.bits.c_str());
    } else {
      ESP_LOGD(TAG, "No match, Bits: %s", task.bits.c_str());
    }
    task.published = true;
  } else {
    task.published = true;
  }
}

void OBD2BLEClient::publish_message(const std::string &topic, const std::map<std::string, OBD2Data> &data) {
  this->publish_json(topic, [data](JsonObject root) {
    JsonObject state = root["state"].to<JsonObject>();
    JsonObject reported = state["reported"].to<JsonObject>();
    for (auto it = data.begin(); it != data.end(); ++it) {
      const std::string &key = it->first;
      const OBD2Data &value = it->second;

      switch (value.get_type()) {
        case OBD2Data::STRING:
          ESP_LOGD("OBD2BLEClient", "Adding key '%s' with string value '%s'", key.c_str(), value.get_string().c_str());
          reported[key.c_str()] = value.get_string().c_str();
          break;
        case OBD2Data::FLOAT:
          ESP_LOGD("OBD2BLEClient", "Adding key '%s' with float value '%f'", key.c_str(), value.get_float());
          reported[key.c_str()] = value.get_float();
          break;
        case OBD2Data::BOOL:
          ESP_LOGD("OBD2BLEClient", "Adding key '%s' with bool value '%s'", key.c_str(), value.get_bool() ? "true" : "false");
          reported[key.c_str()] = value.get_bool();
          break;
        default:
          ESP_LOGW("OBD2BLEClient", "Unknown type for key '%s'", key.c_str());
      }
    }
  });
}

std::string OBD2BLEClient::transform_string(const std::string &input) {
  std::string result;
  for (char ch : input) {
    if (std::isupper(ch)) {
      result += std::tolower(ch);
    } else if (std::isspace(ch) || std::ispunct(ch)) {
      result += '_';
    } else {
      result += ch;
    }
  }
  return result;
}

void OBD2BLEClient::cleanup() {
  ESP_LOGW(TAG, "Cleaning up...");
  mtu_configured_ = false;
  notifications_ready_ = false;
  descr_write_done_ = false;
  current_task_index_ = 0;
  current_code_index_ = 0;
  for (auto &task : task_queue_) {
    task.status = PENDING;
    task.data.clear();
    task.can_id_map.clear();
    task.published = false;
  }
}

}  // namespace obd2_ble
}  // namespace esphome

#endif
