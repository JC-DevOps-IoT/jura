#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"
#include <map>

namespace esphome {
namespace jura {

// Estados de la máquina de estados
enum class JuraState {
  IDLE,
  SENDING_RT,
  WAITING_RT,
  SENDING_IC,
  WAITING_IC,
};

class Jura : public PollingComponent, public uart::UARTDevice {
 public:

  void set_model(const std::string &m) { model_ = m; }
  void register_metric_sensor(const std::string &key, sensor::Sensor *s) { numeric_[key] = s; }
  void register_text_sensor(const std::string &key, text_sensor::TextSensor *t) { text_[key] = t; }

  void publish_number(const std::string &key, float value) {
    auto it = numeric_.find(key);
    if (it != numeric_.end() && it->second) it->second->publish_state(value);
  }
  void publish_text(const std::string &key, const std::string &value) {
    auto it = text_.find(key);
    if (it != text_.end() && it->second) it->second->publish_state(value);
  }

  // Envía un comando codificado a la Jura (solo TX, no espera respuesta)
  void send_command_(const std::string &cmd) {
    while (available()) { read(); }  // flush RX
    rx_buffer_.clear();
    rx_bit_ = 0;
    rx_inbyte_ = 0;

    std::string out = cmd + "\r\n";
    for (int i = 0; i < (int)out.size(); i++) {
      uint8_t src = static_cast<uint8_t>(out[i]);
      for (int s = 0; s < 8; s += 2) {
        uint8_t rawbyte = 0xFF;
        bitWrite(rawbyte, 2, bitRead(src, s + 0));
        bitWrite(rawbyte, 5, bitRead(src, s + 1));
        write(rawbyte);
      }
      delay(8);  // delay entre chars es obligatorio por protocolo Jura
    }
    rx_start_ms_ = millis();
  }

  // Lee bytes disponibles sin bloquear — acumula en rx_buffer_
  // Retorna true cuando llega \r\n completo
  bool read_nonblocking_() {
    while (available()) {
      uint8_t rawbyte = static_cast<uint8_t>(read());
      bitWrite(rx_inbyte_, rx_bit_ + 0, bitRead(rawbyte, 2));
      bitWrite(rx_inbyte_, rx_bit_ + 1, bitRead(rawbyte, 5));
      rx_bit_ += 2;
      if (rx_bit_ >= 8) {
        rx_bit_ = 0;
        rx_buffer_.push_back(static_cast<char>(rx_inbyte_));
        rx_inbyte_ = 0;
      }
      // Detectar fin de mensaje
      size_t sz = rx_buffer_.size();
      if (sz >= 2 && rx_buffer_[sz-2] == '\r' && rx_buffer_[sz-1] == '\n') {
        return true;
      }
    }
    return false;
  }

  // Llamado por PollingComponent cada update_interval — solo dispara el ciclo
  void update() override {
    if (state_ != JuraState::IDLE) {
      ESP_LOGW("jura", "update() called while busy, skipping");
      return;
    }
    send_command_("RT:0000");
    state_ = JuraState::WAITING_RT;
  }

  // loop() es llamado por ESPHome en cada ciclo — nunca bloquea
  void loop() override {
    switch (state_) {

      case JuraState::IDLE:
        break;

      case JuraState::WAITING_RT: {
        if (read_nonblocking_()) {
          // Respuesta RT completa
          std::string result = rx_buffer_.substr(0, rx_buffer_.size() - 2);
          if (result.size() >= 64) {
            process_rt_(result);
          } else {
            ESP_LOGW("jura", "RT response too short: len=%d", (int)result.size());
          }
          // Pasar a IC
          send_command_("IC:");
          state_ = JuraState::WAITING_IC;

        } else if (millis() - rx_start_ms_ > 5000) {
          ESP_LOGW("jura", "RT timeout - cafetera no responde");
          state_ = JuraState::IDLE;
        }
        break;
      }

      case JuraState::WAITING_IC: {
        if (read_nonblocking_()) {
          std::string ic = rx_buffer_.substr(0, rx_buffer_.size() - 2);
          if (ic.size() >= 7) {
            process_ic_(ic);
          } else {
            ESP_LOGW("jura", "IC response too short: len=%d", (int)ic.size());
          }
          state_ = JuraState::IDLE;

        } else if (millis() - rx_start_ms_ > 5000) {
          ESP_LOGW("jura", "IC timeout - cafetera no responde");
          state_ = JuraState::IDLE;
        }
        break;
      }

      default:
        state_ = JuraState::IDLE;
        break;
    }
  }

  // cmd2jura público sigue disponible para botones desde YAML
  std::string cmd2jura(std::string cmd) {
    state_ = JuraState::IDLE;
    rx_buffer_.clear();
    rx_bit_ = 0;
    rx_inbyte_ = 0;
    send_command_(cmd);
    uint32_t start = millis();
    while (millis() - start < 3000) {
      if (read_nonblocking_()) {
        return rx_buffer_.substr(0, rx_buffer_.size() - 2);
      }
      delay(5);
      App.feed_wdt();  // <- alimenta el watchdog mientras espera
    }
    ESP_LOGW("jura", "cmd2jura timeout for: %s", cmd.c_str());
    return "";
  }

 protected:
  void process_rt_(const std::string &result) {
    std::vector<long> current = parse_all_counters_(result);
    for (int n = 1; n <= 16; n++) {
      publish_number("counter_" + std::to_string(n), get_counter_n_(current, n));
    }
    publish_counter_changes_(current);
  }

  void process_ic_(const std::string &ic) {
    byte a = static_cast<byte>(strtol(ic.substr(3,2).c_str(), NULL, 16));
    byte b = static_cast<byte>(strtol(ic.substr(5,2).c_str(), NULL, 16));

    publish_ic_bits_if_changed_(a, b);

    byte trayBit       = bitRead(a, 4);
    byte left_readyBit = bitRead(a, 2);
    byte tankBit       = bitRead(b, 5);
    byte right_busyBit = bitRead(b, 6);

    publish_text("tray_status",       (trayBit == 1) ? "Present" : "Missing");
    publish_text("water_tank_status", (tankBit == 1) ? "Fill Tank" : "OK");

    std::string machine_status = "Ready";
    if (trayBit == 0)       machine_status = "Tray Missing";
    if (tankBit == 1)       machine_status = "Fill Tank";
    if (right_busyBit == 1) machine_status = "Busy (Milk Drink)";
    if (left_readyBit == 0) machine_status = "Busy (Coffee Drink)";
    publish_text("machine_status", machine_status);
  }

  long get_counter_n_(const std::vector<long> &v, int n) const {
    const size_t idx = (n >= 1) ? (size_t)(n - 1) : (size_t)-1;
    if (idx < v.size()) return v[idx];
    return -1;
  }

  std::vector<long> parse_all_counters_(const std::string &rt) const {
    std::vector<long> out;
    for (size_t pos = 3; pos + 4 <= rt.size(); pos += 4)
      out.push_back(strtol(rt.substr(pos, 4).c_str(), nullptr, 16));
    return out;
  }

  void publish_counter_changes_(const std::vector<long> &current) {
    if (!last_counters_initialized_) {
      last_counters_ = current;
      last_counters_initialized_ = true;
      return;
    }
    std::string msg;
    bool any = false;
    char buf[48];
    for (size_t i = 0; i < std::max(last_counters_.size(), current.size()); ++i) {
      long prev = (i < last_counters_.size()) ? last_counters_[i] : -1;
      long now  = (i < current.size())        ? current[i]         : -1;
      if (prev != now) {
        if (any) msg += ", ";
        snprintf(buf, sizeof(buf), "counter_%u %ld\u2192%ld", (unsigned)(i+1), prev, now);
        msg += buf;
        any = true;
      }
    }
    if (any) {
      publish_text("counters_changed", msg);
      ESP_LOGD("jura", "Changed: %s", msg.c_str());
    }
    last_counters_ = current;
  }

  static inline void byte_to_bits(uint8_t v, char out[9]) {
    for (int i = 7; i >= 0; --i) out[7-i] = (v & (1u << i)) ? '1' : '0';
    out[8] = '\0';
  }

  void publish_ic_bits_if_changed_(uint8_t a, uint8_t b) {
    if (!ic_bits_initialized_ || a != last_ic_a_ || b != last_ic_b_) {
      char abits[9], bbits[9], buf[32];
      byte_to_bits(a, abits);
      byte_to_bits(b, bbits);
      snprintf(buf, sizeof(buf), "A=%s B=%s", abits, bbits);
      publish_text("ic_bits", std::string(buf));
      last_ic_a_ = a;
      last_ic_b_ = b;
      ic_bits_initialized_ = true;
      ESP_LOGD("jura", "IC bits changed: %s", buf);
    }
  }

  // Estado de la máquina
  JuraState state_{JuraState::IDLE};
  uint32_t rx_start_ms_{0};

  // Buffer RX compartido
  std::string rx_buffer_;
  int rx_bit_{0};
  uint8_t rx_inbyte_{0};

  std::string model_{"UNKNOWN"};
  std::map<std::string, sensor::Sensor*>          numeric_;
  std::map<std::string, text_sensor::TextSensor*> text_;

  std::vector<long> last_counters_;
  bool last_counters_initialized_{false};
  uint8_t last_ic_a_{0};
  uint8_t last_ic_b_{0};
  bool ic_bits_initialized_{false};
};

}  // namespace jura
}  // namespace esphome
