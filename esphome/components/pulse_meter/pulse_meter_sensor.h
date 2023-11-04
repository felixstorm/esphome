#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>

// ISR debugging only (defined(tskKERNEL_VERSION_NUMBER) => FreeRTOS and QueueHandle_t exist)
#if defined(USE_LOGGER) && ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE && defined(tskKERNEL_VERSION_NUMBER)
#define USE_PM_ISR_DEBUG_LOGGING
#endif

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  enum InternalFilterMode {
    FILTER_EDGE = 0,
    FILTER_PULSE,
  };

  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_filter_us(uint32_t filter) { this->filter_us_ = filter; }
  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { this->filter_mode_ = mode; }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  static void edge_intr(PulseMeterSensor *sensor);
  static void pulse_intr(PulseMeterSensor *sensor);

  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};

  // Variables used in the loop
  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;
  uint32_t total_pulses_ = 0;
  uint32_t last_processed_edge_us_ = 0;

  // This struct (and the two pointers) are used to pass data between the ISR and loop.
  // These two pointers are exchanged each loop.
  // Therefore you can't use data in the pointer to loop receives to set values in the pointer to loop sends.
  // As a result it's easiest if you only use these pointers to send data from the ISR to the loop.
  // (except for resetting the values)
  struct State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t count_ = 0;
  };
  State state_[2];
  volatile State *set_ = state_;
  volatile State *get_ = state_ + 1;

  // Only use these variables in the ISR
  ISRInternalGPIOPin isr_pin_;
  uint32_t last_edge_candidate_us_ = 0;
  uint32_t last_intr_ = 0;
  bool in_pulse_ = false;
  bool last_pin_val_ = false;

#ifdef USE_PM_ISR_DEBUG_LOGGING
  // ISR debugging only
  struct IsrStateSnapshot {
    struct IsrStateValues {
      uint32_t last_edge_candidate_us_ = 0;
      uint32_t last_intr_ = 0;
      bool last_pin_val_ = false;
      bool in_pulse_ = false;
      State set_;
    };
    uint32_t time = 0;
    uint32_t time_since_last = 0;
    bool pin_val = false;
    IsrStateValues on_enter;
    int8_t state_machine = 0;
    IsrStateValues on_exit;
  };
  QueueHandle_t isr_logger_queue_ = nullptr;
#endif
};

}  // namespace pulse_meter
}  // namespace esphome
