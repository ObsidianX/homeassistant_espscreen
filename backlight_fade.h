#pragma once
#include <algorithm>
#include <cmath>
#include <driver/ledc.h>
#include "esphome/components/ledc/ledc_output.h"
#include "esphome/components/light/light_state.h"

// Backlight fades through the LEDC hardware fader. ESPHome's software
// transition steps the duty from the main loop, and the full LVGL redraws that
// standby triggers on the Guition's RGB panel stall that loop long enough to
// make dimming visibly jerky. The peripheral steps on its own clock instead.
namespace backlight_fade {
template<typename T> struct Peek : T {
  static uint8_t channel(T &o) { return o.*(&Peek::channel_); }
  static uint8_t bits(T &o) { return o.*(&Peek::bit_depth_); }
};
inline bool installed = false, hardware_owns = false;
inline ledc_mode_t mode(uint8_t channel) {
#ifdef SOC_LEDC_SUPPORT_HS_MODE
  return channel < 8 ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE;
#else
  return LEDC_LOW_SPEED_MODE;
#endif
}
// Returns false when the fader is unavailable; the caller keeps ESPHome's transition.
inline bool start(esphome::ledc::LEDCOutput &output, esphome::light::LightState &light, float level, uint32_t ms) {
#ifndef SOC_LEDC_SUPPORT_FADE_STOP
  return false;  // Classic ESP32 cannot interrupt a running fade for a touch wake.
#else
  using P = Peek<esphome::ledc::LEDCOutput>;
  const auto channel = static_cast<ledc_channel_t>(P::channel(output));
  const uint8_t bits = P::bits(output);
  if (!bits) return false;
  if (!installed) installed = ledc_fade_func_install(0) == ESP_OK;
  if (!installed) return false;
  const auto speed = mode(channel);
  const uint32_t max_duty = (1u << bits) - 1;
  ledc_fade_stop(speed, channel);
  // ESPHome parks 0 % and 100 % with ledc_stop and leaves a stale duty register.
  // Restart the channel at the level it believes, unless a fade already owns it.
  float current = 0;
  light.current_values_as_brightness(&current);
  const uint32_t from = hardware_owns ? ledc_get_duty(speed, channel) : static_cast<uint32_t>(std::lround(current * max_duty));
  ledc_set_duty(speed, channel, from);
  ledc_update_duty(speed, channel);
  hardware_owns = true;
  const auto target = static_cast<uint32_t>(std::lround(light.gamma_correct_lut(std::clamp(level, 0.0f, 1.0f)) * max_duty));
  return ledc_set_fade_with_time(speed, channel, target, ms) == ESP_OK &&
         ledc_fade_start(speed, channel, LEDC_FADE_NO_WAIT) == ESP_OK;
#endif
}
// Hand the finished level back to ESPHome so HA and later transitions start from the truth.
inline void sync(esphome::ledc::LEDCOutput &output, esphome::light::LightState &light, float level) {
#ifdef SOC_LEDC_SUPPORT_FADE_STOP
  if (hardware_owns) ledc_fade_stop(mode(Peek<esphome::ledc::LEDCOutput>::channel(output)), static_cast<ledc_channel_t>(Peek<esphome::ledc::LEDCOutput>::channel(output)));
#endif
  light.turn_on().set_transition_length(0).set_brightness(std::clamp(level, 0.0f, 1.0f)).perform();
  hardware_owns = false;
}
}  // namespace backlight_fade
