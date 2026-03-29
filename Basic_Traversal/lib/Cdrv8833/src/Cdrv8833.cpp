#include "Cdrv8833.h"

#include <Arduino.h>
#include <cmath>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define CDRV8833_USE_NEW_LEDC_API 1
#else
#define CDRV8833_USE_NEW_LEDC_API 0
#endif

static bool drv_ledc_setup(uint8_t channel, uint32_t frequency, uint8_t resolution) {
#if CDRV8833_USE_NEW_LEDC_API
  (void)channel;
  (void)frequency;
  (void)resolution;
  return true;
#else
  return ledcSetup(channel, frequency, resolution) > 0.0;
#endif
}

static bool drv_ledc_attach(uint8_t pin, uint8_t channel, uint32_t frequency, uint8_t resolution) {
#if CDRV8833_USE_NEW_LEDC_API
  return ledcAttachChannel(pin, frequency, resolution, channel);
#else
  (void)frequency;
  (void)resolution;
  ledcAttachPin(pin, channel);
  return true;
#endif
}

static void drv_ledc_detach(uint8_t pin) {
#if CDRV8833_USE_NEW_LEDC_API
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
}

static void drv_ledc_write(uint8_t channel, uint32_t dutyCycle) {
#if CDRV8833_USE_NEW_LEDC_API
  ledcWriteChannel(channel, dutyCycle);
#else
  ledcWrite(channel, dutyCycle);
#endif
}

Cdrv8833::Cdrv8833() {
  m_in1Pin = -1;
  m_in2Pin = -1;
  m_power = 0;
  m_swapDirection = false;
  m_decayMode = drv8833DecaySlow;
  m_channel = 0;
  m_frequency = PWM_FREQUENCY;
}

Cdrv8833::Cdrv8833(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
  m_in1Pin = -1;
  m_in2Pin = -1;
  m_power = 0;
  m_swapDirection = false;
  m_decayMode = drv8833DecaySlow;
  m_channel = 0;
  m_frequency = PWM_FREQUENCY;
  init(in1Pin, in2Pin, channel, swapDirection);
}

Cdrv8833::~Cdrv8833() {
  stop();
}

bool Cdrv8833::init(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
  if (channel > 15) {
    return false;
  }

  if ((m_in1Pin != -1) && (m_in2Pin != -1)) {
    stop();
  }

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  m_in1Pin = (int8_t)in1Pin;
  m_in2Pin = (int8_t)in2Pin;
  m_power = 0;
  m_swapDirection = swapDirection;
  m_channel = channel;
  m_decayMode = drv8833DecaySlow;
  m_frequency = PWM_FREQUENCY;

  return drv_ledc_setup(m_channel, m_frequency, PWM_BIT_RESOLUTION);
}

bool Cdrv8833::move(int8_t power) {
  if (m_in1Pin == -1 || m_in2Pin == -1) {
    return false;
  }

  if (power == 0) {
    stop();
    return true;
  }

  if (power > 100) {
    power = 100;
  }
  if (power < -100) {
    power = -100;
  }
  m_power = power;

  if (m_swapDirection) {
    power = -power;
  }

  float value = (float)((1 << PWM_BIT_RESOLUTION) - 1) * ((float)abs(power)) / 100.0f;
  uint32_t dutyCycle = (uint32_t)((value - truncf(value)) < 0.5f ? value : value + 1.0f);

  if (m_decayMode == drv8833DecaySlow) {
    dutyCycle = ((1u << PWM_BIT_RESOLUTION) - 1u) - dutyCycle;
  }

  if (power > 0) { // forward
    if (m_decayMode == drv8833DecayFast) {
      drv_ledc_detach((uint8_t)m_in2Pin);
      digitalWrite(m_in2Pin, LOW);
      if (!drv_ledc_attach((uint8_t)m_in1Pin, m_channel, m_frequency, PWM_BIT_RESOLUTION)) {
        return false;
      }
    } else {
      drv_ledc_detach((uint8_t)m_in1Pin);
      digitalWrite(m_in1Pin, HIGH);
      if (!drv_ledc_attach((uint8_t)m_in2Pin, m_channel, m_frequency, PWM_BIT_RESOLUTION)) {
        return false;
      }
    }
  } else { // reverse
    if (m_decayMode == drv8833DecayFast) {
      drv_ledc_detach((uint8_t)m_in1Pin);
      digitalWrite(m_in1Pin, LOW);
      if (!drv_ledc_attach((uint8_t)m_in2Pin, m_channel, m_frequency, PWM_BIT_RESOLUTION)) {
        return false;
      }
    } else {
      drv_ledc_detach((uint8_t)m_in2Pin);
      digitalWrite(m_in2Pin, HIGH);
      if (!drv_ledc_attach((uint8_t)m_in1Pin, m_channel, m_frequency, PWM_BIT_RESOLUTION)) {
        return false;
      }
    }
  }

  drv_ledc_write(m_channel, dutyCycle);
  return true;
}

bool Cdrv8833::stop() {
  if (m_in1Pin == -1 || m_in2Pin == -1) {
    return false;
  }

  drv_ledc_detach((uint8_t)m_in1Pin);
  drv_ledc_detach((uint8_t)m_in2Pin);
  digitalWrite(m_in1Pin, LOW);
  digitalWrite(m_in2Pin, LOW);
  m_power = 0;
  return true;
}

bool Cdrv8833::brake() {
  if (m_in1Pin == -1 || m_in2Pin == -1) {
    return false;
  }

  drv_ledc_detach((uint8_t)m_in1Pin);
  drv_ledc_detach((uint8_t)m_in2Pin);
  digitalWrite(m_in1Pin, HIGH);
  digitalWrite(m_in2Pin, HIGH);
  m_power = 0;
  return true;
}

void Cdrv8833::setDecayMode(drv8833DecayMode decayMode) {
  stop();
  m_decayMode = decayMode;
}

void Cdrv8833::setFrequency(uint32_t frequency) {
  stop();
  m_frequency = frequency;
#if !CDRV8833_USE_NEW_LEDC_API
  ledcChangeFrequency(m_channel, m_frequency, PWM_BIT_RESOLUTION);
#endif
}

void Cdrv8833::swapDirection(bool swapDirection) {
  stop();
  m_swapDirection = swapDirection;
}
