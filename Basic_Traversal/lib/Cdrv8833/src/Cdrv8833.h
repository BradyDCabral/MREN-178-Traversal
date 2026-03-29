#ifndef DRV8833_H
#define DRV8833_H

#include <stdint.h>

// default values
#define PWM_FREQUENCY 5000      // 1 - 50000 Hz
#define PWM_BIT_RESOLUTION 8    // pwm bit resolution

enum drv8833DecayMode {
  drv8833DecaySlow = 0,
  drv8833DecayFast = 1
};

class Cdrv8833 {
public:
  Cdrv8833();
  Cdrv8833(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection = false);
  ~Cdrv8833();

  bool init(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection = false);
  bool move(int8_t power);  // use fast decay -> smooth movement
  bool stop();
  bool brake();

  void setDecayMode(drv8833DecayMode decayMode);
  void setFrequency(uint32_t frequency);
  void swapDirection(bool swapDirection);

private:
  int8_t m_in1Pin;
  int8_t m_in2Pin;
  bool m_swapDirection;
  drv8833DecayMode m_decayMode;
  uint8_t m_channel;
  int8_t m_power;
  uint32_t m_frequency;
};

#endif
