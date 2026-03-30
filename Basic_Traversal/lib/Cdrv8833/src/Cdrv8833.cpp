#include "Cdrv8833.h"
#include "esp32-hal-gpio.h"

Cdrv8833::Cdrv8833() {
    m_in1Pin = -1;
    m_in2Pin = -1;
    m_activePin = -1;  // add this
    m_power = 0;
    m_swapDirection = false;
    m_decayMode = drv8833DecaySlow;
}

Cdrv8833::Cdrv8833(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
	init(in1Pin, in2Pin, channel, swapDirection);
}

Cdrv8833::~Cdrv8833() {
	stop();
}

bool Cdrv8833::init(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
 
	if (channel > 15)
		return false;
	if ((m_in1Pin != -1) && (m_in2Pin != -1))
		stop();

	m_in1Pin = -1;
    m_in2Pin = -1;
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);
	m_in1Pin = in1Pin;
	m_in2Pin = in2Pin;
	m_power = 0;
	m_swapDirection = swapDirection;
	m_channel = channel;
	m_activePin = in1Pin;  // ← add this default
	m_decayMode = drv8833DecaySlow;
	//ledcSetup(channel, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
	return true;
}

bool Cdrv8833::move(int8_t power) {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	if (0 == power) {
		stop();
		return true;
	}
	if (power > 100)
		power = 100;
	if (power < -100)
		power = -100;
	m_power = power;

	if (m_swapDirection)
		power = -power;
	float value = (float)((1 << PWM_BIT_RESOLUTION) - 1) * ((float)abs(power))/100.0;
	uint32_t dutyCycle;

	if ((value - trunc(value)) < 0.5)
		dutyCycle = value;
	else
		dutyCycle = value + 1;

	if (drv8833DecaySlow == m_decayMode)
		dutyCycle = ((1 << PWM_BIT_RESOLUTION) - 1) - dutyCycle;

	if (power > 0) {
    if (drv8833DecayFast == m_decayMode) {
        // forward fast decay -- PWM on in1
        ledcDetach(m_in2Pin);
        digitalWrite(m_in2Pin, LOW);
        m_activePin = m_in1Pin;  // ← track it
        ledcAttach(m_activePin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
    }
    else {
        // forward slow decay -- PWM on in2
        ledcDetach(m_in1Pin);
        digitalWrite(m_in1Pin, HIGH);
        m_activePin = m_in2Pin;  // ← track it
        ledcAttach(m_activePin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
    }
}
else {
    if (drv8833DecayFast == m_decayMode) {
        // reverse fast decay -- PWM on in2
        ledcDetach(m_in1Pin);
        digitalWrite(m_in1Pin, LOW);
        m_activePin = m_in2Pin;  // ← track it
        ledcAttach(m_activePin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
    }
    else {
        // reverse slow decay -- PWM on in1
        ledcDetach(m_in2Pin);
        digitalWrite(m_in2Pin, HIGH);
        m_activePin = m_in1Pin;  // ← track it
        ledcAttach(m_activePin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
    }
}
	ledcWriteChannel(m_channel, dutyCycle);
	return true;
}

bool Cdrv8833::stop() {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	if (-1 == m_activePin) {  // ← add this check
        // never moved, just set pins LOW directly
        digitalWrite(m_in1Pin, LOW);
        digitalWrite(m_in2Pin, LOW);
        m_power = 0;
        return true;
    }
	ledcDetach(m_in1Pin);
	ledcDetach(m_in2Pin);
	digitalWrite(m_in1Pin, LOW);
	digitalWrite(m_in2Pin, LOW);
	m_power = 0;
	return true;
}

bool Cdrv8833::brake() {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	ledcDetach(m_in1Pin);
	ledcDetach(m_in2Pin);
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
    ledcChangeFrequency(m_activePin, frequency, PWM_BIT_RESOLUTION);  // pin not channel
}

void Cdrv8833::swapDirection(bool swapDirection) {
	stop();
	m_swapDirection = swapDirection;
}

