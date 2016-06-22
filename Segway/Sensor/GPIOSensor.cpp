/*
	Written by Niklas Fauth at 4am. No warranty...
*/

#include "GPIOSensor.h"

GPIOSensor::GPIOSensor() {}

GPIOSensor::~GPIOSensor() {
    *PUER = 0;
    *GFER = 0;
}

void GPIOSensor::init(Configuration::s_GPIOSensorConfig* thisGPIOSensorConfig_) {
    pin = thisGPIOSensorConfig_->pin;
    PVR = (volatile int*)(GPIO_MODULE + thisGPIOSensorConfig_->port * GPIO_PORT_OFFSET + GPIO_PVR_OFFSET);
    PUER = (volatile int*)(GPIO_MODULE + thisGPIOSensorConfig_->port * GPIO_PORT_OFFSET + GPIO_PUER_OFFSET);
    GFER = (volatile int*)(GPIO_MODULE + thisGPIOSensorConfig_->port * GPIO_PORT_OFFSET + GPIO_GFER_OFFSET);
    if (thisGPIOSensorConfig_->pullupEnabled) {
      *PUER |= (1u << pin);
    }
    else {
      *PUER &= (unsigned)~(1u << (pin));
    }
    *GFER |= (1u <<pin);
}

bool GPIOSensor::getValue() {
    return *PVR & (1u << (pin));
}
