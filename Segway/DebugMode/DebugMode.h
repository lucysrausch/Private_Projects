#ifndef DEBUGMODE_H_
#define DEBUGMODE_H_

#include "../Timer/Timer.h"
#include "../Antrieb/Motor.h"
#include "../Sensor/ADC.h"
#include "../Sensor/ADCSensor.h"
#include "../Sensor/GPIOSensor.h"
#include "../UART/UART.h"

class DebugMode {
  private:
    Motor leftMotor;
    Motor rightMotor;

    UART rs232UART;
    UART bluetoothUART;

    // Class Functions
    void initHelpers();
    void setMotorSpeed(Motor* motor, long speed);


    static const char* MYAVR32_GPIO_ADDRESS;
    static const unsigned int MYAVR32_GPIO_SIZE_PORT;
    static const unsigned int MYAVR32_GPIO_OFFSET_GPER;
    static const unsigned int MYAVR32_GPIO_OFFSET_ODER;
    static const unsigned int MYAVR32_GPIO_OFFSET_OVR;

  public:
    DebugMode();
    void main();
};


#endif /* DEBUGMODE_H_ */