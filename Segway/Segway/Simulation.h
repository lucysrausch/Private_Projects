#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "../Timer/Timer.h"
#include "../Antrieb/Motor.h"
#include "../Sensor/ADC.h"
#include "../Sensor/ADCSensor.h"
#include "../Sensor/GPIOSensor.h"
#include "../UART/UART.h"

class Simulation {
  private:

    //! Object of Timer class to initialize the timer.
    Timer myTimer;
    //! Object of ADC class as a helper for the ADCSensor objects.
    ADC myADC;
    //! Sensor object for the foot switch to detect if a person is standing on the segway.
    GPIOSensor footSwitchSensor;
    //! Sensor object for the accelerometer for balancing.
    ADCSensor orientationAccelerometer;
    //! Sensor object for the gyrometer for balancing.
    ADCSensor orientationGyrometer;
    //! Sensor object for the gyrometer's reference value.
    ADCSensor orientationGyrometerReference;
    //! Sensor object for the potentiometer measuring the steering angle.
    ADCSensor steeringPotentiometer;
    //! Sensor object for measuring battery voltage.
    ADCSensor batteryVoltageSensor;
    //! Representation of the left motor.
    Motor leftMotor;
    //! Representation of the right motor.
    Motor rightMotor;

    //! Representation of the UART connected to the RS232 interface
    UART rs232UART;
    //! Representation of the UART connected to the bluetooth interface
    UART bluetoothUART;

    // Class Functions
    void initHelpers();
    void cleanUp();

    void displayError(unsigned char errorCode);

    void initStatusLED(Configuration::s_StatusLED* statusLEDConfig);
    void setStatusLED(Configuration::s_StatusLED* statusLEDConfig, bool on);
    void setMotorSpeed(Motor* motor, long speed);


    /*
        Error Codes
    */
    /*!
    \brief Error code: battery low
    */
    static const unsigned char ERROR_CODE_INIT_LOWBATTERY = 1;
    /*!
    \brief Error code: timer initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_TIMER = 2;
    /*!
    \brief Error code: ADC initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_ADC = 3;
    /*!
    \brief Error code: orientation accelerometer initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_ORIENTACCEL = 4;
    /*!
    \brief Error code: orientation gyrometer initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_ORIENTGYRO = 5;
    /*!
    \brief Error code: orientation gyrometer reference voltage sensor initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_ORIENTGYROREFERENCE = 6;
    /*!
    \brief Error code: steering potentiometer initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_STEERINGPOTI = 7;
    /*!
    \brief Error code: multiplex sensor initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_BATTERYSENS = 8;
    /*!
    \brief Error code: left motor initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_LEFTMOTOR = 9;
    /*!
    \brief Error code: right motor initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_RIGHTMOTOR = 10;
    /*!
    \brief Error code: UART for RS232 initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_RS232UART = 11;
    /*!
    \brief Error code: UART for bluetooth initialization failed
    */
    static const unsigned char ERROR_CODE_INIT_BLUETOOTHUART = 12;


    static const char* MYAVR32_GPIO_ADDRESS;
    static const unsigned int MYAVR32_GPIO_SIZE_PORT;
    static const unsigned int MYAVR32_GPIO_OFFSET_GPER;
    static const unsigned int MYAVR32_GPIO_OFFSET_ODER;
    static const unsigned int MYAVR32_GPIO_OFFSET_OVR;

  public:
    void main();
};


#endif /* SIMULATION_H_ */