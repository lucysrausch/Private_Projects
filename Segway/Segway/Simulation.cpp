#include "Simulation.h"
#include "../bitmacros.h"

const char* Simulation::MYAVR32_GPIO_ADDRESS = (char*) 0xFFFF1000;
const unsigned int Simulation::MYAVR32_GPIO_SIZE_PORT = 0x0100;
const unsigned int Simulation::MYAVR32_GPIO_OFFSET_GPER = 0x00;
const unsigned int Simulation::MYAVR32_GPIO_OFFSET_ODER = 0x40;
const unsigned int Simulation::MYAVR32_GPIO_OFFSET_OVR = 0x50;

/*! \brief  Sets the motor's direction and speed to a given PWM value.
    \param  motor   [in] pointer to the motor object
    \param  speed   [in] signed PWM value (-255 to 255) representing the motor speed
*/
void Simulation::setMotorSpeed(Motor* motor, long speed) {
    if (speed < 0) {
        motor->setDirection(false);
        motor->setSpeed(-speed);
    } else {
        motor->setDirection(true);
        motor->setSpeed(speed);
    }
}


void Simulation::main() {
    for (unsigned int i = 0; i < 100; i++)
        asm("nop");
    //BP1
    asm("nop");
    initHelpers();
    // Disable motors and enable timer
    leftMotor.setEnabled(false);
    rightMotor.setEnabled(false);
    myTimer.setIsTimerEnabled(true);
    setStatusLED(&Configuration::greenStatusLEDConfig, true);
    for (unsigned int i = 0; i < 100; i++)
        asm("nop");
    //BP2
    asm("nop");
    leftMotor.setEnabled(true);
    leftMotor.setSpeed(50);
    for (unsigned int i = 0; i < 100; i++)
        asm("nop");
    //BP3
    asm("nop");
    while (true)
        asm("nop");
    cleanUp();
}


/*! \brief  Initializes the helper objects. Blocks execution on error.
*/
void Simulation::initHelpers() {
    // Initialize status LEDs
    initStatusLED(&Configuration::redStatusLEDConfig);
    setStatusLED(&Configuration::redStatusLEDConfig, false);
    initStatusLED(&Configuration::greenStatusLEDConfig);
    setStatusLED(&Configuration::greenStatusLEDConfig, false);
    // Initialize Timer at 100Hz, will be in deactivated state.
    if (!myTimer.initTimer(100))
        displayError(ERROR_CODE_INIT_TIMER);
    // Initialize the ADC, so it can be used by the ADCSensors
    if (!myADC.init())
        displayError(ERROR_CODE_INIT_ADC);
    footSwitchSensor.init(&Configuration::footSwitchConfig);
    if (!orientationAccelerometer.init(&Configuration::orientationAccelerometerConfig, &myADC))
        displayError(ERROR_CODE_INIT_ORIENTACCEL);
    if (!orientationGyrometer.init(&Configuration::orientationGyrometerConfig, &myADC))
        displayError(ERROR_CODE_INIT_ORIENTGYRO);
    if (!orientationGyrometerReference.init(&Configuration::orientationGyrometerReferenceConfig, &myADC))
        displayError(ERROR_CODE_INIT_ORIENTGYROREFERENCE);
    if (!steeringPotentiometer.init(&Configuration::steeringPotentiometerConfig, &myADC))
        displayError(ERROR_CODE_INIT_STEERINGPOTI);
    if (!batteryVoltageSensor.init(&Configuration::batteryVoltageSensorConfig, &myADC))
        displayError(ERROR_CODE_INIT_BATTERYSENS);
    // init multiplexer pins
    unsigned char multiplexer_ports[] = {1, 1, 1};
    unsigned char multiplexer_pins[] = {2, 3, 4};
    for (unsigned char i = 0; i < 3; i++) {
        //SET_BIT( AVR32_GPIO.port[multiplexer_ports[i]].gper, multiplexer_pins[i] ); // Pin is controlled by GPIO
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + multiplexer_ports[i] * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), multiplexer_pins[i]);
        //SET_BIT( AVR32_GPIO.port[multiplexer_ports[i]].oder, multiplexer_pins[i] ); // Pin is driven
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + multiplexer_ports[i] * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), multiplexer_pins[i]);
        //CLEAR_BIT( AVR32_GPIO.port[multiplexer_ports[i]].ovr, multiplexer_pins[i] ); // set pin to zero
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + multiplexer_ports[i] * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), multiplexer_pins[i]);
    }
    //CLEAR_BIT( AVR32_GPIO.port[1].ovr, 2 );
    CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 1 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 2);
    //SET_BIT( AVR32_GPIO.port[1].ovr, 3 );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 1 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 3);
    //CLEAR_BIT( AVR32_GPIO.port[1].ovr, 4 );
    CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 1 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 4);
    Motor::initEnablePin();
    if (!leftMotor.init(&Configuration::leftMotorConfig))
        displayError(ERROR_CODE_INIT_LEFTMOTOR);
    if (!rightMotor.init(&Configuration::rightMotorConfig))
        displayError(ERROR_CODE_INIT_RIGHTMOTOR);
    if (!rs232UART.init(&Configuration::rs232UARTConfig))
        displayError(ERROR_CODE_INIT_RS232UART);
    rs232UART.enableInPinSelector(true);
    if (!bluetoothUART.init(&Configuration::bluetoothUARTConfig))
        displayError(ERROR_CODE_INIT_BLUETOOTHUART);
    bluetoothUART.enableInPinSelector(true);
}


/*! \brief  Cleans up before class is destroyed. Disables the timer.
*/
void Simulation::cleanUp() {
    myTimer.setIsTimerEnabled(false);
    myTimer.setIsTimerInterruptEnabled(false);
}


/*! \brief  Displays error code. Blocks execution.

        Green LED is off. Red LED blinks errorCode times, then stays off some time and restarts.
*/
void Simulation::displayError(unsigned char errorCode) {
    // turn off both LEDs
    setStatusLED(&Configuration::greenStatusLEDConfig, false);
    setStatusLED(&Configuration::redStatusLEDConfig, false);
    while (true) {
        for (unsigned char i = 0; i < errorCode; i++) {
            // blink error code
            setStatusLED(&Configuration::greenStatusLEDConfig, true);
            setStatusLED(&Configuration::redStatusLEDConfig, true);
            for (unsigned long wait = 0; wait < 1000000; wait++)
                asm("nop");
            setStatusLED(&Configuration::greenStatusLEDConfig, false);
            setStatusLED(&Configuration::redStatusLEDConfig, false);
            for (unsigned long wait = 0; wait < 1000000; wait++)
                asm("nop");
        }
        // blink green led only to let the user know that the error code is over
        setStatusLED(&Configuration::greenStatusLEDConfig, true);
        for (unsigned long wait = 0; wait < 1000000; wait++)
            asm("nop");
        setStatusLED(&Configuration::greenStatusLEDConfig, false);
        for (unsigned long wait = 0; wait < 1000000; wait++)
            asm("nop");
        setStatusLED(&Configuration::greenStatusLEDConfig, true);
        for (unsigned long wait = 0; wait < 1000000; wait++)
            asm("nop");
        setStatusLED(&Configuration::greenStatusLEDConfig, false);
        for (unsigned long wait = 0; wait < 1000000; wait++)
            asm("nop");
    }
}


/*! \brief  Initializes the status LEDs.
    \param  statusLEDConfig [in] Pointer to configuration struct for the LED to initialize.
*/
void Simulation::initStatusLED(Configuration::s_StatusLED* statusLEDConfig) {
    //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].gpers, statusLEDConfig->pin );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), statusLEDConfig->pin);
    //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].oders, statusLEDConfig->pin );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), statusLEDConfig->pin);
}


/*! \brief  Enables/Disables the status LEDs.
    \param  statusLEDConfig [in] Pointer to the configuration struct for the LED to initialize.
    \param  on  [in] true = LED on, false = LED off
*/
void Simulation::setStatusLED(Configuration::s_StatusLED* statusLEDConfig, bool on) {
    if (on) {
        //CLEAR_BIT( AVR32_GPIO.port[statusLEDConfig->port].ovr, statusLEDConfig->pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), statusLEDConfig->pin);
    } else {
        //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].ovr, statusLEDConfig->pin );
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), statusLEDConfig->pin);
    }
}
