#include "Configuration.h"

// MISC
unsigned long
Configuration::Oscillator_Freq = 0,
               Configuration::CPUCLK = 0,
                              Configuration::PBACLK = 0,
                                             Configuration::PWMCLK = 0,
                                                            Configuration::ADCCLK = 0;


// TIMER
unsigned char
Configuration::Timer_Channel = 0,
               Configuration::Timer_Clock_Connection = 0;


// PWM
Configuration::s_PWMConfig
Configuration::leftPWMConfig = {},
               Configuration::rightPWMConfig = {};


// MOTOR
Configuration::s_MotorConfig
Configuration::leftMotorConfig = {},
               Configuration::rightMotorConfig = {};

unsigned char
Configuration::Motor_enabledPinPort = 0;
unsigned long
Configuration::Motor_enabledPinPin = 0;
bool
Configuration::Motor_enabledPinEnabledValue = 0; // 1: high on enabled, 0: low on enabled


// GPIOSENSOR
Configuration::s_GPIOSensorConfig
Configuration::footSwitchConfig = {};


// ADC(SENSOR)
unsigned long
Configuration::ADC_Internal_Clock = 0;

Configuration::s_gpioMultiplexData
Configuration::ADC_gpioMultiplexData[ADC_NUM_CONFIGURED_CHANNELS] = {



};

Configuration::s_ADCSensorConfig
Configuration::orientationAccelerometerConfig = {},
               Configuration::orientationGyrometerConfig = {},
                              Configuration::orientationGyrometerReferenceConfig = {},
                                             Configuration::steeringPotentiometerConfig = {},
                                                            Configuration::batteryVoltageSensorConfig = {};

// UART
Configuration::s_UARTConfig
Configuration::rs232UARTConfig = {},
               Configuration::bluetoothUARTConfig = {};

// Status LEDs
Configuration::s_StatusLED
Configuration::redStatusLEDConfig,
              Configuration::greenStatusLEDConfig;




/*! \brief  Initializes all configuration variables.
*/
void Configuration::init() {
    // MISC
    Oscillator_Freq = 16000000;     //Oscillator frequency in Hz
    CPUCLK = Oscillator_Freq;
    PBACLK = Oscillator_Freq;
    PWMCLK = Oscillator_Freq;
    ADCCLK = Oscillator_Freq;
    init_Timer();
    init_PWM();
    init_Motor();
    init_Sensors();
    init_UART();
    init_StatusLEDs();
}

/*! \brief  Initializes timer variables.

        Timer_Channel: selects which of the controller's timer channel is used.
        Timer_Clock_Connection: selects source clock and prescaler.
*/
void Configuration::init_Timer() {
    Timer_Channel = 0;
    Timer_Clock_Connection = 0x02;      // AVR32_TC_TIMER_CLOCK3 | OSC0: 16MHz, Prescaler: 8, Timer: 16 Bit => 30 Hz to 2 MHz
}

/*! \brief  Initializes PWM variables for left and right motor.

        channelID: the microcontroller's PWM channel.
        maxPWMRatio: maximum PWM radio allowed (0: always off, 255: always on).
        frequency: PWM frequency.
        GPIO_port: GPIO port, at which the PWM signal is outputted.
        GPIO_pin: GPIO pin, at which the PWM signal is outputted.
        GPIO_multiplexRegisterValue: value for the GPIO port's multiplexer to select PWM as signal source.
*/
void Configuration::init_PWM() {
    leftPWMConfig.channelID = 0;
    leftPWMConfig.maxPWMRatio = 100;//204;
    leftPWMConfig.frequency = 18000;
    leftPWMConfig.GPIO_port = 0; //PA11
    leftPWMConfig.GPIO_pin = 11;
    leftPWMConfig.GPIO_multiplexRegisterValue = 2; //0b10 -> C
    rightPWMConfig.channelID = 1;
    rightPWMConfig.maxPWMRatio = 100;//204;
    rightPWMConfig.frequency = 18000;
    rightPWMConfig.GPIO_port = 0; //PA12
    rightPWMConfig.GPIO_pin = 12;
    rightPWMConfig.GPIO_multiplexRegisterValue = 2; //0b10 -> C
}

/*! \brief  Initializes Motor variables for left and right motor.

        Motor_enabledPinEnabledValue: 1: high output = enabled, 0: low output = enabled.
        Motor_enabledPinPort: GPIO port, at which the enabled signal is outputted.
        Motor_enabledPinPin: GPIO pin, at which the enabled signal is outputted.

        directionPinForwardValue: 1: high output = forward, 0: low output = forward.
        directionPinPort: GPIO port, at which the direction signal is outputted.
        directionPinPin: GPIO pin, at which the direction signal is outputted.
        PWMConfig: Pointer to the motor's PWM config.
*/
void Configuration::init_Motor() {
    Motor_enabledPinEnabledValue = 1;
    Motor_enabledPinPort = 0; //PA16
    Motor_enabledPinPin = 16;
    rightMotorConfig.directionPinForwardValue = 1;
    rightMotorConfig.directionPinPort = 0; //PA14
    rightMotorConfig.directionPinPin = 14;
    rightMotorConfig.PWMConfig = &leftPWMConfig;
    leftMotorConfig.directionPinForwardValue = 1;
    leftMotorConfig.directionPinPort = 0; //PA15
    leftMotorConfig.directionPinPin = 15;
    leftMotorConfig.PWMConfig = &rightPWMConfig;
}

/*! \brief  Initializes sensor variables.

        ADC_Internal_Clock: Target value for ADC internal clock (Clock used for the AD conversion).
                            For 10 bit conversion this clock must not exceed 5 MHz.
                            The prescaler is automatically chosen by the ADC class to set this clock
                            to the closest value available, which is lower than ADC_Internal_Clock.
*/
void Configuration::init_Sensors() {
    // GPIO-Sensors
    init_Sensor_FootSwitch();
    // ADC-Sensors
    ADC_Internal_Clock = 5000000;
    init_Sensor_OrientationAccelerometer();
    init_Sensor_OrientationGyrometer();
    init_Sensor_OrientationGyrometerReference();
    init_Sensor_SteeringPotentiometer();
    init_Sensor_BatteryVoltage();
}

/*! \brief  Initializes footswitch sensor variables.

        port: GPIO port, at which the foot switch signal is measured.
        pin: GPIO pin, at which the foot switch signal is measured.
        pullupEnabled: Enable GPIO pull up resistor..
*/
void Configuration::init_Sensor_FootSwitch() {
    footSwitchConfig.port = 1; //PB06
    footSwitchConfig.pin = 6;
    footSwitchConfig.pullupEnabled = true;
}

/*! \brief  Initializes orientationAccelerometer-related configuration variables.

        ADC_gpioMultiplexData[x]: configure x-th channel of ADC system.

        configured: is configured.
        port: GPIO port, at which the signal is measured.
        pin: GPIO pin, at which the signal is measured.
        multiplexRegisterValue: value for the GPIO port's multiplexer to select the ADC as destination.

        ADCChannelID: select ADC channel id.
        useSlopeFactor: enable multiplication of the measured value by slopeFactor.
        slopeFactor: factor to multiply the measured value with.
        useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
        zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationAccelerometer() {
    ADC_gpioMultiplexData[3].configured = true;
    ADC_gpioMultiplexData[3].port = 0; //PA06
    ADC_gpioMultiplexData[3].pin = 6;
    ADC_gpioMultiplexData[3].multiplexRegisterValue = 1; //0b01 -> B
    orientationAccelerometerConfig.ADCChannelID = 3;
    orientationAccelerometerConfig.useSlopeFactor = false;
    orientationAccelerometerConfig.slopeFactor = 0;
    orientationAccelerometerConfig.useZeroOffset = false;
    orientationAccelerometerConfig.zeroOffset = 0;
}

/*! \brief  Initializes orientationGyrometer-related configuration variables.

        ADC_gpioMultiplexData[x]: configure x-th channel of ADC system.

        configured: is configured.
        port: GPIO port, at which the signal is measured.
        pin: GPIO pin, at which the signal is measured.
        multiplexRegisterValue: value for the GPIO port's multiplexer to select the ADC as destination.

        ADCChannelID: select ADC channel id.
        useSlopeFactor: enable multiplication of the measured value by slopeFactor.
        slopeFactor: factor to multiply the measured value with.
        useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
        zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationGyrometer() {
    ADC_gpioMultiplexData[1].configured = true;
    ADC_gpioMultiplexData[1].port = 0;
    ADC_gpioMultiplexData[1].pin = 4;
    ADC_gpioMultiplexData[1].multiplexRegisterValue = 0; //0b00 -> A
    orientationGyrometerConfig.ADCChannelID = 1;
    orientationGyrometerConfig.useSlopeFactor = false;
    orientationGyrometerConfig.slopeFactor = 0;
    orientationGyrometerConfig.useZeroOffset = false;
    orientationGyrometerConfig.zeroOffset = 0;
}

/*! \brief  Initializes orientationGyrometerReference (reference voltage for the orientationGyrometer)-related configuration variables.

        ADC_gpioMultiplexData[x]: configure x-th channel of ADC system.

        configured: is configured.
        port: GPIO port, at which the signal is measured.
        pin: GPIO pin, at which the signal is measured.
        multiplexRegisterValue: value for the GPIO port's multiplexer to select the ADC as destination.

        ADCChannelID: select ADC channel id.
        useSlopeFactor: enable multiplication of the measured value by slopeFactor.
        slopeFactor: factor to multiply the measured value with.
        useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
        zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_OrientationGyrometerReference() {
    ADC_gpioMultiplexData[2].configured = true;
    ADC_gpioMultiplexData[2].port = 0;
    ADC_gpioMultiplexData[2].pin = 5;
    ADC_gpioMultiplexData[2].multiplexRegisterValue = 1; //0b01 -> B
    orientationGyrometerReferenceConfig.ADCChannelID = 2;
    orientationGyrometerReferenceConfig.useSlopeFactor = false;
    orientationGyrometerReferenceConfig.slopeFactor = 0;
    orientationGyrometerReferenceConfig.useZeroOffset = false;
    orientationGyrometerReferenceConfig.zeroOffset = 0;
}

/*! \brief  Initializes steeringPotentiometer-related configuration variables.

        ADC_gpioMultiplexData[x]: configure x-th channel of ADC system.

        configured: is configured.
        port: GPIO port, at which the signal is measured.
        pin: GPIO pin, at which the signal is measured.
        multiplexRegisterValue: value for the GPIO port's multiplexer to select the ADC as destination.

        ADCChannelID: select ADC channel id.
        useSlopeFactor: enable multiplication of the measured value by slopeFactor.
        slopeFactor: factor to multiply the measured value with.
        useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
        zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_SteeringPotentiometer() {
    ADC_gpioMultiplexData[6].configured = true;
    ADC_gpioMultiplexData[6].port = 0;
    ADC_gpioMultiplexData[6].pin = 30;
    ADC_gpioMultiplexData[6].multiplexRegisterValue = 0; //0b00 -> A
    steeringPotentiometerConfig.ADCChannelID = 6;
    steeringPotentiometerConfig.useSlopeFactor = false;
    steeringPotentiometerConfig.slopeFactor = 0;
    steeringPotentiometerConfig.useZeroOffset = false;
    steeringPotentiometerConfig.zeroOffset = 0;
}

/*! \brief  Initializes batteryVoltageSensor-related configuration variables.

        ADC_gpioMultiplexData[x]: configure x-th channel of ADC system.

        configured: is configured.
        port: GPIO port, at which the signal is measured.
        pin: GPIO pin, at which the signal is measured.
        multiplexRegisterValue: value for the GPIO port's multiplexer to select the ADC as destination.

        ADCChannelID: select ADC channel id.
        useSlopeFactor: enable multiplication of the measured value by slopeFactor.
        slopeFactor: factor to multiply the measured value with.
        useZeroOffset: enable the subtraction of zeroOffset from the measured value before slopeFactor is applied.
        zeroOffset: offset to subtract from the measured value.
*/
void Configuration::init_Sensor_BatteryVoltage() {
    ADC_gpioMultiplexData[7].configured = true;
    ADC_gpioMultiplexData[7].port = 0;
    ADC_gpioMultiplexData[7].pin = 31;
    ADC_gpioMultiplexData[7].multiplexRegisterValue = 0; //0b00 -> A
    batteryVoltageSensorConfig.ADCChannelID = 7;
    batteryVoltageSensorConfig.useSlopeFactor = false;
    batteryVoltageSensorConfig.slopeFactor = 0;
    batteryVoltageSensorConfig.useZeroOffset = false;
    batteryVoltageSensorConfig.zeroOffset = 0;
}


/*! \brief  Initializes UART configuration variables.

        baudRate: baud rate of the UART communication
        channelmode: set normal channel mode.
        charlength: user 8 bit per character.
        paritytype: do not use parity checking.
        stopbits: one stop bit.
        usart: select USART-module to use
        RX_GPIO_port: receiving GPIO port
        RX_GPIO_pin: receiving GPIO pin
        RX_GPIO_multiplexRegisterValue: value for the receiving GPIO port's multiplexer to select USART as destination.
        TX_GPIO_port: sending GPIO port
        TX_GPIO_pin: sending GPIO pin
        TX_GPIO_multiplexRegisterValue: value for the sending GPIO port's multiplexer to select USART as destination.
*/
void Configuration::init_UART() {
    bluetoothUARTConfig.baudRate = 19200;
    bluetoothUARTConfig.channelmode = 0; // normal channel mode
    bluetoothUARTConfig.charlength = 8; // 8 bit word length
    bluetoothUARTConfig.paritytype = 0x04; // AVR32_USART_PAR_NONE | no parity bits
    bluetoothUARTConfig.stopbits = 0; // 1 stop bit
    bluetoothUARTConfig.usart_address = (char*) 0xFFFF1400;
    bluetoothUARTConfig.RX_GPIO_port = 1;
    bluetoothUARTConfig.RX_GPIO_pin = 10;
    bluetoothUARTConfig.RX_GPIO_multiplexRegisterValue = 2; //0b10 -> C
    bluetoothUARTConfig.TX_GPIO_port = 1;
    bluetoothUARTConfig.TX_GPIO_pin = 11;
    bluetoothUARTConfig.TX_GPIO_multiplexRegisterValue = 2; //0b10 -> C
    rs232UARTConfig.baudRate = 38400;
    rs232UARTConfig.channelmode = 0; // normal channel mode
    rs232UARTConfig.charlength = 8; // 8 bit word length
    rs232UARTConfig.paritytype = 0x04; // AVR32_USART_PAR_NONE | no parity bits
    rs232UARTConfig.stopbits = 0; // 1 stop bit
    rs232UARTConfig.usart_address = (char*) 0xFFFF1C00;
    rs232UARTConfig.RX_GPIO_port = 0;
    rs232UARTConfig.RX_GPIO_pin = 26;
    rs232UARTConfig.RX_GPIO_multiplexRegisterValue = 1; //0b01 -> B
    rs232UARTConfig.TX_GPIO_port = 0;
    rs232UARTConfig.TX_GPIO_pin = 27;
    rs232UARTConfig.TX_GPIO_multiplexRegisterValue = 1; //0b10 -> B
}

/*! \brief  Initializes status LED configuration variables.

        port: GPIO port
        pin: GPIO pin
*/
void Configuration::init_StatusLEDs() {
    // Status LEDs
    redStatusLEDConfig.port = 1;
    redStatusLEDConfig.pin = 0;
    greenStatusLEDConfig.port = 1;
    greenStatusLEDConfig.pin = 1;
}
