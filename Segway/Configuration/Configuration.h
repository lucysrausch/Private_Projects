

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#define ADC_NUM_CONFIGURED_CHANNELS 8

class Configuration {
  private:
    static void init_Timer();
    static void init_PWM();
    static void init_Motor();
    static void init_Sensors();
    static void init_Sensor_FootSwitch();
    static void init_Sensor_OrientationAccelerometer();
    static void init_Sensor_OrientationGyrometer();
    static void init_Sensor_OrientationGyrometerReference();
    static void init_Sensor_SteeringPotentiometer();
    static void init_Sensor_BatteryVoltage();
    static void init_UART();
    static void init_StatusLEDs();

  public:
    struct s_PWMConfig {
        unsigned char channelID;
        unsigned char maxPWMRatio; //Value between 0 and 255. 255 equals always-on
        unsigned long frequency;
        unsigned char GPIO_port;
        unsigned char GPIO_pin;
        unsigned char GPIO_multiplexRegisterValue;
    };
    struct s_MotorConfig {
        unsigned char directionPinPort;
        unsigned long directionPinPin;
        bool directionPinForwardValue; // 1: high on forward, 0: low on backward
        s_PWMConfig* PWMConfig;
    };
    struct s_GPIOSensorConfig {
        unsigned char port;
        unsigned long pin;
        bool pullupEnabled;
    };
    struct s_gpioMultiplexData {
        bool configured;
        unsigned char port;
        unsigned long pin;
        unsigned char multiplexRegisterValue;
    };
    struct s_ADCSensorConfig {
        unsigned long
        ADCChannelID;
        signed long
        zeroOffset;
        float
        slopeFactor;
        bool
        useZeroOffset,
        useSlopeFactor;
    };
    struct s_UARTConfig {
        volatile char* usart_address;
        unsigned long baudRate;
        unsigned char
        charlength,
        paritytype,
        channelmode,
        stopbits;
        unsigned char
        RX_GPIO_port,
        RX_GPIO_pin,
        RX_GPIO_multiplexRegisterValue,
        TX_GPIO_port,
        TX_GPIO_pin,
        TX_GPIO_multiplexRegisterValue;
    };
    struct s_StatusLED {
        unsigned char port;
        unsigned long pin;
    };
    // MISC
    static unsigned long
    Oscillator_Freq,
    CPUCLK,                                 //processor clock
    PBACLK,                                 //peripheral clock
    PWMCLK,                                 //PWM clock
    ADCCLK;                                 //ADC clock (Clock provided by the Power Manager to the ADC)

    // TIMER
    static unsigned char
    Timer_Channel,
    Timer_Clock_Connection;
    // PWM
    static s_PWMConfig
    leftPWMConfig,
    rightPWMConfig;
    // MOTOR
    static s_MotorConfig
    leftMotorConfig,
    rightMotorConfig;
    static unsigned char Motor_enabledPinPort;
    static unsigned long Motor_enabledPinPin;
    static bool Motor_enabledPinEnabledValue;   // 1: high on enabled, 0: low on enabled
    // GPIOSENSOR
    static s_GPIOSensorConfig
    footSwitchConfig;
    // ADC(SENSOR)
    static unsigned long ADC_Internal_Clock;    //ADC internal clock (Clock used for the AD conversion, max 5 MHz at 10 bit)
    static s_gpioMultiplexData
    ADC_gpioMultiplexData[ADC_NUM_CONFIGURED_CHANNELS];
    static s_ADCSensorConfig
    orientationAccelerometerConfig,
    orientationGyrometerConfig,
    orientationGyrometerReferenceConfig,
    steeringPotentiometerConfig,
    batteryVoltageSensorConfig;
    // UART
    static s_UARTConfig
    rs232UARTConfig,
    bluetoothUARTConfig;
    // Status LEDs
    static s_StatusLED
    redStatusLEDConfig,
    greenStatusLEDConfig;
    static void init();
};


#endif /* CONFIGURATION_H_ */

// shortcut
#define VINTP volatile int*

// adresses
#define GPIO_MODULE 0xFFFF1000
#define GPIO_GPER_OFFSET 0x00 // Enable
#define GPIO_PMR0_OFFSET 0x10 // Peripheral mux 0
#define GPIO_PMR1_OFFSET 0x20 // Peripheral mux 1
#define GPIO_ODER_OFFSET 0x40 // Output driver
#define GPIO_OVR_OFFSET 0x50 // Output value
#define GPIO_PVR_OFFSET 0x60 // Value
#define GPIO_PUER_OFFSET 0x70 // Pull-up
#define GPIO_GFER_OFFSET 0xC0 // Glitch filter
#define GPIO_PORT_OFFSET 0x0100 // Port size

#define PWM_MODULE 0xFFFF3000
#define PWM_ENA_OFFSET 0x004 // Enable
#define PWM_DIS_OFFSET 0x008 // Disable
#define PWM_SR_OFFSET 0x00C // Status
#define PWM_CMR0_OFFSET 0x200 // Channel 0
#define PWM_CHANNEL_OFFSET 0x020 // Channel size
#define PWM_CDTY0_OFFSET 0x004 // Duty cycle
#define PWM_CPRD0_OFFSET 0x008 // Period
#define PWM_CCNT0_OFFSET 0x00C // Counter
#define PWM_CUPD0_OFFSET 0x010 // Update

#define TIMER_MODULE 0xFFFF3800
#define TIMER_CCR0_OFFSET 0x00 // Control
#define TIMER_CMR0_OFFSET 0x04 // Mode
#define TIMER_CV0_OFFSET 0x10 // Counter Value
#define TIMER_RA0_OFFSET 0x14 // Register A
#define TIMER_RB0_OFFSET 0x18 // Register B
#define TIMER_RC0_OFFSET 0x1C // Register C
#define TIMER_SR0_OFFSET 0x20 // Status
#define TIMER_IER0_OFFSET 0x24 // Interrupt Enable
#define TIMER_IDR0_OFFSET 0x28 // Interrupt Disable
#define TIMER_IMR0_OFFSET 0x2C // Interrupt Mask

#define ADC_MODULE 0xFFFF3C00
#define ADC_CR_OFFSET 0x00 // Control
#define ADC_MR_OFFSET 0x04 // Mode
#define ADC_CHER_OFFSET 0x10 // Channel enable
#define ADC_CHDR_OFFSET 0x14 // Channel disable
#define ADC_CHSR_OFFSET 0x18 // Channel Status
#define ADC_LCDR_OFFSET 0x20 // Last converted data
#define ADC_SR_OFFSET 0x1C // Status

// debugging
#include <stdio.h>
#include <stdlib.h>
