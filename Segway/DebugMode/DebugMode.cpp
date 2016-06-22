#include "DebugMode.h"
// #include "../UART/bitmacros.h"

const char* DebugMode::MYAVR32_GPIO_ADDRESS = (char*) 0xFFFF1000;
const unsigned int DebugMode::MYAVR32_GPIO_SIZE_PORT = 0x0100;
const unsigned int DebugMode::MYAVR32_GPIO_OFFSET_GPER = 0x00;
const unsigned int DebugMode::MYAVR32_GPIO_OFFSET_ODER = 0x40;
const unsigned int DebugMode::MYAVR32_GPIO_OFFSET_OVR = 0x50;


DebugMode::DebugMode() {
    if (!rs232UART.init(&Configuration::rs232UARTConfig)) {
        //AVR32_GPIO.port[0].ovrc = ( 1 << 24 );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 24);
        //AVR32_GPIO.port[0].ovrc = ( 1 << 25 );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 25);
        while (true);
    }
    rs232UART.enableInPinSelector(true);
    if (!bluetoothUART.init(&Configuration::bluetoothUARTConfig)) {
        //AVR32_GPIO.port[0].ovrc = ( 1 << 24 );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 24);
        //AVR32_GPIO.port[0].ovrc = ( 1 << 25 );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), 25);
        while (true);
    }
    bluetoothUART.enableInPinSelector(true);
}


void DebugMode::setMotorSpeed(Motor* motor, long speed) {
    if (speed < 0) {
        motor->setDirection(false);
        motor->setSpeed(-speed);
    } else {
        motor->setDirection(true);
        motor->setSpeed(speed);
    }
}


void DebugMode::main() {
    Motor::initEnablePin();
    leftMotor.init(&Configuration::leftMotorConfig);
    rightMotor.init(&Configuration::rightMotorConfig);
    leftMotor.setDirection(true);
    rightMotor.setDirection(true);
    // Debug
    //AVR32_GPIO.port[0].gpers = ( 1 << 24 ) | ( 1 << 25 );
    //AVR32_GPIO.port[0].oders = ( 1 << 24 ) | ( 1 << 25 );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), 24);
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), 24);
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), 25);
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + 0 * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), 25);
    bool
    leftDirection = true,
    rightDirection = true;
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftMotor.setEnabled(true);
    rightMotor.setEnabled(true);
    rs232UART.sendString("lDir: ");
    rs232UART.sendString(leftDirection ? "fw" : "bw");
    rs232UART.sendString("  lSp: ");
    rs232UART.sendNumber((unsigned long) 123);
    //rs232UART.sendNumber(( unsigned long ) leftMotor.getSpeed());
    rs232UART.sendString("\nrDir: ");
    rs232UART.sendString(rightDirection ? "fw" : "bw");
    rs232UART.sendString("  rSp: ");
    rs232UART.sendNumber((unsigned long) 123);
    //rs232UART.sendNumber(( unsigned long ) rightMotor.getSpeed());
    rs232UART.sendString("\r\n\r\n\r\n");
    while (true) {
        if (rs232UART.isDataAvailable()) {
            unsigned long input = rs232UART.getData();
            if (input == '0') {
                leftDirection = !leftDirection;
                leftMotor.setDirection(leftDirection);
            } else if (input == '1')
                leftMotor.setSpeed(leftMotor.getSpeed() + 5);
            else if (input == '2')
                leftMotor.setSpeed(leftMotor.getSpeed() - 5);
            else if (input == '3') {
                rightDirection = !rightDirection;
                rightMotor.setDirection(rightDirection);
            } else if (input == '4')
                rightMotor.setSpeed(rightMotor.getSpeed() + 5);
            else if (input == '5')
                rightMotor.setSpeed(rightMotor.getSpeed() - 5);
            rs232UART.sendString("lDir: ");
            rs232UART.sendString(leftDirection ? "fw" : "bw");
            rs232UART.sendString("  lSp: ");
            rs232UART.sendNumber((unsigned long) leftMotor.getSpeed());
            rs232UART.sendString("\nrDir: ");
            rs232UART.sendString(rightDirection ? "fw" : "bw");
            rs232UART.sendString("  rSp: ");
            rs232UART.sendNumber((unsigned long) rightMotor.getSpeed());
            rs232UART.sendString("\r\n\r\n\r\n");
        }
    }
}
