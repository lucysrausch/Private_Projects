#include "Segway.h"
#include <cstring>
// #include "../UART/bitmacros.h"

const char* Segway::MYAVR32_GPIO_ADDRESS = (char*) 0xFFFF1000;
const unsigned int Segway::MYAVR32_GPIO_SIZE_PORT = 0x0100;
const unsigned int Segway::MYAVR32_GPIO_OFFSET_GPER = 0x00;
const unsigned int Segway::MYAVR32_GPIO_OFFSET_ODER = 0x40;
const unsigned int Segway::MYAVR32_GPIO_OFFSET_OVR = 0x50;

const char* Segway::BLUETOOTH_NAME = "TivSeg13";
const float Segway::TIMER_DELTA_T_SECONDS = 0.01;
const float Segway::ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR = 0.02;
const float Segway::MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV = (33000 / 1023);
const float Segway::PWM_TO_DRIVESUM_FACTOR = (ALGCFG_DRIVESUM_MAX_VALUE / (PWMCLASS_MAX_PWM* DRIVESPEED_PWM_MAX_PERCENT / 100));
const float Segway::SPEEDLIMIT_START_DRIVESUM = (PWM_TO_DRIVESUM_FACTOR* PWMCLASS_MAX_PWM* ALGCFG_SPEEDLIMIT_START_PWM_PERCENT / 100);
const float Segway::SPEEDLIMIT_END_DRIVESUM = (PWM_TO_DRIVESUM_FACTOR* PWMCLASS_MAX_PWM* ALGCFG_SPEEDLIMIT_END_PWM_PERCENT / 100);


/*! \brief  Function called by the timer. It contains the control algorithm.

        The function first receives the sensor values and then calculates and sets the motor's PWM
        according to the controlling algorithm.
*/
void Segway::timerFunction() {
    // get sensor values
    bool footSwitchPressed = footSwitchSensor.getValue();
    long orientationGyrometerValue = orientationGyrometer.getIntegerValue() - orientationGyrometerReference.getIntegerValue(); // orientationGyrometer does not have to be calibrated as it has it's own reference voltage
    long orientationAccelerometerValue = orientationAccelerometer.getIntegerValue();
    long steeringPotentiometerValue = 0 - steeringPotentiometer.getIntegerValue();
    // calculate the angle rate in °/s from the sensor value (+- 1000mV = +- 100°/s)
    float realAngleRate = 0.0 - float (orientationGyrometerValue) * 100.0 / 310.0;
    // calculate the acceleration g from the sensor value (+- 800mV = +- 1g)
    float acceleration = float (orientationAccelerometerValue) / 248.0;
    // calculate the (tilt) angle of the segway.
    // sin(a) = acceleration / 1g
    // a in rad = acceleration / 1g
    // a in ° = acceleration / 1g * 360/2pi
    // Approximation for small values
    float accelerometerAngle = acceleration * 57;
    // Apply complementary filter to the measured angle and the measured angle rate to get the real angle.
    // angle = (0.98)*(angle + gyro * dt) + (0.02)*(x_acc);
    CFTiltAngle = (1 - ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR) * (CFTiltAngle + realAngleRate * TIMER_DELTA_T_SECONDS) + (ALGCFG_COMPL_FILTER_ACCELANGLE_FACTOR) * (accelerometerAngle);
    // Calculate correction angle for soft speed limit
    float speedLimitTiltAngle = limitSpeedSoft(driveSum);
    // Correction term to be added to the driveSum.
    float balanceMoment = BALANCEMOMENT_ANGLERATE_WEIGHT * realAngleRate + BALANCEMOMENT_ANGLE_WEIGHT * (CFTiltAngle + speedLimitTiltAngle);
    // Integration
    driveSum = driveSum + balanceMoment;
    // Hard Limit Speed
    if (driveSum > ALGCFG_DRIVESUM_MAX_VALUE)
        driveSum = ALGCFG_DRIVESUM_MAX_VALUE;
    else if (driveSum < -ALGCFG_DRIVESUM_MAX_VALUE)
        driveSum = -ALGCFG_DRIVESUM_MAX_VALUE;
    // Convert driveSum to PWM value
    long driveSpeedPWM = driveSum / PWM_TO_DRIVESUM_FACTOR;
    // Calculate steering PWM if not blocked.
    long steeringPWM = 0;
    if (!steeringBlocked)
        steeringPWM = limitSteeringPWM(steeringPotentiometerValue, driveSum);
    long leftSpeed = driveSpeedPWM;
    long rightSpeed = driveSpeedPWM;
    if (operationMode == _standby) {
        // If in standby mode, don't move
        driveSum = 0;
        leftSpeed = 0;
        rightSpeed = 0;
        if (footSwitchPressed) {
            // If foot switch is pressed in standby mode, take current position as zero position, set up blocking
            // of the steering for 5 seconds, do a battery check and then enter run mode.
            setCurrentOrientationAsZero();
            setCurrentSteeringPositionAsZero();
            footSwitchTimeout = 0;
            CFTiltAngle = 0;
            AVGSpeedLimitDriveSumArray = {0};
            AVGSpeedLimitDriveSumArray_index = 0;
            // prevent Steering within the first 5 seconds
            blockSteeringTimeout = 500;
            steeringBlocked = true;
            // Check battery voltage, block if voltage is too low.
            doBatteryCheck();
            leftMotor.setEnabled(true);
            rightMotor.setEnabled(true);
            operationMode = _run;
        }
    }
    if (operationMode == _run) {
        // If in run mode, control motors.
        if (footSwitchPressed) {
            // If foot switch is pressed, continue normally.
            footSwitchTimeout = 0;
            if (!steeringBlocked) {
                // If steering is not blocked, add steering PWM to motor speeds.
                leftSpeed += steeringPWM;
                rightSpeed -= steeringPWM;
            } else {
                // If steering is blocked, wait decrease timeout and enable the steering when the timeout value reaches zero.
                if (blockSteeringTimeout > 0)
                    blockSteeringTimeout--;
                else {
                    if (steeringBlocked) {
                        if (steeringPWM == 0 && steeringPotentiometerValue == 0) {
                            // Blocking time elapsed and steeringPWM is zero => activate steering
                            steeringBlocked = false;
                            blockSteeringTimeout = 0;
                        }
                    }
                }
            }
        } else {
            // If foot switch is not pressed, start timeout.
            footSwitchTimeout++;
            if (footSwitchTimeout >= ALGCFG_FOOTSWITCHTIMEOUT_CRITICAL) {
                // Foot switch released too long -> go to standby
                leftMotor.setEnabled(false);
                rightMotor.setEnabled(false);
                operationMode = _standby;
            }
        }
    }
    // Apply an offset to the motor speeds to face the problem that the wheels - when standing still - won't start
    // turning until a PWM of about 12 (of 255) is reached.
    // If the target speed is lower than 5, set it to zero. Otherwise add/subtract 3 according to the direction to
    // get closer towards 12.
    if (leftSpeed > -5 && leftSpeed < 5)
        leftSpeed = 0;
    else {
        if (leftSpeed > 0)
            leftSpeed += 3;
        else
            leftSpeed -= 3;
    }
    if (rightSpeed > -5 && rightSpeed < 5)
        rightSpeed = 0;
    else {
        if (rightSpeed > 0)
            rightSpeed += 3;
        else
            rightSpeed -= 3;
    }
    // Set the motor's speed.
    // 1.741 is a correction factor resulting from fixing a rounding error in the PWM class as the
    // other parameters of the control algorithm were adjusted to the erroneous PWM class.
    setMotorSpeed(&leftMotor, (float (leftSpeed) / 1.741));
    setMotorSpeed(&rightMotor, (float (rightSpeed) / 1.741));
    // Every 20 cycles, store debug data.
    debugDataCounter++;
    if (debugDataCounter >= 20) {
        debugDataCounter = 0;
        if (!debugData.doNotRefresh) {
            debugData.leftPWM = leftSpeed;
            debugData.rightPWM = rightSpeed;
            debugData.orientationAccelerometerValue = orientationAccelerometerValue;
            debugData.orientationGyrometerValue = orientationGyrometerValue;
            debugData.steeringPotentiometerValue = steeringPotentiometerValue;
            debugData.footSwitchPressed = footSwitchPressed;
            debugData.batteryVoltage = batteryVoltageSensor.getIntegerValue() * MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV; //in mV
            debugData.driveSum = driveSum;
            debugData.steeringPWM = steeringPWM;
            debugData.leftSpeed = leftSpeed;
            debugData.rightSpeed = rightSpeed;
            debugData.CFTiltAngle = CFTiltAngle;
            debugData.speedLimitTiltAngle = speedLimitTiltAngle;
            debugData.accelerometerAngle = accelerometerAngle;
            debugData.realAngleRate = realAngleRate;
            debugData.newDataAvailable = true;
        }
    }
}


/*! \brief  Constructor. Does minimal initialization.
*/
Segway::Segway() {
    debugData = {0};
    debugDataCounter = 0;
    CFTiltAngle = 0;
    AVGSpeedLimitDriveSumArray = {0};
    AVGSpeedLimitDriveSumArray_index = 0;
}


/*! \brief  Calculates an exponential moving average for data type long.

        An exponential moving average is calculated as follows:
        new_averaged_value = weight * new_input_value + (1-weight) * old_averaged_value

        The weight factor has to be given in permille, so 500 means that newVal is weighted 50%.

    \param  oldVal  [in] old/current average
    \param  newVal  [in] measured value
    \param  weightOfNewVal_permille [in] weight factor according to function description in permille.
    \return Exponential moving average
*/
long Segway::expMovingAverage(long oldVal, long newVal, unsigned char weightOfNewVal_permille) {
    return ((1000 - weightOfNewVal_permille) * oldVal + weightOfNewVal_permille * newVal) / 1000;
}

/*! \brief  Calculates an exponential moving average for data type float.

        An exponential moving average is calculated as follows:
        new_averaged_value = weight * new_input_value + (1-weight) * old_averaged_value

        The weight factor has to be given in permille, so 500 means that newVal is weighted 50%.

    \param  oldVal  [in] old/current average
    \param  newVal  [in] measured value
    \param  weightOfNewVal_permille [in] weight factor according to function description in permille.
    \return Exponential moving average
*/
float Segway::expMovingAverage(float oldVal, float newVal, unsigned char weightOfNewVal_permille) {
    return (float (1000 - weightOfNewVal_permille) * oldVal + weightOfNewVal_permille * newVal) / 1000.0;
}

/*! \brief  Calculates the PWM values for steering.

        The steering PWM is calculated by calculating the maximum steering PWM allowed at current speed
        and by calculating the current steering handle position.
        The maximum allowed steering PWM at no speed is ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED. It linearily decreases
        to ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM with an increasing Speed-PWM from 0 to ALGCFG_DYNAMIC_STEERING_MAX_PWM.
        If speed is increased further, the maximum allowed PWM remains constant until ALGCFG_BLOCK_STEERING_PWM is reached.
        If ALGCFG_BLOCK_STEERING_PWM is reached, the maximum allowed steering PWM is 0.

        To smooth the segway's behavior, the max allowed steering PWM ist filtered using an exponential moving average.

    \param  steeringPotentiometerValue  [in] value returned by steering potentiometer (signed, zero means no steering)
    \param  driveSum    [in] current Speed (driveSum value in timerFunction)
    \return steering PWM
*/
long Segway::limitSteeringPWM(long steeringPotentiometerValue, long driveSum) {
    // Limit steering potentiometer value to what is expected at maximum (ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE)
    if (steeringPotentiometerValue > ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE)
        steeringPotentiometerValue = ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE;
    else if (steeringPotentiometerValue < -ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE)
        steeringPotentiometerValue = -ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE;
    // Calculate the absolute value of the current drive speed.
    long absDriveSum;
    if (driveSum < 0)
        absDriveSum = -driveSum;
    else
        absDriveSum = driveSum;
    static float MAMaxSteeringPWM = 0;
    if (absDriveSum > ALGCFG_BLOCK_STEERING_PWM * PWM_TO_DRIVESUM_FACTOR) {
        // If drive speed PWM exceeds ALGCFG_BLOCK_STEERING_PWM, block steering.
        return 0;
    } else if (absDriveSum > ALGCFG_DYNAMIC_STEERING_MAX_PWM * PWM_TO_DRIVESUM_FACTOR) {
        // If drive speed PWM is lower than ALGCFG_BLOCK_STEERING_PWM but over ALGCFG_DYNAMIC_STEERING_MAX_PWM,
        // allow minimal steering.
        MAMaxSteeringPWM = expMovingAverage(MAMaxSteeringPWM, float (ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM), 20);
    } else {
        // steeringSpeed decreases from ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED to ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM with increasing PWM from 0 to ALGCFG_DYNAMIC_STEERING_MAX_PWM
        MAMaxSteeringPWM = expMovingAverage(MAMaxSteeringPWM, float (ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM + (ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED - ALGCFG_DYNAMIC_STEERING_AT_MAX_PWM) * (1 - absDriveSum / (ALGCFG_DYNAMIC_STEERING_MAX_PWM * PWM_TO_DRIVESUM_FACTOR))), 20);
    }
    // If the maximum Steering PWM exceeds ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED (should not happen ), limit it to that.
    if (MAMaxSteeringPWM > ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED)
        MAMaxSteeringPWM = ALGCFG_DYNAMIC_STEERING_AT_NO_SPEED;
    return (MAMaxSteeringPWM * steeringPotentiometerValue) / float (ALGCFG_STEERING_MAX_STEERINGPOTI_VALUE);
}

/*! \brief  Calculates the speed limit angle

        As a speed limit, the segway is intended to lean back against the driver if it runs too fast. The correction
        angle, which is returned by this function, is the angle to which the segway leans against the driver.

        The angle is zero while driveSum is lower than SPEEDLIMIT_START_DRIVESUM. Then, it increases linearly to
        ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES with an increasing driveSum to SPEEDLIMIT_END_DRIVESUM. At higher
        speed, the angle remains at ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES.

    \param  driveSum    [in] current Speed (driveSum value in timerFunction)
    \return correction angle for the speed limit
*/
float Segway::limitSpeedSoft(long driveSum) {
    // Store current driveSum in an array for averaging.
    if (AVGSpeedLimitDriveSumArray_index >= 200)
        AVGSpeedLimitDriveSumArray_index = 0;
    AVGSpeedLimitDriveSumArray[AVGSpeedLimitDriveSumArray_index] = driveSum;
    AVGSpeedLimitDriveSumArray_index++;
    // If speed is low, reset speed limit array data
    if (driveSum < 2 * PWM_TO_DRIVESUM_FACTOR && driveSum > -2 * PWM_TO_DRIVESUM_FACTOR) {
        AVGSpeedLimitDriveSumArray = {0};
        AVGSpeedLimitDriveSumArray_index = 0;
    }
    // Calculate average driveSum using the array values.
    long AVGSpeedLimitDriveSum = 0;
    for (int i = 0; i < 200; i++)
        AVGSpeedLimitDriveSum += AVGSpeedLimitDriveSumArray[i];
    AVGSpeedLimitDriveSum /= 200;
    if (AVGSpeedLimitDriveSum > SPEEDLIMIT_END_DRIVESUM) {
        // maximum leaning back
        return ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES;
    } else if (AVGSpeedLimitDriveSum > SPEEDLIMIT_START_DRIVESUM) {
        // leaning back according to the speed
        return ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES * (AVGSpeedLimitDriveSum - SPEEDLIMIT_START_DRIVESUM) / (SPEEDLIMIT_END_DRIVESUM - SPEEDLIMIT_START_DRIVESUM);
    } else if (AVGSpeedLimitDriveSum < -SPEEDLIMIT_END_DRIVESUM) {
        // maximum leaning ahead
        return -ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES;
    } else if (AVGSpeedLimitDriveSum < -SPEEDLIMIT_START_DRIVESUM) {
        // leaning ahead according to the speed
        return -ALGCFG_SPEEDLIMIT_LEAN_BACK_MAX_DEGREES * (-AVGSpeedLimitDriveSum - SPEEDLIMIT_START_DRIVESUM) / (SPEEDLIMIT_END_DRIVESUM - SPEEDLIMIT_START_DRIVESUM);
    }
    return 0;
}


/*! \brief  Sets the motor's direction and speed to a given PWM value.
    \param  motor   [in] pointer to the motor object
    \param  speed   [in] signed PWM value (-255 to 255) representing the motor speed
*/
void Segway::setMotorSpeed(Motor* motor, long speed) {
    if (speed < 0) {
        motor->setDirection(false);
        motor->setSpeed(-speed);
    } else {
        motor->setDirection(true);
        motor->setSpeed(speed);
    }
}


/*! \brief  Checks the battery voltage and blocks execution if it is lower then MIN_BATTERY_VOLTAGE.

        Blocking is done via displayError.
*/
void Segway::doBatteryCheck() {
    // Check battery voltage
    if (batteryVoltageSensor.getIntegerValue() * MULTIPLEXSENSOR_BATTERYVOLTAGE_FACTOR_MV < MIN_BATTERY_VOLTAGE) {
        // battery voltage too low
        displayError(ERROR_CODE_INIT_LOWBATTERY);
    }
}


/*! \brief  Main loop.

        Initializes helper objects, then starts the timer containing the controlling algorithm.
        Before starting the timer, it is made sure that the foot switch is not pressed.

        While running, this function sends debug data via the bluetooth interface.
        Different debug values can be enabled by uncommenting them here and in timerFunction().
*/
void Segway::main() {
    initHelpers();
    // Footswitch pressed on start -> don't continue until foot switch is released
    waitForFootSwitchReleased();
    // Calibrate steering
    setCurrentSteeringPositionAsZero();
    setCurrentOrientationAsZero();
    operationMode = _standby;
    // Disable motors and enable timer
    leftMotor.setEnabled(false);
    rightMotor.setEnabled(false);
    myTimer.setIsTimerEnabled(true);
    setStatusLED(&Configuration::greenStatusLEDConfig, true);
    while (true) {
        // Wait for new data to be available
        while (!debugData.newDataAvailable)
            asm("nop");
        // Prevent data refresh by timer
        debugData.doNotRefresh = true;
        debugData.newDataAvailable = false;
        // Format: "name: value\r\n"
        //bluetoothUART.sendString( "lPWM: " );
        //bluetoothUART.sendNumber( debugData.leftPWM );
        //bluetoothUART.sendString( "\r\nrPWM: " );
        //bluetoothUART.sendNumber( debugData.rightPWM );
        //bluetoothUART.sendString( "\r\norientAccel: " );
        //bluetoothUART.sendNumber( debugData.orientationAccelerometerValue );
        //bluetoothUART.sendString( "\r\norientGyro: " );
        //bluetoothUART.sendNumber( debugData.orientationGyrometerValue );
        //bluetoothUART.sendString( "\r\nsteerPoti: " );
        //bluetoothUART.sendNumber( debugData.steeringPotentiometerValue );
        //if( debugData.footSwitchPressed ) {
        //  bluetoothUART.sendString( "\r\nFussschalter: true" );
        //} else {
        //  bluetoothUART.sendString( "\r\nFootswi: false" );
        //}
        //
        bluetoothUART.sendString("\r\nbattVolt: ");
        bluetoothUART.sendNumber(debugData.batteryVoltage);
        bluetoothUART.sendString(" mV");
        //
        //
        //bluetoothUART.sendString( "\r\nMAOrientGyro: " );
        //bluetoothUART.sendNumber( debugData.MAOrientationGyrometer );
        //bluetoothUART.sendString( "\r\nMAAnglePlusGyro: " );
        //bluetoothUART.sendNumber( debugData.MAAnglePlusGyro );
        //bluetoothUART.sendString( "\r\nMAGyro: " );
        //bluetoothUART.sendNumber( debugData.MAGyro );
        //bluetoothUART.sendString( "\r\nMAAngle: " );
        //bluetoothUART.sendNumber( debugData.MAAngle );
        //
        bluetoothUART.sendString("\r\nDriveSum: ");
        bluetoothUART.sendNumber(debugData.driveSum);
        bluetoothUART.sendString("\r\nSteeringPWM: ");
        bluetoothUART.sendNumber(debugData.steeringPWM);
        bluetoothUART.sendString("\r\nLSpeed: ");
        bluetoothUART.sendNumber(debugData.leftSpeed);
        bluetoothUART.sendString("\r\nRSpeed: ");
        bluetoothUART.sendNumber(debugData.rightSpeed);
        //
        bluetoothUART.sendString("\r\nCFTiltAngle: ");
        bluetoothUART.sendNumber(debugData.CFTiltAngle);
        bluetoothUART.sendString("\r\nSpeedLimitTiltAngle: ");
        bluetoothUART.sendNumber(debugData.speedLimitTiltAngle);
        //bluetoothUART.sendString( "\r\naccelerometerAngle: " );
        //bluetoothUART.sendNumber( debugData.accelerometerAngle );
        //bluetoothUART.sendString( "\r\nrealAngleRate: " );
        //bluetoothUART.sendNumber( debugData.realAngleRate );
        bluetoothUART.sendString("\r\n\r\n");
        // Allow data refresh by timer
        debugData.doNotRefresh = false;
    }
    cleanUp();
}


/*! \brief  Initializes the helper objects. Blocks execution on error.
*/
void Segway::initHelpers() {
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
    // Enter command mode of bluetooth module connected to bluetoothUART
    bluetoothUART.sendString("+++\r");
    // Wait some time for command mode
    for (int i = 0; i < 500000; i++)
        asm("nop");
    // Set name
    char bluetoothCommand[50] = "ATN=";
    strcat(bluetoothCommand, BLUETOOTH_NAME);
    bluetoothUART.sendString(bluetoothCommand);
}


/*! \brief  Configures the steering potentiometer object so that the current steering potentiometer position is the new zero position.

        If there is an offset set currently, the offset has to be added to the currently measured sensor value as the sensor
        subtracted this offset directly after measuring the value. So the new offset value is the current value + the old offset value.

        If there is no offset set currently, use the current value as the new offset value.

        The current value is averaged over 10 measurements.
*/
void Segway::setCurrentSteeringPositionAsZero() {
    steeringPotentiometer.setZeroOffset(true,
                                        (steeringPotentiometer.getZeroOffsetIsActive() ?
                                         (steeringPotentiometer.getIntegerValue(true, 10) + steeringPotentiometer.getZeroOffset())
                                         : steeringPotentiometer.getIntegerValue(true, 10)));
}


/*! \brief  Configures the orientation accelerometer object so that the current steering potentiometer position is the new zero position.

        If there is an offset set currently, the offset has to be added to the currently measured sensor value as the sensor
        subtracted this offset directly after measuring the value. So the new offset value is the current value + the old offset value.

        If there is no offset set currently, use the current value as the new offset value.

        The current value is averaged over 10 measurements.
*/
void Segway::setCurrentOrientationAsZero() {
    orientationAccelerometer.setZeroOffset(true,
                                           (orientationAccelerometer.getZeroOffsetIsActive() ?
                                            (orientationAccelerometer.getIntegerValue(true, 10) + orientationAccelerometer.getZeroOffset())
                                            : orientationAccelerometer.getIntegerValue(true, 10)));
}


/*! \brief  Cleans up before class is destroyed. Disables the timer.
*/
void Segway::cleanUp() {
    myTimer.setIsTimerEnabled(false);
    myTimer.setIsTimerInterruptEnabled(false);
}


/*! \brief  Displays error code. Blocks execution.

        Green LED is off. Red LED blinks errorCode times, then stays off some time and restarts.
*/
void Segway::displayError(unsigned char errorCode) {
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
void Segway::initStatusLED(Configuration::s_StatusLED* statusLEDConfig) {
    //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].gpers, statusLEDConfig->pin );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), statusLEDConfig->pin);
    //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].oders, statusLEDConfig->pin );
    SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), statusLEDConfig->pin);
}


/*! \brief  Enables/Disables the status LEDs.
    \param  statusLEDConfig [in] Pointer to the configuration struct for the LED to initialize.
    \param  on  [in] true = LED on, false = LED off
*/
void Segway::setStatusLED(Configuration::s_StatusLED* statusLEDConfig, bool on) {
    if (on) {
        //CLEAR_BIT( AVR32_GPIO.port[statusLEDConfig->port].ovr, statusLEDConfig->pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), statusLEDConfig->pin);
    } else {
        //SET_BIT( AVR32_GPIO.port[statusLEDConfig->port].ovr, statusLEDConfig->pin );
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + statusLEDConfig->port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_OVR), statusLEDConfig->pin);
    }
}

/*! \brief  Blocks execution until the foot switch is released.
*/
void Segway::waitForFootSwitchReleased() {
    // Footswitch pressed -> don't continue until foot switch is released
    if (footSwitchSensor.getValue()) {
        setStatusLED(&Configuration::redStatusLEDConfig, true);
        setStatusLED(&Configuration::greenStatusLEDConfig, true);
        // Block execution until the foot switch sensor is released.
        while (footSwitchSensor.getValue()) {
            for (unsigned long wait = 0; wait < 1000000; wait++)
                asm("nop");
        }
        setStatusLED(&Configuration::redStatusLEDConfig, false);
        setStatusLED(&Configuration::greenStatusLEDConfig, false);
    }
}
