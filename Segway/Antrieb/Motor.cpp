#include "Motor.h"

/*! \fn Motor::Motor
 * \brief
 *      Konstruktor
 *      Creates a new PWM object that provides the HAL.
 */
Motor::Motor() {
    pwm = new PWM();
}

/*! \fn Motor::~Motor
 * \brief
 *      Destruktor wird nicht benutzt
 */
Motor::~Motor() {}

/*! \fn Motor::initEnablePin
 * \brief
 *      Initialises the pin that is used for enabling/disabling the motor.
 */
void Motor::initEnablePin() {
    VINTP GPER = (VINTP)(GPIO_MODULE + (Configuration::Motor_enabledPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_GPER_OFFSET);
    SET_BIT(*GPER, Configuration::Motor_enabledPinPin);
    VINTP ODER = (VINTP)(GPIO_MODULE + (Configuration::Motor_enabledPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_ODER_OFFSET);
    SET_BIT(*ODER, Configuration::Motor_enabledPinPin);
    VINTP OVR = (VINTP)(GPIO_MODULE + (Configuration::Motor_enabledPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_OVR_OFFSET);
    CLEAR_BIT(*OVR, Configuration::Motor_enabledPinPin);
}

/*! \fn Motor::init
 * \param thisMotorConfig_ Initialisierungseinstellungen
 * \return true
 * \brief
 *      Wendet die gegebenen Einstellungen
 */
bool Motor::init(Configuration::s_MotorConfig* thisMotorConfig_) {
    // init PWM
    pwm->init(thisMotorConfig_->PWMConfig);
    // init register
    OVR = (VINTP)(GPIO_MODULE + (thisMotorConfig_->directionPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_OVR_OFFSET);
    directionPinForwardValue = thisMotorConfig_->directionPinForwardValue;
    directionPinPin = thisMotorConfig_->directionPinPin;
    return true;
}

/*! \fn Motor::setSpeed
 * \param Der zu setzende Wert
 * \return Die PWM Antwort
 * \brief
 *      Leitet den Aufruf an den PWM weiter
 */
bool Motor::setSpeed(unsigned char ratioOn) {
    // forward call to PWM
    return pwm->setChannelPWMRatio(ratioOn, true);
}

/*! \fn Motor::getSpeed
 * \param Der zu setzende Wert
 * \return Die PWM Antwort
 * \brief
 *      Leitet den Aufruf an den PWM weiter
 */
unsigned char Motor::getSpeed() {
    // forward call to PWM
    return pwm->getChannelPWMRatio();
}

/*! \fn Motor::setDirection
 * \param forward Forwärts
 * \brief
 *      Legt die Richtung fest
 */
void Motor::setDirection(bool forward) {
    // set direction pin value
    if (forward == directionPinForwardValue) SET_BIT(*OVR, directionPinPin);
    else CLEAR_BIT(*OVR, directionPinPin);
}

/*! \fn Motor::setEnabled
 * \param enabled
 * \brief
 *      Schaltet den Motor frei / sperrt den Motor.
 */
void Motor::setEnabled(bool enabled) {
    // set enable value
    VINTP OVR = (VINTP)(GPIO_MODULE + (Configuration::Motor_enabledPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_OVR_OFFSET);
    if (enabled) SET_BIT(*OVR, Configuration::Motor_enabledPinPin);
    else CLEAR_BIT(*OVR, Configuration::Motor_enabledPinPin);
}

/*! \fn Motor::getIsEnabled
 * \return Ob der Motor freigegeben ist
 */
bool Motor::getIsEnabled() {
    // read enable value
    VINTP PVR = (VINTP)(GPIO_MODULE + (Configuration::Motor_enabledPinPort ? GPIO_PORT_OFFSET : 0) + GPIO_OVR_OFFSET);
    return BIT_IS_SET(*PVR, Configuration::Motor_enabledPinPin);
}