#include "ADCSensor.h"

ADCSensor::ADCSensor() {}

ADCSensor::~ADCSensor() {}

/*! \brief
    Uebergibt die Werte aus thisADCSensorConfig_ an ADCController_
*/
bool ADCSensor::init(Configuration::s_ADCSensorConfig* thisADCSensorConfig_, ADC* ADCController_) {
    ADCController_->ID = thisADCSensorConfig_->ADCChannelID;
    ADCController_->offsetValue = thisADCSensorConfig_->zeroOffset;
    ADCController_->ADCSlopeFactor = thisADCSensorConfig_->slopeFactor;
    ADCController_->useADCSlopeFactor = thisADCSensorConfig_->useSlopeFactor;
    ADCController_->useADCZeroOffset = thisADCSensorConfig_->useZeroOffset;
    adcSensor = thisADCSensorConfig_;
    controller = ADCController_;
    controller->enableInPinSelector(thisADCSensorConfig_->ADCChannelID, true);
    return 1;
}

/*! \brief
    Verwendet die ADC Methode getChannelValue, um den Wert des ADCSensors
    auszulesen.
*/
signed long ADCSensor::getIntegerValue(bool average, unsigned long numberOfValuesForAverage) {
    //Verwendet die ADC Methode
    integerValue = controller->getChannelValue(adcSensor->ADCChannelID, average, numberOfValuesForAverage);
    //Falls ein offset verwendet wird, wird er vor der Ausgabe vom Wert des ADC abgezogen
    if (adcSensor->useZeroOffset == true)
        integerValue = integerValue - adcSensor->zeroOffset;
    return integerValue;
}


void ADCSensor::setZeroOffset(bool active, signed long offset) {
    adcSensor->zeroOffset = offset;
    adcSensor->useZeroOffset = active;
}

bool ADCSensor::getZeroOffsetIsActive() {
    return adcSensor->useZeroOffset;
}

signed long ADCSensor::getZeroOffset() {
    return adcSensor->zeroOffset;
}


/*! \brief
    Verwendet die ADC Methode getChannelValue, um den Wert des ADCSensors
    auszulesen. Gibt das Ergebnis allerdings als float aus.
*/
float ADCSensor::getFloatValue(bool average, unsigned long numberOfValuesForAverage) {
    floatValue = (float) controller->getChannelValue(adcSensor->ADCChannelID, average, numberOfValuesForAverage);
    return floatValue;
}

void ADCSensor::setSlopeFactor(bool active, float factor) {
    adcSensor->useSlopeFactor = active;
    adcSensor->slopeFactor = factor;
}

bool ADCSensor::getSlopeFactorIsActive(void) {
    return adcSensor->useSlopeFactor;
}

float ADCSensor::getSlopeFactor(void) {
    return adcSensor->slopeFactor;
}