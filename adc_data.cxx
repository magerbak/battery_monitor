#include <Arduino.h>

#include "adc_data.h"

void AdcData::begin(int pin, float r1, float r2, float adcAdjustment, int avgCount) {
    m_pin = pin;

    m_adcAdjustment = adcAdjustment;
    m_scaleFactor = (r1 + r2) / r2;

    m_avg.begin(avgCount);
}

bool AdcData::updateVoltage() {
    float v = readVoltage();
    bool bDone = m_avg.updateData(v);

    return bDone;
}

float AdcData::readVoltage() {
    // See https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html

    // Read ADC and convert to calibrated voltage.
    float adcVoltage = (float)analogReadMilliVolts(m_pin) * m_adcAdjustment / 1000.0;
    //Serial.println(adcVoltage, 3);

    // Based on resistor divider, calculate the AdcData voltage.
    float AdcDataVoltage = adcVoltage * m_scaleFactor;
    //Serial.println(AdcDataVoltage, 2);

    return AdcDataVoltage;
}

