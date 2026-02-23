#include <Arduino.h>

#include "battery.h"

// Percent State of Charge table for a typical 12v AGM battery
// Ref: https://lifelinebatteries.com/wp-content/uploads/2015/12/6-0101G_Lifeline_Technical_Manual.pdf
//
// Adjust the values below based on specific battery chemistry and model.
//
// TODO: May need different tables for each battery (probably need a const battery info object passed
// in to begin)
const Battery::PSoC Battery::m_psocTable[] = {
    { 100, MAX_VOLTAGE },
    { 100, 12.78 },
    {  90, 12.66 },
    {  80, 12.54 },
    {  70, 12.42 },
    {  60, 12.30 },
    {  50, 12.18 },
    {  40, 12.06 },
    {  30, 11.94 },
    {  20, 11.82 },
    {  10, 11.70 },
    {   0, 11.58 },
    {   0, 0 },
};

// We want ADC input to be less than 3.1V, which is about 20% of MAX_VOLTAGE.
// Using a voltage divider, the battery voltage will be scaled by R2 / (R1 + R2).
//
// BAT 12v--
//         |
//         R1
// ADC-----|----|
//         R2   C1
//         |    |
// GND-----------
//
// Using R1 = 200k and R2 = 47k, R2 / (R1 + R2) = 18.8%.
// C1 is a 100nF capacitor to smooth out noise.
//
// Note: The ESP32 ADC technically measures inputs in the 0-950mV range using an
// internal VRef of 1100mV. Configuring the input pin to use 11dB attenuation this
// range is expanded to 0-3100mV, although the raw input is non-linear above 2.5V.
// We rely on the use of the ESP32 calibration and curve fitting APIs to
// compensate for this, although we primarily care about accuracy in the 11-13V
// range at the battery for determining state of charge, which corresponds to
// 2-2.5V at the ADC using the current resistor values. In retrospect, using
// 220k for R1 would have been a simpler choice.
//
// See the v4.4 doc for background, however the APIs have radically changed in
// v.5.5. Fortunately, the Arduino analogReadMilliVolts() API takes care of all
// these details for us.
// https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32s3/api-reference/peripherals/adc.html
// https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/api-reference/peripherals/adc_calibration.html
//
// Adjust resistor values to measured resistance of specific resistors in your circuit.
//
// TODO. R2/(R1+R2) (and maybe ADC_CAL_ADJ_FACTOR) should be passed as a param
// to begin() instead of being hardcoded here.
//
#define DIVIDER_R1      (100 + 101)
#define DIVIDER_R2      46.3

// This adjustment factor can be used to tweak the factory calibration used
// by analogReadMillivolts() to match a trusted external voltage meter.
#define ADC_CAL_ADJ_FACTOR  (0.997)


void Battery::begin(const char* name, int pin, float* histData, size_t histLen, int avgCount) {
    m_pName = name;
    m_pin = pin;

    m_history.begin(histData, histLen, avgCount);
}

bool Battery::updateVoltageData() {
    float v = readVoltage(m_pin);
    
    bool bDone = m_history.updateData(v);
    if (bDone) {
        //Serial.println(getName());
        //Serial.println(m_history.getLatestData(), 3);
    }
    return bDone;
}

void Battery::updateVoltageHistory() {
    m_history.updateHistory();
}

float Battery::readVoltage(int pin) {
    // See https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html

    // Read ADC and convert to calibrated voltage.
    float adcVoltage = (float)analogReadMilliVolts(pin) * ADC_CAL_ADJ_FACTOR / 1000.0;
    //Serial.println(adcVoltage, 3);

    // Based on resistor divider, calculate the battery voltage.
    float batteryVoltage = adcVoltage * (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2;
    //Serial.println(batteryVoltage, 2);

    return batteryVoltage;
}

float Battery::calcPSoC(float voltage) {
    // Estimate Percent State of Charge based on battery voltage.
    float psoc = m_psocTable[0].psoc;

    for (auto i = 0; m_psocTable[i].v > 0.0; i++) {
        if (voltage > m_psocTable[i + 1].v) {
            psoc = (voltage - m_psocTable[i + 1].v) / (m_psocTable[i].v - m_psocTable[i + 1].v) *
                (m_psocTable[i].psoc - m_psocTable[i + 1].psoc) + m_psocTable[i + 1].psoc;
            return psoc;
        }
    }
    return 0.0;
}

