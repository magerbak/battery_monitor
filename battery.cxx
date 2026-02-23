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

// This adjustment factor can be used to tweak the factory calibration used
// by analogReadMillivolts() to match a trusted external voltage meter.
#define ADC_CAL_ADJ_FACTOR  (0.997)


void Battery::begin(const char* name, int pin, float r1, float r2, float* histData, size_t histLen, int avgCount) {
    m_pName = name;
    m_pin = pin;

    m_scaleFactor = (r1 + r2) / r2;

    m_history.begin(histData, histLen, avgCount);
}

bool Battery::updateVoltageData() {
    float v = readVoltage(m_pin, m_scaleFactor);
    
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

float Battery::readVoltage(int pin, float scaleFactor) {
    // See https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html

    // Read ADC and convert to calibrated voltage.
    float adcVoltage = (float)analogReadMilliVolts(pin) * ADC_CAL_ADJ_FACTOR / 1000.0;
    //Serial.println(adcVoltage, 3);

    // Based on resistor divider, calculate the battery voltage.
    float batteryVoltage = adcVoltage * scaleFactor;
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

