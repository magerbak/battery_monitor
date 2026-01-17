#include <Arduino.h>
#include "battery.h"

// Percent State of Charge table for a typical 12v AGM battery
// Ref: https://lifelinebatteries.com/wp-content/uploads/2015/12/6-0101G_Lifeline_Technical_Manual.pdf
//
// Adjust the values below based on specific battery chemistry and model.
//
// TODO: May need different tables for each battery (probably need a const battery info object passed
//in to begin)
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

// Adjust values to measured resistance of specific resistors in your circuit.
//
// TODO. May need different tables for each battery. Also R2/(R1+R2) should be passed
// as a param to begin() instead of being hardcoded here.
//
// 12v----
//        |
//        R1
// A0-----|
//        R2
//        |
// GND----
//
// We want ADC input to be less than 3.1v, which is about 20% of MAX_VOLTAGE.
// R2 / (R1 + R2) = 18.8% is close.
#define DIVIDER_R1      200.4   // Nominally 200K Ohms
#define DIVIDER_R2      46.3    // Nominally 47K Ohms


void Battery::begin(const char* name, int pin, float* histData, size_t histLen, int avgCount) {
    m_pName = name;
    m_pin = pin;

    m_history.begin(histData, histLen, avgCount);
}

bool Battery::updateVoltageData() {
    float v = readVoltage(m_pin);
    return m_history.updateData(v);
}

void Battery::updateVoltageHistory() {
    m_history.updateHistory();
}

float Battery::readVoltage(int pin) {
    // Read 12-bit ADC
    int val = analogRead(pin);
    //Serial.println(val);

    // From empirical data, ADC value is approximately 1079/volt
    float adcVoltage = (2.429 * val / 2621);
    //Serial.println(adcVoltage, 2);

    // Based on resistor divider, calculate the true voltage.
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

