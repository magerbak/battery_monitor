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


void Battery::begin(const char* name, int pin, float r1, float r2, float adcAdjustment,
                    int avgCount, ExtHistory* history) {
    m_pName = name;
    m_history = history;

    m_adc.begin(pin, r1, r2, adcAdjustment, avgCount);
}

bool Battery::updateVoltageData() {
    bool bDone = m_adc.updateVoltage();
    if (bDone) {
        //Serial.println(getName());
        //Serial.println(m_adc.getVoltage(), 3);
    }
    return bDone;
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

