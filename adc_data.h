#pragma once

#include <stdint.h>
#include <stddef.h>

#include "avg_data.h"

// Encapsulates ADC.
//
// Uses Arduino APIs for reading calibrated ADC values and incorporates an
// additional caller-provided adjustment factor. r1 and r2 can be used to
// represent the resistor values in an external voltage divider so that Vin can
// be derived from the ADC voltage. Finally, the resulting voltage is averaged
// across avgCount samples to reduce noise.
//
// Vin------
//         |
//         R1
// ADC-----|
//         R2
//         |
// GND------
//
class AdcData
{
public:
    AdcData() = default;
    ~AdcData() = default;

    void begin(int pin, float r1, float r2, float adcAdjustment, int avgCount);

    bool updateVoltage();
    float getVoltage() const { return m_avg.getAverage(); }

    void setAdcAdjustment(float adcAdjustment) {m_adcAdjustment = adcAdjustment; }

private:
    float readVoltage();

    int m_pin = 0;
    float m_adcAdjustment = 0.0;
    float m_scaleFactor = 0.0;

    AvgData<float> m_avg;
};


