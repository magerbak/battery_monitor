#pragma once

#include <stdint.h>
#include <stddef.h>

#include "adc_data.h"

// We assume we're monitoring a nominal 12v battery.
// See also the m_psocTable in battery.cxx which assumes a 12V AGM battery.
// That may need to be passed in per battery.
#define MAX_VOLTAGE     15.0
#define MIN_VOLTAGE     11.0

class ExtHistory;

// A light wrapper around an ADC object for tracking the current voltage of a
// battery.
class Battery
{
public:
    Battery() = default;
    ~Battery() = default;

    void begin(const char* name, int pin, float r1, float r2, float adcAdjustment,
               int avgCount, ExtHistory* history);

    bool updateVoltageData();
    void setAdcAdjustment(float adcAdjustment) { m_adc.setAdcAdjustment(adcAdjustment); }

    const char* getName() const { return m_pName; }
    float getVoltage() const { return m_adc.getVoltage(); }
    const ExtHistory* getHistory() const { return m_history; }

    static float calcPSoC(float voltage);

private:
    struct PSoC
    {
        float psoc;
        float v;
    };

    static const PSoC m_psocTable[];

    const char* m_pName = nullptr;
    AdcData m_adc;
    ExtHistory* m_history;
};


