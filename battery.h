#pragma once

#include <stdint.h>
#include <stddef.h>

#include "avg_data_history.h"

// We assume we're monitoring a nominal 12v battery. 
// See also the percent state of charge table and resistor divider defines in battery.cxx 
// that may need to be customized.
#define MAX_VOLTAGE     15.0
#define MIN_VOLTAGE     11.0

class Battery
{
public:
    Battery() = default;
    ~Battery() = default;

    void begin(const char* name, int pin, float r1, float r2, float adcAdjustment,
               float* histData, size_t histLen, int avgCount);

    bool updateVoltageData();
    void updateVoltageHistory();

    const char* getName() const { return m_pName; }
    float getVoltage() const { return m_history.getLatestData(); }
    const AvgDataHistory<float>* getHistory() const { return &m_history; }

    void setAdcAdjustment(float adcAdjustment) {m_adcAdjustment = adcAdjustment; }

    static float calcPSoC(float voltage);

private:
    struct PSoC
    {
        float psoc;
        float v;
    };

    float readVoltage();

    static const PSoC m_psocTable[];

    const char* m_pName = nullptr;
    int m_pin = 0;
    float m_adcAdjustment = 0.0;
    float m_scaleFactor = 0.0;

    AvgDataHistory<float> m_history;
};


