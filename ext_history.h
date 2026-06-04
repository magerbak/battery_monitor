#pragma once

#include <memory>

#include <FS.h>
#include <LittleFS.h>

#include "data_history.h"

class Preferences;

class ExtHistory : public DataHistory
{
public:
    ExtHistory() = default;
    ~ExtHistory() = default;

    // MA! FIXME: This should be passed in and stored as a member variable
    static const unsigned int NUM_SEGMENTS = 24 * 4;

    void reset() override;

    bool begin(int instance, DataHistory* rtc, Preferences* prefs, fs::FS* fs, const char* path);
    bool addSample(float val) override;

private:
    void nextSegment();
    bool flashLoad();
    bool flashSave();
    bool flashClear();
    void rtcLoad();

    int m_instance = 0;
    Preferences* m_prefs = nullptr;
    const char* m_path = nullptr;
    fs::FS* m_fs = nullptr;

    // Current segment index being recorded.
    unsigned int m_idx = 0;

    // Reference to current segment backed by RTC memory
    DataHistory* m_rtcData = nullptr;
};



