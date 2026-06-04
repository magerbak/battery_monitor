#include <memory.h>

#include <Arduino.h>
#include <Preferences.h>     // For persistent storage of segment index
#include "FS.h"
#include <LittleFS.h>

#include "ext_history.h"

#define EXT_HISTORY_MAGIC   0x48697374
#define EXT_HISTORY_VER     1

struct ExtHistoryHeader {
    uint32_t m_magic;
    uint32_t m_version;
    uint32_t m_length;
    uint32_t m_idx;
    uint32_t m_wrapped;
};


bool ExtHistory::begin(int instance, DataHistory* rtc,
                       Preferences* prefs, fs::FS* fs, const char* path)
{
    char name[16];

    m_instance = instance;
    m_prefs = prefs;
    m_path = path;
    m_fs = fs;
    m_rtcData = rtc;

    size_t l = NUM_SEGMENTS * m_rtcData->m_size;
    float* d = new float[l];
    DataHistory::begin(d, l);

    flashLoad();
    rtcLoad();

    return true;
}

void ExtHistory::reset()
{
    char name[16];

    DataHistory::reset();
    m_rtcData->reset();
    m_idx = 0;

    flashClear();
}

bool ExtHistory::addSample(float val)
{
    DataHistory::addSample(val);

    // Keep a shadow copy in RTC memory so it survives deep sleep.
    if (m_rtcData->addSample(val) == true) {
        // If RTC history is full, switch to the next segment and save history
        // to flash.
        nextSegment();
        flashSave();
    }
    return true;
}

void ExtHistory::nextSegment()
{
    m_idx = (m_idx + 1) % NUM_SEGMENTS;

    m_rtcData->reset();
}

bool ExtHistory::flashLoad()
{
    size_t n = m_size * sizeof(float);

    memset(m_data, 0, n);

    if (!m_fs) {
        return false;
    }

    // Load segments from flash.
    File file = m_fs->open(m_path);
    if (!file) {
        Serial.println("Failed to open");
        return false;
    }

    ExtHistoryHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    size_t l = file.read((uint8_t *)&hdr, sizeof(hdr));
    if (l != sizeof(hdr)) {
        Serial.println("Failed to load header");
        file.close();
        return false;
    }
    if (hdr.m_magic != EXT_HISTORY_MAGIC || hdr.m_version != EXT_HISTORY_VER) {
        Serial.println("Incompatible history format");
        file.close();
        return false;
    }
    if (hdr.m_length != n || file.size() != n + sizeof(hdr)) {
        Serial.println("Unexpected history length");
        file.close();
        return false;
    }
    m_idx = hdr.m_idx;
    if (m_idx >= NUM_SEGMENTS) {
        m_idx = 0;
        Serial.println("Invalid flash index");
        file.close();
        return false;
    }
    m_bWrapped = hdr.m_wrapped;

    l = file.read((uint8_t *)m_data, n);
    if (l != n) {
        Serial.println("Failed to load file");
    }

    file.close();

    m_offset = m_idx * m_rtcData->m_size;
    return true;
}

bool ExtHistory::flashSave()
{
    size_t n = m_size * sizeof(float);

    if (!m_fs) {
        return false;
    }

    File file = m_fs->open(m_path, "w");
    if (!file) {
        Serial.println("Failed to open file to write");
        return false;
    }

    ExtHistoryHeader hdr = {
        EXT_HISTORY_MAGIC,
        EXT_HISTORY_VER,
        n,
        m_idx,
        m_bWrapped};

    size_t l = file.write((const uint8_t *)&hdr, sizeof(hdr));
    if (l != sizeof(hdr)) {
        Serial.println("Failed to write header to file");
        file.close();
        return false;
    }
    l = file.write((const uint8_t *)m_data, n);
    if (l != n) {
        Serial.println("Failed to write to file");
        file.close();
        return false;
    }

    file.close();
    return true;
}

bool ExtHistory::flashClear()
{
    if (!m_fs) {
        return false;
    }

    // Delete all segments stored in flash
    m_fs->remove(m_path);

    return true;
}

void ExtHistory::rtcLoad()
{
    // Restore current segment from deep sleep memory to flash history.
    memcpy(&m_data[m_offset], m_rtcData->m_data, m_rtcData->m_offset * sizeof(float));
    m_offset += m_rtcData->m_offset;
}



