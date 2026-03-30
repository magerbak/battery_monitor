#include <memory.h>

#include <Arduino.h>
#include <Preferences.h>     // For persistent storage of segment index
#include "FS.h"
#include <LittleFS.h>

#include "ext_history.h"

bool ExtHistory::begin(int instance, DataHistory* rtc,
                       Preferences* prefs, fs::FS* fs, const char* filePath)
{
    char name[16];

    m_instance = instance;
    m_prefs = prefs;
    m_filePath = filePath;
    m_fs = fs;
    m_rtcData = rtc;

    size_t l = NUM_SEGMENTS * m_rtcData->m_size;
    float* d = new float[l];
    DataHistory::begin(d, l);

    snprintf(name, sizeof(name), "histIdx%d", m_instance);
    m_idx = m_prefs->getUInt(name, 0);

    // Sanity checks
    if (m_idx >= NUM_SEGMENTS) {
        m_idx = 0;
        Serial.println("Invalid flash index");
        return false;
    }

    loadFlashSegments();
    loadRtcSegment();

    return true;
}

void ExtHistory::reset()
{
    char name[16];

    DataHistory::reset();
    m_rtcData->reset();
    m_idx = 0;

    snprintf(name, sizeof(name), "histIdx%d", m_instance);
    m_prefs->putUInt(name, m_idx);

    clearFlashSegments();
}

bool ExtHistory::addSample(float val)
{
    DataHistory::addSample(val);

    // Keep a shadow copy in RTC memory so it survives deep sleep.
    if (m_rtcData->addSample(val) == true) {
        // If RTC history is full, it's time to save to flash and then switch to
        // next segment.
        saveSegment();
        nextSegment();
    }
    return true;
}

void ExtHistory::nextSegment()
{
    char name[16];

    m_idx = (m_idx + 1) % NUM_SEGMENTS;
    Serial.print("idx ");
    Serial.println(m_idx);

    snprintf(name, sizeof(name), "histIdx%d", m_instance);
    m_prefs->putUInt(name, m_idx);

    //size_t n = m_rtcData->m_size * sizeof(float);
    //memset(&m_data[m_offset], 0, n);

    m_rtcData->reset();
}

bool ExtHistory::loadFlashSegments()
{
    char fn[64];
    size_t offset = 0;
    size_t n = m_rtcData->m_size * sizeof(float);

    memset(m_data, 0, m_size * sizeof(float));

    if (!m_fs) {
        return false;
    }

    // Load segments from flash skipping current segment.
    for (auto i = 0; i < NUM_SEGMENTS; i++, offset += m_rtcData->m_size) {
        int rc = snprintf(fn, sizeof(fn), "%s_%u", m_filePath, i);
        if (rc >= (int)sizeof(fn)) {
            return false;
        }

        // Load what we can from flash
        File file = m_fs->open(fn);
        if (!file) {
            continue;
        }
        // The RTC history is one segment, and each file must match this size.
        if (file.size() == n) {
            if (i == m_idx) {
                // If the current segment already exists in flash then we must have
                // wrapped.
                m_bWrapped = true;
            }

            Serial.println(fn);
            Serial.println(offset);
            size_t l = file.read((uint8_t *)&m_data[offset], n);
            if (l != n) {
                Serial.println("Failed to load file");
            }
        }

        file.close();
    }

    m_offset = m_idx * m_rtcData->m_size;
    return true;
}

bool ExtHistory::saveSegment()
{
    char fn[64];
    size_t offset = m_idx * m_rtcData->m_size;
    size_t n = m_rtcData->m_size * sizeof(float);

    if (!m_fs) {
        return false;
    }

    int rc = snprintf(fn, sizeof(fn), "%s_%u", m_filePath, m_idx);
    if (rc >= (int)sizeof(fn)) {
        return false;
    }
    Serial.println(fn);
    File file = m_fs->open(fn, "w");
    if (!file) {
        Serial.println("Failed to open file to write");
        return false;
    }

    Serial.println("Writing ");
    size_t l = file.write((const uint8_t*)&m_data[offset], n);
    if (l != n) {
        Serial.println("Failed to write to file");
    }
    file.close();
    return true;
}

bool ExtHistory::clearFlashSegments()
{
    char fn[64];

    if (!m_fs) {
        return false;
    }

    // Delete all segments stored in flash
    for (auto i = 0; i < NUM_SEGMENTS; i++) {
        int rc = snprintf(fn, sizeof(fn), "%s_%u", m_filePath, i);
        if (rc >= (int)sizeof(fn)) {
            return false;
        }

        m_fs->remove(fn);
    }

    return true;
}

void ExtHistory::loadRtcSegment()
{
    // Restore current segment from deep sleep memory to flash history.
    memcpy(&m_data[m_offset], m_rtcData->m_data, m_rtcData->m_offset * sizeof(float));
    m_offset += m_rtcData->m_offset;
}



