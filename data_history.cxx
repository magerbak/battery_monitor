
#include "data_history.h"

void DataHistory::begin(float* data, size_t size)
{
    m_data = data;
    m_size = size;
    m_offset = 0;

    m_bWrapped = false;
}

void DataHistory::reset()
{
    m_offset = 0;
    m_bWrapped = false;
}


bool DataHistory::addSample(float val)
{
    m_data[m_offset++] = val;

    if (m_offset == m_size) {
        m_offset = 0;
        m_bWrapped = true;
        return true;
    }
    return false;
}

// Pass data history to callback from oldest to latest.
void DataHistory::forEachData(Callback callback, void* user) const
{
    size_t logical_offset = 0;
    if (m_bWrapped) {
        (*callback)(user, &m_data[m_offset], m_size - m_offset, logical_offset);
    }
    if (m_offset > 0) {
        logical_offset += m_size - m_offset;
        (*callback)(user, &m_data[0], m_offset, logical_offset);
    }
}

