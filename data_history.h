#pragma once

#include <cstddef>

struct DataHistory {
    typedef void (*Callback)(void* user, const float* data, size_t len, size_t offset);

    DataHistory() = default;
    virtual ~DataHistory() = default;

    virtual void begin(float* data, size_t size);
    virtual void reset();
    virtual bool addSample(float val);

    virtual void forEachData(Callback callback, void* user) const;

    bool m_bWrapped = false;

    float* m_data = nullptr;
    size_t m_size = 0;
    size_t m_offset = 0;
};
