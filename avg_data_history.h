#pragma once

// Stores a rolling history of data values of type T.
//
// Data is sampled every time updateData is called.
// History is advanced (with the aveage of the previously sampled data) each time
//   updateHistory is called.
//
// History data is retrieved by passing a callback to forEachData. Callback will
//   be called once or twice (after buffer has wrapped) with a range of data (oldest
//   to newest) and the logical offset of the start of the data range.
//   Final call will be when offset + len == getSize()
template <typename T>
class AvgDataHistory
{
public:
    // Passes data of length len starting from offset in the history.
    typedef void (*Callback)(void* user, const T* data, size_t len, size_t offset);

    AvgDataHistory() = default;
    ~AvgDataHistory() = default;

    void begin(T* data, size_t length, int avgCount)
    {
        m_data = data;
        m_len = length;
        m_avgCount = avgCount;

        reset();
    }

    void reset()
    {
        m_offset = 0;
        m_bWrapped = false;

        m_accum = 0;
        m_avg = 0;
        m_count = 0;
    }

    // Continuously calculates a data average. When the average is updated returns true.
    bool updateData(T t)
    {
        m_accum += t;

        // When we have accumulated enough samples, calculate the average and
        // start again.
        if (++m_count >= m_avgCount) {
            m_avg = m_accum / m_avgCount;
            m_accum = 0;
            m_count = 0;
            return true;
        }
        return false;
    }

    // Store the latest average in the circular buffer.
    void updateHistory()
    {
        m_data[m_offset] = m_avg;

        if (++m_offset == m_len) {
            Serial.println("Wrap");
            m_bWrapped = true;
            m_offset = 0;
        }
    }

    T getLatestData() const { return m_avg; }

    // Pass data history to callback from oldest to latest.
    void forEachData(Callback callback, void* user) const
    {
        size_t logical_offset = 0;
        if (m_bWrapped) {
            (*callback)(user, &m_data[m_offset], m_len - m_offset, logical_offset);
        }
        if (m_offset > 0) {
            logical_offset += m_len - m_offset;
            (*callback)(user, &m_data[0], m_offset, logical_offset);
        }
    }

    // Max length of history (fixed)
    size_t getSize() const { return m_len; }

    // Current length of history
    size_t getLength() const { return m_bWrapped ? m_len : m_offset; }

private:
    T* m_data = nullptr;
    size_t m_len = 0;

    size_t m_offset = 0;
    bool m_bWrapped = false;

    int m_count = 0;
    int m_avgCount = 0;
    T m_accum = 0;
    T m_avg = 0;
};
