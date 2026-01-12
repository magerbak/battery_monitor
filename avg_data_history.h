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
    ~AvgDataHistory()
    {
        if (m_data) {
            delete [] m_data;
            m_data = nullptr;
        }
        m_len = 0;

        reset();
    }

    void begin(size_t length)
    {
        if (m_data){
            delete [] m_data;
        }
        m_data = new T[length];
        m_len = length;

        reset();
    }

    void reset()
    {
        m_offset = 0;
        m_bWrapped = false;

        m_accum = 0;
        m_count = 0;
    }

    void updateData(T t)
    {
        m_accum += t;
        m_count++;
    }

    void updateHistory() 
    {
        m_data[m_offset] = m_accum / (m_count == 0 ? 1 : m_count);
        m_accum = 0;
        m_count = 0;

        if (++m_offset == m_len) {
            m_bWrapped = true;
            m_offset = 0;
        }
    }

    T getLastData() const
    {
        if (m_offset > 0) {
            return m_data[m_offset - 1];
        }

        if (m_bWrapped) {
            return m_data[m_len - 1];
        } else {
            // No valid data. This could assert or throw an exception.
            return 0;
        }
    }

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
    T m_accum = 0;
};
