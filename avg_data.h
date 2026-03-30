#pragma once

// Maintains a simple running average of numeric data values of type T.
//
template <typename T>
class AvgData
{
public:
    AvgData() = default;
    ~AvgData() = default;

    void begin(int avgCount)
    {
        m_avgCount = avgCount;

        reset();
    }

    void reset()
    {
        m_count = 0;
        m_accum = 0;

        m_avg = 0;
    }

    // Continuously calculates a data average. When the average is updated returns true.
    bool updateData(T t)
    {
        m_accum += t;

        // When we have accumulated enough samples, calculate the average and
        // start again.
        if (++m_count >= m_avgCount) {
            m_avg = m_accum / m_avgCount;

            m_count = 0;
            m_accum = 0;
            return true;
        }
        return false;
    }

    T getAverage() const { return m_avg; }

private:
    int m_avgCount = 0;
    int m_count = 0;
    T m_accum = 0;
    T m_avg = 0;
};
