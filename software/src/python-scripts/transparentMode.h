#include <Arduino.h>

// This code will consist of a proportional-derivative (PD) control loop for enabling transparency mode

#include <Arduino.h>

class PDControl
{
public:
    PDControl(float pGain = 1.0f, float dGain = 1.0f)
        : m_pGain(pGain), m_dGain(dGain)
    {
    }

    float update(float error)
    {
        if (!m_initialized)
        {
            m_previousError = error;
            m_timer = 0;
            m_initialized = true;

            return m_pGain * error;
        }

        const float dt = static_cast<float>(m_timer) * 1.0e-6f;
        m_timer = 0;

        const float proportional = m_pGain * error;

        float derivative = 0.0f;
        if (dt > 0.0f)
        {
            derivative =
                m_dGain * (error - m_previousError) / dt;
        }

        m_previousError = error;

        return proportional + derivative;
    }

    void reset()
    {
        m_previousError = 0.0f;
        m_initialized = false;
        m_timer = 0;
    }

private:
    float m_pGain;
    float m_dGain;
    float m_previousError = 0.0f;

    elapsedMicros m_timer;
    bool m_initialized = false;
};

