
#pragma once

#include <Arduino.h>
#include <cmath>
#include <cstdint>

#include "ProtocolTypes.h"
#include "LoadCell.h"
#include "CANMotorMIT.h"

#if defined(PLATFORM_TEENSY41) && HW_VERSION_AT_MOST(1,1,0)

using LoadCellObj = LoadCell_Teensy41;
using LoadCellHandlerObj = LoadCellHandler_Teensy41;


#elif defined(PLATFORM_NORDIC) && HW_VERSION_AT_LEAST(1,1,1)
#pragma message "Enabling Arduino Nano ADC"

using LoadCellObj = LoadCell_NanoBLE;
using LoadCellHandlerObj = LoadCellHandler_NanoBLE;
#endif

#if defined(PLATFORM_TEENSY41)

/**
 * Converts an unbounded input into a bounded torque command.
 *
 * Output range: approximately [-maxTorque, +maxTorque].
 */
inline float symmetricSigmoid(float x, float maxTorque)
{
    return maxTorque * std::tanh(0.5f * x);
}

/**
 * PD controller used for transparent-mode torque control.
 */
class PDControl
{
public:
    PDControl(
        CANMotorMIT_Handler& motorHandler,
        LoadCellHandlerObj& loadCellHandler,
        float kp = 1.0f,
        float kd = 0.0f,
        float maxTorque = 0.5f,
        unsigned long updatePeriodMicros = 1000UL,
        std::uint8_t motorId = 2)
        : m_motorHandler(motorHandler),
          m_loadCellHandler(loadCellHandler),
          m_kp(kp),
          m_kd(kd),
          m_maxTorque(maxTorque),
          m_updatePeriodMicros(updatePeriodMicros),
          m_motorId(motorId)
    {
    }

    /**
     * Call this continuously from loop().
     *
     * A new controller iteration is performed only when the configured
     * update period has elapsed.
     */
	    void begin()
    	{
        m_command.type = MotorCommandType::Start;
        m_command.motorId = m_motorId;

        m_command.cmd = MotorCmd{
            0.0f,             // position
            0.0f,             // velocity
            0.0f,  // torque
            0.0f,             // kp
            0.0f              // kd
        };
        m_motorHandler.handleSerialCommand(m_command);
    }

    void update()
    {
        const unsigned long nowMicros = micros();
        const unsigned long elapsedMicros =
            nowMicros - m_lastUpdateMicros;

        if (elapsedMicros < m_updatePeriodMicros) {
            return;
        }

        m_lastUpdateMicros = nowMicros;

        if (!updateError()) {
            applyTorque(0.0f);
            return;
        }

        const float proportional = m_kp * m_currentError;

        float derivative = 0.0f;

        if (m_initialized) {
            const float dtSeconds =
                static_cast<float>(elapsedMicros) * 1.0e-6f;

            if (dtSeconds > 0.0f) {
                derivative =
                    m_kd *
                    (m_currentError - m_previousError) /
                    dtSeconds;
            }
        } else {
            m_initialized = true;
        }

        const float rawTorque = proportional + derivative;

        const float commandedTorque = std::clamp(symmetricSigmoid(rawTorque, m_maxTorque),-0.2f,0.2f);


        applyTorque(commandedTorque);

        m_previousError = m_currentError;

#if defined(PD_CONTROL_DEBUG)
        printDebug(
            proportional,
            derivative,
            rawTorque,
            commandedTorque,
            elapsedMicros
        );
#endif
    }

    void setGains(float kp, float kd)
    {
        m_kp = kp;
        m_kd = kd;
    }

    void setMaximumTorque(float maxTorque)
    {
        m_maxTorque = std::fabs(maxTorque);
    }

    void reset()
    {
        m_previousError = 0.0f;
        m_currentError = 0.0f;
        m_lastUpdateMicros = micros();
        m_initialized = false;

        applyTorque(0.0f);
    }

    float currentError() const
    {
        return m_currentError;
    }

private:
    bool updateError()
    {
        const float* forces = m_loadCellHandler.sampleAll();

        if (forces == nullptr) {
            return false;
        }

        // Confirm that these indices correspond to the correct
        // left/right load-cell channels.
        m_currentError = forces[1] - forces[0];

        return std::isfinite(m_currentError);
    }

    void applyTorque(float commandedTorque)
    {
        m_command.type = MotorCommandType::Set;
        m_command.motorId = m_motorId;

        m_command.cmd = MotorCmd{
            0.0f,             // position
            0.0f,             // velocity
            commandedTorque,  // torque
            0.0f,             // kp
            0.0f              // kd
        };

        m_motorHandler.handleSerialCommand(m_command);
    }

#if defined(PD_CONTROL_DEBUG)
    void printDebug(
        float proportional,
        float derivative,
        float rawTorque,
        float commandedTorque,
        unsigned long elapsedMicros) const
    {
        Serial.print("dt_us=");
        Serial.print(elapsedMicros);

        Serial.print(", error=");
        Serial.print(m_currentError, 6);

        Serial.print(", P=");
        Serial.print(proportional, 6);

        Serial.print(", D=");
        Serial.print(derivative, 6);

        Serial.print(", raw=");
        Serial.print(rawTorque, 6);

        Serial.print(", torque=");
        Serial.println(commandedTorque, 6);
    }
#endif

    CANMotorMIT_Handler& m_motorHandler;
    LoadCellHandlerObj& m_loadCellHandler;

    CommandPayload m_command{};

    float m_kp;
    float m_kd;
    float m_maxTorque;

    unsigned long m_updatePeriodMicros;
    unsigned long m_lastUpdateMicros{};

    std::uint8_t m_motorId;

    float m_previousError{};
    float m_currentError{};

    bool m_initialized{};
};

#endif

