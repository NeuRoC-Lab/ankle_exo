#include "MessageBus.h"

class SafetyController : public ITask
{
public:
// define the required depedencies in terms of topics (effectively subscribing to them)
    SafetyController(
        Topic<EncoderPositions>& encoder,
        Topic<LoadCellForces>& loads,
        Topic<MotorReply>& leftMotor,
        Topic<MotorReply>& rightMotor)
        : m_encoder(encoder),
          m_loads(loads),
          m_leftMotor(leftMotor),
          m_rightMotor(rightMotor)
    {}

    void update(uint32_t nowUs) override
    {
        if (!m_encoder.valid() ||
            !m_loads.valid() ||
            !m_leftMotor.valid() ||
            !m_rightMotor.valid())
        {
            return;
        }

        const auto& encoder =
            m_encoder.latest();

        const auto& loads =
            m_loads.latest();

        const auto& left =
            m_leftMotor.latest();

        const auto& right =
            m_rightMotor.latest();

        // safety logic here
    }

private:
    Topic<EncoderPositions>& m_encoder;
    Topic<LoadCellForces>& m_loads;

    Topic<MotorReply>& m_leftMotor;
    Topic<MotorReply>& m_rightMotor;
};