#pragma once

#include <cstdint>
#include <cmath>

#include <Arduino.h>


#include <ArduinoJson.h>

#include "ProtocolTypes.h"


inline float p_target = 10.0f;
inline float v_target = 0.0f;
inline float kp_target = 0.5f;
inline float kd_target = 0.5f;
inline float trq_target = 0.0f;

typedef struct {
    float p_min,p_max;
    float v_min,v_max;
    float kp_min,kp_max;
    float kd_min,kd_max;
    float trq_min,trq_max;

} AK60Params;

constexpr AK60Params motorParams = {
    // these are the nominal min/max values specified for the AK60 KV140 V1.1 when issuing a user command to the motor
    // These are not meant to clip the user commands ; altering these values will make the motor misbehave
    -12.5f, 12.5f,   // position (rad)
    -45.0f,  45.0f,   // velocity
      0.0f, 500.0f,   // kp
      0.0f,   5.0f,   // kd
    -15.0f,  15.0f    // torque
};


//extern const AK60Params motorSoftwareConstraints;
// These values clip the user's position, velocity and feedforward torque before sending them to the motor
//extern const AK60Params motorRunningConstraints;
// These values are compared against the motor's feedback messages and will enforce a hard stop if one/more of the velocity, position, torque exceeds the range specified here


static constexpr uint8_t exitMotorMode[8] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFD
};

static constexpr uint8_t enterMotorMode[8] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFC
};

static constexpr uint8_t setZeroPosition[8] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFE
};

static constexpr uint8_t neutralMITCommand[8] = {
    0x7F, 0xFF, 0x7F, 0xF0,
    0x00, 0x00, 0x07, 0xFF
};

//TODO determine if MotorCmd should be motor-agnostic or not i.e should it include the target motor's CAN ID?



#if defined(PLATFORM_TEENSY41)

#include <ArduinoJSON.h>
class CANMotorMIT {
/*
Generic methods shared between the two CAN implementations (i.e Teensy 4.1 and Arduino UNO R4)
One CANMotorMIT instance per CubeMars motor
*/
public:
    CANMotorMIT(
    const byte canId,
    const AK60Params* motorSettings,
    const AK60Params* motorSoftwareConstraints,
    const AK60Params* motorRunningConstraints,
    MotorCmd& cmd,
    uint32_t kPrintEvery = 20

)
    : m_canId(canId),
      m_cmd(cmd),
      m_reply{},
      m_enabled(false),
      m_motorSettings(motorSettings),
      m_motorSoftwareConstraints(motorSoftwareConstraints),
      m_motorRunningConstraints(motorRunningConstraints),
      m_printCounter(0),
      m_kPrintEvery(kPrintEvery)
{}

    MotorReply m_reply;
    bool m_enabled;
    MotorCmd& m_cmd;
    bool m_hardStopActive = false;
	const byte m_canId;
	const AK60Params* m_motorSettings {};
    const AK60Params* m_motorSoftwareConstraints {};
    const AK60Params* m_motorRunningConstraints {};

    enum class HardStopState : uint8_t
    {
        Normal,
        RecoveringFromLowerPosition,
        RecoveringFromUpperPosition
    };

    HardStopState m_hardStopState = HardStopState::Normal;


    bool resetMotor()
    {
        m_enabled = false;
        m_hardStopActive = false;

        Serial.println("Exiting MIT motor mode...");

        if (!sendMessage(exitMotorMode))
        {
            return false;
        }

        delay(500);

        Serial.println("Entering MIT motor mode...");

        if (!sendMessage(enterMotorMode))
        {
            return false;
        }

        delay(100);

        if (!sendMessage(neutralMITCommand))
        {
            return false;
        }

        m_enabled = true;
        return true;
    }


    void update()
    {
        while (readMessages(m_reply))
        {
            if(isOutsideSoftwareLimits()){
            //Serial.println("Reached software limits. Increasing damping factor to slow down the motor inertia");
            float closest = (std::abs(m_reply.position - m_motorSoftwareConstraints->p_min) <= std::abs(m_reply.position - m_motorSoftwareConstraints->p_max))
                                  ? m_motorSoftwareConstraints->p_min
                                  : m_motorSoftwareConstraints->p_max;
            m_cmd.position = closest;
            m_cmd.kp = 0.5;
            m_cmd.torque = 0.0;
            m_cmd.kd = 0.0;
            //Serial.println("Setting damping");
            }
            else{
            m_cmd.kp = 0.0;
            }
            updateHardStopState();

            if (++m_printCounter >= m_kPrintEvery)
            {
                // print_can_msg(m_reply);
                m_printCounter = 0;
            }
        }

        if (!m_enabled)
        {
            return;
        }

        uint8_t tx_buf[8];

        if (commandAllowedByHardStop())
        {
            pack_cmd(
                tx_buf,
                m_cmd.position,
                m_cmd.velocity,
                m_cmd.kp,
                m_cmd.kd,
                m_cmd.torque
            );
        }
        else
        {
            memcpy(
                tx_buf,
                neutralMITCommand,
                sizeof(tx_buf)
            );
        }

        if (!sendMessage(tx_buf))
        {
            Serial.println("MIT command send failed");
            sendMessage(neutralMITCommand);
            sendMessage(enterMotorMode);
            delay(100);
        }
    }


/*
void writeReplyToJson(JsonObject object) const
{
    object[TelemetryKey::MotorId] = m_reply.can_id;
    object[TelemetryKey::MotorPos] = m_reply.position;
    object[TelemetryKey::MotorVel] = m_reply.velocity;
    object[TelemetryKey::MotorTrq] = m_reply.torque;
    object[TelemetryKey::MotorTemp] = m_reply.temperature;
    object[TelemetryKey::MotorErr] = m_reply.error;
}
*/


bool isOutsideRunningLimits() const
{
    return
        m_reply.position < m_motorRunningConstraints->p_min ||
        m_reply.position > m_motorRunningConstraints->p_max ||

        m_reply.velocity < m_motorRunningConstraints->v_min ||
        m_reply.velocity > m_motorRunningConstraints->v_max ||

        m_reply.torque < m_motorRunningConstraints->trq_min ||
        m_reply.torque > m_motorRunningConstraints->trq_max;
}

    bool isOutsideSoftwareLimits() const
    {
        return
            m_reply.position < m_motorSoftwareConstraints->p_min ||
            m_reply.position > m_motorSoftwareConstraints->p_max ||

            m_reply.velocity < m_motorSoftwareConstraints->v_min ||
            m_reply.velocity > m_motorSoftwareConstraints->v_max ||

            m_reply.torque < m_motorSoftwareConstraints->trq_min ||
            m_reply.torque > m_motorSoftwareConstraints->trq_max;
    }


void updateHardStopState()
{
    switch (m_hardStopState)
    {
        case HardStopState::Normal:
        {
            // Running constraints are the outer hard-stop limits.
            if (m_reply.position <= m_motorRunningConstraints->p_min)
            {
                Serial.println(
                    "Lower position hard stop reached. "
                    "Only positive recovery commands are allowed."
                );

                m_hardStopState =
                    HardStopState::RecoveringFromLowerPosition;

                sendMessage(neutralMITCommand);
            }
            else if (
                m_reply.position >=
                m_motorRunningConstraints->p_max
            )
            {
                Serial.println(
                    "Upper position hard stop reached. "
                    "Only negative recovery commands are allowed."
                );

                m_hardStopState =
                    HardStopState::RecoveringFromUpperPosition;

                sendMessage(neutralMITCommand);
            }

            break;
        }

        case HardStopState::RecoveringFromLowerPosition:
        {
            /*
             * Remain in recovery mode until the motor enters the
             * normal software-constrained operating region.
             */
            if (
                m_reply.position >=
                m_motorSoftwareConstraints->p_min
            )
            {
                Serial.println(
                    "Motor recovered past lower software limit."
                );

                m_hardStopState = HardStopState::Normal;
            }

            break;
        }

        case HardStopState::RecoveringFromUpperPosition:
        {
            if (
                m_reply.position <=
                m_motorSoftwareConstraints->p_max
            )
            {
                Serial.println(
                    "Motor recovered past upper software limit."
                );

                m_hardStopState = HardStopState::Normal;
            }

            break;
        }
    }
}


/*
 * Return:
 *
 *   +1: recovery requires increasing motor position / positive torque
 *   -1: recovery requires decreasing motor position / negative torque
 *    0: no violation
 *    2: conflicting violations; no unambiguous recovery direction
 */
int8_t requiredRecoveryDirection() const
{
    bool requiresPositive = false;
    bool requiresNegative = false;

    // Position hard stop.
    if (m_reply.position < m_motorRunningConstraints->p_min)
    {
        requiresPositive = true;
    }
    else if (m_reply.position > m_motorRunningConstraints->p_max)
    {
        requiresNegative = true;
    }

    // Velocity hard stop.
    if (m_reply.velocity < m_motorRunningConstraints->v_min)
    {
        requiresPositive = true;
    }
    else if (m_reply.velocity > m_motorRunningConstraints->v_max)
    {
        requiresNegative = true;
    }

    // Torque hard stop.
    if (m_reply.torque < m_motorRunningConstraints->trq_min)
    {
        requiresPositive = true;
    }
    else if (m_reply.torque > m_motorRunningConstraints->trq_max)
    {
        requiresNegative = true;
    }

    if (requiresPositive && requiresNegative)
    {
        return 2;
    }

    if (requiresPositive)
    {
        return +1;
    }

    if (requiresNegative)
    {
        return -1;
    }

    return 0;
}

    bool commandAllowedByHardStop() const
    {
        constexpr float torqueEpsilon = 0.01f;

        const float estimatedTorque =
            estimatedCommandedTorque();

        switch (m_hardStopState)
        {
            case HardStopState::Normal:
                return true;

            case HardStopState::RecoveringFromLowerPosition:
                // Positive torque should increase position.
                return estimatedTorque > torqueEpsilon;

            case HardStopState::RecoveringFromUpperPosition:
                // Negative torque should decrease position.
                return estimatedTorque < -torqueEpsilon;
        }

        return false;
    }

    float estimatedCommandedTorque() const
    {
        const float positionContribution =
            m_cmd.kp *
            (m_cmd.position - m_reply.position);

        const float velocityContribution =
            m_cmd.kd *
            (m_cmd.velocity - m_reply.velocity);

        return positionContribution +
               velocityContribution +
               m_cmd.torque;
    }


bool commandMovesTowardValidRegion() const
{
    const int8_t requiredDirection = requiredRecoveryDirection();

    if (requiredDirection == 0)
    {
        return true;
    }

    if (requiredDirection == 2)
    {
        // Conflicting violations: remain neutral.
        return false;
    }

    const float commandedTorque = estimatedCommandedTorque();

    /*
     * Avoid accepting tiny numerical values as intentional recovery commands.
     * Adjust this threshold for your motor if necessary.
     */
    constexpr float recoveryTorqueEpsilon = 0.01f;

    if (requiredDirection > 0)
    {
        return commandedTorque > recoveryTorqueEpsilon;
    }

    return commandedTorque < -recoveryTorqueEpsilon;
}

virtual bool begin()= 0; // this function will call platform specific initialization
virtual bool sendMessage(const uint8_t data[8]) = 0;
virtual bool readMessages(MotorReply& reply)= 0; // will pop the element in the FIFO mailbox

// to prevent one from calling these functions directly from the SuperClass
protected:
    uint32_t m_printCounter;
    uint32_t m_kPrintEvery;

    void pack_cmd(
        uint8_t tx_buf[8],
        float p_in,
        float v_in,
        float kp_in,
        float kd_in,
        float trq_in
    )
    {
        uint16_t position = float_to_uint(
            constrain_float(
                p_in,
                m_motorSoftwareConstraints->p_min,
                m_motorSoftwareConstraints->p_max
            ),
            m_motorSettings->p_min,
            m_motorSettings->p_max,
            16
        );
    
        uint16_t velocity = float_to_uint(
            constrain_float(
                v_in,
                m_motorSoftwareConstraints->v_min,
                m_motorSoftwareConstraints->v_max
            ),
            m_motorSettings->v_min,
            m_motorSettings->v_max,
            12
        );
    
        uint16_t kp = float_to_uint(
            constrain_float(
                kp_in,
                m_motorSoftwareConstraints->kp_min,
                m_motorSoftwareConstraints->kp_max
            ),
            m_motorSettings->kp_min,
            m_motorSettings->kp_max,
            12
        );
    
        uint16_t kd = float_to_uint(
            constrain_float(
                kd_in,
                m_motorSoftwareConstraints->kd_min,
                m_motorSoftwareConstraints->kd_max
            ),
            m_motorSettings->kd_min,
            m_motorSettings->kd_max,
            12
        );
    
        uint16_t trq = float_to_uint(
            constrain_float(
                trq_in,
                m_motorSoftwareConstraints->trq_min,
                m_motorSoftwareConstraints->trq_max
            ),
            m_motorSettings->trq_min,
            m_motorSettings->trq_max,
            12
        );
    
        tx_buf[0] = (position >> 8) & 0xFF;
        tx_buf[1] = position & 0xFF;
    
        tx_buf[2] = (velocity >> 4) & 0xFF;
        tx_buf[3] = ((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F);
    
        tx_buf[4] = kp & 0xFF;
    
        tx_buf[5] = (kd >> 4) & 0xFF;
        tx_buf[6] = ((kd & 0x0F) << 4) | ((trq >> 8) & 0x0F);
    
        tx_buf[7] = trq & 0xFF;
    }

MotorReply unpack_reply(const uint8_t rx_buf[8])
    {
    MotorReply reply;

    reply.can_id = rx_buf[0];

    uint16_t position_raw =
        ((uint16_t)(rx_buf[1] & 0xFF) << 8) |
         (uint16_t)(rx_buf[2] & 0xFF);

    uint16_t velocity_raw =
        ((uint16_t)(rx_buf[3] & 0xFF) << 4) |
        ((uint16_t)(rx_buf[4] & 0xF0) >> 4);

    uint16_t trq_raw =
        ((uint16_t)(rx_buf[4] & 0x0F) << 8) |
         (uint16_t)(rx_buf[5] & 0xFF);

    reply.position = uint_to_float(
        position_raw,
        motorParams.p_min,
        motorParams.p_max,
        16
    );

    reply.velocity = uint_to_float(
        velocity_raw,
        motorParams.v_min,
        motorParams.v_max,
        12
    );

    reply.torque = uint_to_float(
        trq_raw,
        motorParams.trq_min,
        motorParams.trq_max,
        12
    );

    reply.temperature = rx_buf[6];
    reply.error = rx_buf[7];

    return reply;
    }

float uint_to_float(uint16_t code, float x_min, float x_max, int bits)
    {

    float span = x_max - x_min;
    float max_int = (float)(((unsigned long)1<< bits) - 1);
    return ((float)code * span / max_int) + x_min;
    }

uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
    {
    x = constrain_float(x,x_min,x_max);
    float span = x_max - x_min;
    float max_int = (float)(((unsigned long)1 << bits) -1);
    return (uint16_t)((x-x_min)* max_int / span);
    }
float constrain_float(float x, float x_min, float x_max)
    {
    if (x < x_min)
    {
        Serial.print("Note : capping value ");
        Serial.print(x);
        Serial.print(" to ");
        Serial.println(x_min);
        return x_min;
    }

    if (x > x_max)
    {
        Serial.print("Note : capping value ");
        Serial.print(x);
        Serial.print(" to ");
        Serial.println(x_max);
        return x_max;
    }

    return x;
    }

virtual ~CANMotorMIT() = default;
};

// subclasses for UNOR4 and Teensy implementations (their libraries differ)

#if defined(PLATFORM_TEENSY41)
    #include <FlexCAN_T4.h>
    //using PlatformCanBus = CanBus_Teensy41;
    inline FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> TeensyCAN;
    // Teensy 4.1 CAN1:
    // RX = pin 23
    // TX = pin 22
    class CANMotorMIT_Teensy : public CANMotorMIT {

    public:
        CANMotorMIT_Teensy(byte canId,const AK60Params* motorSettings,const AK60Params* motorSoftwareConstraints,const AK60Params* motorRunningConstraints ,MotorCmd& cmd,uint32_t kPrintEvery=20)
        : CANMotorMIT(canId, motorSettings,motorSoftwareConstraints,motorRunningConstraints,cmd,kPrintEvery)
        {}

             bool begin() override{
            // initializes the CAN controller on the Teensy 41

                Serial.println("Now initializing CAN communication");
                TeensyCAN.begin();
                TeensyCAN.setBaudRate(1000000);//TeensyCAN.setBaudRate(1000000);
                TeensyCAN.setMaxMB(16);
                TeensyCAN.enableFIFO();
                return true;
                //TeensyCAN.enableFIFOInterrupt();
        }

            virtual bool sendMessage(const uint8_t data[8]) override {
            CAN_message_t msg;
            msg.id = m_canId;
            msg.len = 8;
            msg.flags.extended = 0; // MIT mode uses standard CAN ID
            memcpy(msg.buf, data, 8);
            int rc = TeensyCAN.write(msg);
            return rc > 0;
            }

            virtual bool readMessages(MotorReply& reply) override {
                CAN_message_t rxMsg;
                while (TeensyCAN.read(rxMsg)) {
                if (rxMsg.len != 8) {
                        continue;
                }

                reply = unpack_reply(rxMsg.buf);
                return true;   // popped one valid message
                }

                return false;      // no valid message available right now
            }
        //
    };

#else
#endif

class CANMotorMIT_Handler {

public :

    CANMotorMIT_Handler(CANMotorMIT& leftMotor,CANMotorMIT* rightMotor = nullptr,uint32_t canRate=500000) // using a pointer to indicate that the right motor is not necessary for now
        : m_leftMotor(leftMotor),
          m_rightMotor(rightMotor),
		  m_canRate(canRate)
    {
    }


	bool begin()
{
    if (!m_leftMotor.begin()) {
        Serial.println(
            "Failed to initialize left motor"
        );
        return false;
    }

    if (
        m_rightMotor != nullptr &&
        !m_rightMotor->begin()
    ) {
        Serial.println(
            "Failed to initialize right motor"
        );
        return false;
    }

    if (!m_leftMotor.resetMotor()) {
        Serial.println(
            "Failed to start left motor"
        );
        return false;
    }

    if (
        m_rightMotor != nullptr &&
        !m_rightMotor->resetMotor()
    ) {
        Serial.println(
            "Failed to start right motor"
        );
        return false;
    }

    return true;
}
CANMotorMIT* findMotor(uint8_t motorId)
{
    if (motorId == m_leftMotor.m_canId) {
        return &m_leftMotor;
    }

    if (
        m_rightMotor != nullptr &&
        motorId == m_rightMotor->m_canId
    ) {
        return m_rightMotor;
    }

    return nullptr;
}

bool handleSerialCommand(
    const CommandPayload& command
)
{
    CANMotorMIT* motor =
        findMotor(command.motorId);

    if (motor == nullptr) {
        Serial.println("Unknown motor ID");
        return false;
    }

    switch (command.type) {
    case MotorCommandType::Stop:
    {
        const bool success =
            motor->sendMessage(exitMotorMode);

        if (success) {
            motor->m_enabled = false;
            motor->m_hardStopActive = false;
        }

        return success;
    }

    case MotorCommandType::Start:
    {
        const bool success =
            motor->sendMessage(enterMotorMode);

        if (success) {
            motor->m_enabled = true;
            motor->m_hardStopActive = false;
        }

        return success;
    }

    case MotorCommandType::Zero:
        return motor->sendMessage(
            setZeroPosition
        );

    case MotorCommandType::Set:
        motor->m_cmd = command.cmd;
        motor->update();
        return true;
    }

    return false;
}
private:
    CANMotorMIT& m_leftMotor;
    CANMotorMIT* m_rightMotor;
    uint32_t m_canRate;
};
#endif