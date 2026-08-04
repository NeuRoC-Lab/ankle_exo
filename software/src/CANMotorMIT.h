#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

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

enum class MotorState : uint8_t
{
    Stopped,
    Running,
    Recovery,
    HardStopped
    };

// a helper function to print a stringed text for the current state
constexpr std::string_view toString(MotorState state)
{
    switch (state) {
        case MotorState::Stopped: return "Stopped";
        case MotorState::Running: return "Running";
        case MotorState::Recovery: return "Recovery";
        case MotorState::HardStopped: return "HardStopped";
    }

    return "Unknown";
}


#if defined(PLATFORM_TEENSY41)

#include <ArduinoJSON.h>

unsigned long lastUpdate {};
class CANMotorMIT {
/*
One CANMotorMIT instance per CubeMars motor
*/
public:
    CANMotorMIT(
    const byte canId,
    const AK60Params& motorSettings, // motor settings which decodes/encoders the ranges for MIT
    const AK60Params& motorSoftwareConstraints, // motor software constraints (which clamp the user's commanded values)
    const AK60Params& motorRunningConstraints, // motor "absolute" / running constraints.
    MotorCmd& cmd,
    uint32_t kPrintEvery = 20

)
    : m_canId(canId),
      m_cmd(cmd),
      commandToSend(cmd), // initialize commandTosend here
      m_reply{},
      m_enabled(false), // to control the motor externally from the function? //TODO remove that if deemed unecessary
      m_motorSettings(motorSettings),
      m_motorSoftwareConstraints(motorSoftwareConstraints),
      m_motorRunningConstraints(motorRunningConstraints),
      m_printCounter(0),
      m_kPrintEvery(kPrintEvery),
      m_state(MotorState::Stopped)
{}

    MotorReply m_reply;
    MotorCmd& m_cmd;
    MotorCmd commandToSend; // actual command being sent
    bool m_enabled;
    MotorState m_state; // state
	const byte m_canId;
	const AK60Params& m_motorSettings {};
    const AK60Params& m_motorSoftwareConstraints {};
    const AK60Params& m_motorRunningConstraints {};


    bool resetMotor()
    {
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
     return true;
    }

    float limitTorqueCommand(float requestedTorque) const
    {
        constexpr float brakingDistance = 0.25f;

        const float p = m_reply.position;
        const float pMin = m_motorSoftwareConstraints.p_min;
        const float pMax = m_motorSoftwareConstraints.p_max;

        // Positive torque is assumed to increase position.
        if (requestedTorque > 0.0f) {
            const float distanceToUpperLimit = pMax - p;

            const float scale = std::clamp(
                distanceToUpperLimit / brakingDistance,
                0.0f,
                1.0f
            );

            return requestedTorque * scale;
        }

        // Negative torque is assumed to decrease position.
        if (requestedTorque < 0.0f) {
            const float distanceToLowerLimit = p - pMin;

            const float scale = std::clamp(
                distanceToLowerLimit / brakingDistance,
                0.0f,
                1.0f
            );

            return requestedTorque * scale;
        }

        return 0.0f;
    }

    void update()
    {
        if(millis() - lastUpdate > 1000){
            Serial.print("Current motor status : ");
            Serial.print(toString(m_state).data());
            Serial.print("  Enabled ? ");
            Serial.print(m_enabled);
            Serial.print("  Position ? ");
            Serial.println(m_reply.position);
            lastUpdate = millis();
        }

        if (!m_enabled)
        {
            return;
        }

        while (readMessages(m_reply))
        {
        #if defined(NO_LIMITS)
        #pragma message("WARNING: motor limits are DISABLED")
            m_state = MotorState::Running;
        #else
           if(isOutsideRunningLimits())
           {
              m_state = MotorState::HardStopped;
           }
          else if(isOutsideSoftwareLimits())
           {
              m_state = MotorState::Recovery;
           }
          else{
              m_state = MotorState::Running;
            }
        #endif
            // telemetry rate control
            if (++m_printCounter >= m_kPrintEvery)
            {
                m_printCounter = 0;
            }
            // handle the current state to determine motor's behaviour
            switch (m_state)
            {
               case MotorState::Recovery:
               {
                   constexpr float recoveryMargin = 0.5f;

                   if (m_reply.position > m_motorSoftwareConstraints.p_max) {
                       commandToSend.position =
                           m_motorSoftwareConstraints.p_max - recoveryMargin;
                   }
                   else if (m_reply.position < m_motorSoftwareConstraints.p_min) {
                       commandToSend.position =
                           m_motorSoftwareConstraints.p_min + recoveryMargin;
                   }

                   commandToSend.velocity = 0.0f;

                   // Start gently and increase gradually.
                   commandToSend.kp = 2.0f;
                   commandToSend.kd = 0.5f;

                   // Remove the user's outward feedforward torque.
                   commandToSend.torque = 0.0f;
                   break;
               }
               case MotorState::Running:
               {
                   commandToSend = m_cmd;

                   commandToSend.kp = 0.0f;
                   commandToSend.kd = 0.0f;

                    #if !defined(NO_LIMITS)
                   commandToSend.torque =
                       limitTorqueCommand(m_cmd.torque);
                    #endif
                   break;
               }
               case MotorState::HardStopped:
                    sendMessage(neutralMITCommand);
                    sendMessage(exitMotorMode);
                    m_enabled = false;
                    m_state = MotorState::Stopped;
                    return;

               case MotorState::Stopped:
                    break;
            }
        }
        if (m_enabled && !updateMIT()) // m_enabled
        {
            //Serial.println("Failed to send MIT command");
            sendMessage(neutralMITCommand);
        }
}

        bool updateMIT()
        // updates the motor with the most recent command buffer (internal state variables)
        {
        uint8_t tx_buf[8]; // create a local array to store the command
        pack_cmd(
                tx_buf,
                commandToSend.position,
                commandToSend.velocity,
                commandToSend.kp,
                commandToSend.kd,
                commandToSend.torque
            );
        return sendMessage(tx_buf);
        }
bool isOutsideRunningLimits() const
{
    return
        m_reply.position < m_motorRunningConstraints.p_min ||
        m_reply.position > m_motorRunningConstraints.p_max ;//||

        //m_reply.velocity < m_motorRunningConstraints.v_min ||
        //m_reply.velocity > m_motorRunningConstraints.v_max ||

        //m_reply.torque < m_motorRunningConstraints.trq_min ||
        //m_reply.torque > m_motorRunningConstraints.trq_max;
}

    bool isOutsideSoftwareLimits() const
    {
        return
            m_reply.position < m_motorSoftwareConstraints.p_min ||
            m_reply.position > m_motorSoftwareConstraints.p_max ; // ||

            //m_reply.velocity < m_motorSoftwareConstraints.v_min ||
            //m_reply.velocity > m_motorSoftwareConstraints.v_max ||

            //m_reply.torque < m_motorSoftwareConstraints.trq_min ||
            //m_reply.torque > m_motorSoftwareConstraints.trq_max;
    }



// What does this do ???
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
            std::clamp(
                p_in,
                m_motorSoftwareConstraints.p_min,
                m_motorSoftwareConstraints.p_max
            ),
            m_motorSettings.p_min,
            m_motorSettings.p_max,
            16
        );
    
        uint16_t velocity = float_to_uint(
            std::clamp(
                v_in,
                m_motorSoftwareConstraints.v_min,
                m_motorSoftwareConstraints.v_max
            ),
            m_motorSettings.v_min,
            m_motorSettings.v_max,
            12
        );
    
        uint16_t kp = float_to_uint(
            std::clamp(
                kp_in,
                m_motorSoftwareConstraints.kp_min,
                m_motorSoftwareConstraints.kp_max
            ),
            m_motorSettings.kp_min,
            m_motorSettings.kp_max,
            12
        );
    
        uint16_t kd = float_to_uint(
            std::clamp(
                kd_in,
                m_motorSoftwareConstraints.kd_min,
                m_motorSoftwareConstraints.kd_max
            ),
            m_motorSettings.kd_min,
            m_motorSettings.kd_max,
            12
        );
    
        uint16_t trq = float_to_uint(
            std::clamp(
                trq_in,
                m_motorSoftwareConstraints.trq_min,
                m_motorSoftwareConstraints.trq_max
            ),
            m_motorSettings.trq_min,
            m_motorSettings.trq_max,
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
    x = std::clamp(x,x_min,x_max);
    float span = x_max - x_min;
    float max_int = (float)(((unsigned long)1 << bits) -1);
    return (uint16_t)((x-x_min)* max_int / span);
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
        CANMotorMIT_Teensy(byte canId,const AK60Params& motorSettings,const AK60Params& motorSoftwareConstraints,const AK60Params& motorRunningConstraints ,MotorCmd& cmd,uint32_t kPrintEvery=20)
        : CANMotorMIT(canId, motorSettings,motorSoftwareConstraints,motorRunningConstraints,cmd,kPrintEvery)
        {}

             bool begin() override{
                return true;

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

        bool readMessages(MotorReply& reply) override
        {
            CAN_message_t rxMsg;

            while (TeensyCAN.read(rxMsg)) {
                if (rxMsg.len != 8) {
                    continue;
                }

                // Adapt this check to the actual reply CAN ID used by your motor.
                if (rxMsg.id != m_canId) {
                    continue;
                }

                MotorReply candidate = unpack_reply(rxMsg.buf);

                if (candidate.can_id != m_canId) {
                    continue;
                }

                reply = candidate;
                return true;
            }

            return false;
        }
    };

#else
#endif

class CANMotorMIT_Handler {

public :

    CANMotorMIT_Handler(CANMotorMIT& leftMotor,CANMotorMIT* rightMotor = nullptr,uint32_t canRate=board::teensy41::motorCanBaud) // using a pointer to indicate that the right motor is not necessary for now
        : m_leftMotor(leftMotor),
          m_rightMotor(rightMotor),
		  m_canRate(canRate)
    {
    }


	bool begin()
{
    Serial.println("Initializing CAN Bus");
    TeensyCAN.begin(); // this seems to be a void function, so how can we check if the CAN is properly initialized ?
    TeensyCAN.setBaudRate(m_canRate);
    TeensyCAN.setMaxMB(16);
    TeensyCAN.enableFIFO();
    //TeensyCAN.enableFIFOInterrupt();

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
        }

        return success;
    }

    case MotorCommandType::Start:
    {
        const bool success =
            motor->sendMessage(enterMotorMode);

        if (success) {
            motor->m_enabled = true;

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