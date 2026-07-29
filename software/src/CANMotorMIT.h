#pragma once
#include <Arduino.h>
#include "ProtocolTypes.h"
#include "SerialProtocol.h"
#include <ArduinoJson.h>


float p_target = 10.0f;
float v_target = 0.0f;
float kp_target = 0.5f;
float kd_target = 0.5f;
float trq_target = 0.0f;

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




#if !defined(PLATFORM_TEENSY41) && !(defined(PLATFORM_RENESAS_RA) || defined(PLATFORM_ATMEL_AVR))
#else
#include <ArduinoJSON.h>
class CANMotorMIT {
/*
Generic methods shared between the two CAN implementations (i.e Teensy 4.1 and Arduino UNO R4)
One CANMotorMIT instance per CubeMars motor
*/
public:
    CANMotorMIT(
    byte canId,
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

    uint8_t m_canId;
    MotorCmd& m_cmd;   // reference, not copy
    MotorReply m_reply;
    bool m_enabled;
    const AK60Params* m_motorSettings;
    const AK60Params* m_motorSoftwareConstraints;
    const AK60Params* m_motorRunningConstraints;


bool resetMotor()
{
    m_enabled = false;

    Serial.println("Exiting MIT motor mode...");

    if (!sendMessage(exitMotorMode)) {
        return false;
    }

    delay(500);

    Serial.println("Entering MIT motor mode...");

    if (!sendMessage(enterMotorMode)) {
        return false;
    }

    delay(100);

    // Zero position, velocity, gains, and feedforward torque.
    if (!sendMessage(neutralMITCommand)) {
        return false;
    }

    m_enabled = true;

    return true;
}

void update(){

        if(m_enabled){
        uint8_t tx_buf[8];
        pack_cmd(
            tx_buf,
            m_cmd.position,
            m_cmd.velocity,
            m_cmd.kp,
            m_cmd.kd,
            m_cmd.torque
        );
        if (!sendMessage(tx_buf)) {
            Serial.println("MIT command send failed");
        }
        }

        while (readMessages(m_reply)) {
            checkHardStop();
            if (++m_printCounter >= m_kPrintEvery) {
                //print_can_msg(m_reply);
                m_printCounter = 0;
            }
        }
    }
void checkHardStop(){
    if (m_enabled &&
        (m_reply.torque   > m_motorRunningConstraints->trq_max ||
         m_reply.torque   < m_motorRunningConstraints->trq_min ||
         m_reply.velocity > m_motorRunningConstraints->v_max   ||
         m_reply.velocity < m_motorRunningConstraints->v_min   ||
         m_reply.position > m_motorRunningConstraints->p_max   ||
         m_reply.position < m_motorRunningConstraints->p_min))
    {
    Serial.println("Detected position/velocity/torque overshoot ! Stopping the motor");
    Serial.print("Position at overshoot");
    Serial.print(m_reply.position);
    Serial.print(" Velocity at overshoot");
    Serial.print(m_reply.velocity);
    Serial.print(" Torque at overshoot");
    Serial.println(m_reply.torque);

    m_enabled = false;
    sendMessage(exitMotorMode);
    // leaving motor mode seems to disable logging of messages
    }
}
// This can be placed outside the class, it does not depend on any instance parameter
void print_can_msg(MotorReply reply){
        Serial.print("  motor id: ");
        Serial.print(reply.can_id);

        Serial.print(" pos(rad): ");
        Serial.print(reply.position, 4);

        Serial.print(" vel(rad/s): ");
        Serial.print(reply.velocity, 4);

        Serial.print(" trq(N*m): ");
        Serial.print(reply.torque, 4);

        Serial.print(" temp(C): ");
        Serial.print(reply.temperature);

        Serial.print(" err: ");
        Serial.println(reply.error);
}

void writeReplyToJson(JsonObject object) const
{
    object[TelemetryKey::MotorId] = m_reply.can_id;
    object[TelemetryKey::MotorPos] = m_reply.position;
    object[TelemetryKey::MotorVel] = m_reply.velocity;
    object[TelemetryKey::MotorTrq] = m_reply.torque;
    object[TelemetryKey::MotorTemp] = m_reply.temperature;
    object[TelemetryKey::MotorErr] = m_reply.error;
}

bool handleSerialCommand(const CommandPayload& command){
    // handles a command
    if (command.motorId != m_canId)
        {
        return false;
        } // discard messages that do not match the motor's CAN ID

    if(command.type == MotorCommandType::Stop){
        return sendMessage(exitMotorMode);
    }
    else if (command.type == MotorCommandType::Start){
        return sendMessage(enterMotorMode);
     }
    else if(command.type == MotorCommandType::Zero){
        return sendMessage(setZeroPosition);
    }
    else if(command.type == MotorCommandType::Set){
        m_cmd = command.cmd; // update m_cmd
        update(); // call update to make sure the command is immediately packed and sent
        return true;
    }
    return false;
}

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
virtual void begin(); // this function will call platform specific initialization
virtual bool sendMessage(const uint8_t data[8]);
virtual bool readMessages(MotorReply& reply); // will pop the element in the FIFO mailbox
};

// subclasses for UNOR4 and Teensy implementations (their libraries differ)

#if defined(PLATFORM_TEENSY41)
    #include <FlexCAN_T4.h>
    //using PlatformCanBus = CanBus_Teensy41;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> TeensyCAN;
    // Teensy 4.1 CAN1:
    // RX = pin 23
    // TX = pin 22
    class CANMotorMIT_Teensy : public CANMotorMIT {

    public:
        CANMotorMIT_Teensy(byte canId,const AK60Params* motorSettings,const AK60Params* motorSoftwareConstraints,const AK60Params* motorRunningConstraints ,MotorCmd& cmd,uint32_t kPrintEvery=20)
        : CANMotorMIT(canId, motorSettings,motorSoftwareConstraints,motorRunningConstraints,cmd,kPrintEvery)
        {}

            virtual void begin(){
            // initializes the CAN controller on the Teensy 41

                Serial.println("Now initializing CAN communication");
                TeensyCAN.begin();
                TeensyCAN.setBaudRate(1000000);//TeensyCAN.setBaudRate(1000000);
                TeensyCAN.setMaxMB(16);
                TeensyCAN.enableFIFO();
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

#elif defined(PLATFORM_RENESAS_RA)
    #include <Arduino_CAN.h>

    class CANMotorMIT_Renesas : public CANMotorMIT {
    //using PlatformCanBus = CanBus_RenesasRA;
    public:
        CANMotorMIT_Renesas(byte canId,const AK60Params* motorSettings,const AK60Params* motorSoftwareConstraints,const AK60Params* motorRunningConstraints,MotorCmd& cmd,uint32_t kPrintEvery=20)
        : CANMotorMIT(canId,motorSettings,motorSoftwareConstraints,motorRunningConstraints,cmd,kPrintEvery)
        {}

            virtual void begin(){
            // initializes the CAN controller on the Teensy 41
            Serial.println("Now initializing CAN communication");
            if (!CAN.begin(CanBitRate::BR_1000k)) // 1M baudrate
            {
                Serial.println("CAN.begin(...) failed.");
                    for (;;) {}
            }
            Serial.println("Successfully started CAN Communication. Entering motor mode");
            delay(1000);
            if(!resetMotor()){
                Serial.println("Failed to start motor. Please check the Motor power supply is ON and the Motor is connected to the CAN bus");
            }
            else {
                Serial.println("Successfully started motor");
            }

        }
    virtual bool sendMessage(const uint8_t data[8]){
        CanMsg const out_msg(CanStandardId(m_canId), 8, data); //sizeof(data)/sizeof(data[0]) why does that not evaluate to 8 properly ??? Because in CPP, data[8] is transformed into a pointer so sizeof(data) = size of the pointer, not the array
        return CAN.write(out_msg) > 0;
    }

    virtual bool readMessages(MotorReply& reply) override {
        while (CAN.available())
        {
        CanMsg const rxMsg = CAN.read();
        if (rxMsg.data_length != 8)
        {
        continue;
        }
        reply = unpack_reply(rxMsg.data);
        return true;   // popped one valid message
        }
        return false;      // no valid message available right now
       }
    };

#else
    #error "No CAN platform selected. Make sure to use Arduino UNO R4 or Teensy 4.1"
#endif

#if !defined(PLATFORM_TEENSY41) && !defined(PLATFORM_RENESAS_RA)
#error "No CAN platform selected. Define PLATFORM_TEENSY41 or PLATFORM_RENESAS_RA."
#endif

#endif
