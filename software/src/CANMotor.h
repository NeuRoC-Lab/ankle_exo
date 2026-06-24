#include <Arduino.h>
#pragma once

float p_target = 10.0f;
float v_target = 0.0f;
float kp_target = 0.5f;
float kd_target = 0.5f;
float trq_target = 0.0f;

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

typedef struct
{
    uint8_t can_id;
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
} MotorCmd;

// AK60 Motor parameter definitions
typedef struct {
    float p_min,p_max;
    float v_min,v_max;
    float kp_min,kp_max;
    float kd_min,kd_max;
    float trq_min,trq_max;

} AK60Params;

static constexpr AK60Params motorParams = {
    -12.5f,  12.5f,   // position (rad)
    -45.0f,  45.0f,   // velocity
      0.0f, 500.0f,   // kp
      0.0f,   5.0f,   // kd
    -15.0f,  15.0f    // torque
};

typedef struct
{
    uint8_t can_id;
    float position;
    float velocity;
    float torque;
    uint8_t temperature;
    uint8_t error;
} MotorReply;



class CANMotor {
/*
Generic methods shared between the two CAN implementations (i.e Teensy 4.1 and Arduino UNO R4)

void pack_cmd() -> in place modification of tx_buf

MotorReply unpack_reply(const uint8_t rx_buf[8]) -> unpacks a command from RX buf
*/
public :
     CANMotor(byte canId,const AK60Params* motorSettings){
        m_canId = canId;
        m_motorSettings = motorSettings;
    }

bool resetMotor(uint32_t id){
    Serial.println("Exiting MIT motor mode...");
    if(!sendMessage(id, neutralMITCommand)){
        return false;
    }
    delay(1000);
    if(!sendMessage(id, exitMotorMode)){
        return false;
    }
    delay(1000);
    if(!sendMessage(id, enterMotorMode)){
        return false;
    }
    return true;
    }

void update(MotorCmd cmd){
        uint8_t tx_buf[8];
        pack_cmd(
            tx_buf,
            cmd.position,
            cmd.velocity,
            cmd.torque,
            cmd.kp,
            cmd.kd
        );
        if (!sendMessage(cmd.can_id, tx_buf)) {
            Serial.println("MIT command send failed");
        }
}

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

// to prevent one from calling these functions directly from the SuperClass
protected:
    // internal variables
    uint8_t m_canId;
    const AK60Params* m_motorSettings;

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
        p_in,
        motorParams.p_min,
        motorParams.p_max,
        16
    );

    uint16_t velocity = float_to_uint(
        v_in,
        motorParams.v_min,
        motorParams.v_max,
        12
    );

    uint16_t kp = float_to_uint(
        kp_in,
        motorParams.kp_min,
        motorParams.kp_max,
        12
    );

    uint16_t kd = float_to_uint(
        kd_in,
        motorParams.kd_min,
        motorParams.kd_max,
        12
    );

    uint16_t trq = float_to_uint(
        trq_in,
        motorParams.trq_min,
        motorParams.trq_max,
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
    float span = x_max - x_min;
    float max_int = (float)(((unsigned long)1 << bits) -1);
    return (uint16_t)((x-x_min)* max_int / span);
    }
float constrain_float(float x, float x_min, float x_max)
    {
    if (x < x_min)
    {
        return x_min;
    }

    if (x > x_max)
    {
        return x_max;
    }

    return x;
    }

virtual ~CANMotor() = default;
virtual void begin(); // this function will call platform specific initialization
virtual bool sendMessage(uint32_t id,const uint8_t data[8]);
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
    class CANMotor_Teensy : public CANMotor {

    public:
        CANMotor_Teensy(byte canId,const AK60Params* motorSettings)
        : CANMotor(canId, motorSettings)
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

            virtual bool sendMessage(uint32_t id,const uint8_t data[8]) override {
            CAN_message_t msg;
            msg.id = id;
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

    class CANMotor_Renesas : public CANMotor {
    //using PlatformCanBus = CanBus_RenesasRA;
    public:
        CANMotor_Renesas(byte canId,const AK60Params* motorSettings)
        : CANMotor(canId,motorSettings)
        {}

            virtual void begin(){
            // initializes the CAN controller on the Teensy 41
            Serial.println("Now initializing CAN communication");
            if (!CAN.begin(CanBitRate::BR_1000k)) // 1M baudrate
            {
                Serial.println("CAN.begin(...) failed.");
                    for (;;) {}
            }
        }
    virtual bool sendMessage(uint32_t id,const uint8_t data[8]){
        CanMsg const out_msg(CanStandardId(id), 8, data); //sizeof(data)/sizeof(data[0]) why does that not evaluate to 8 properly ??? Because in CPP, data[8] is transformed into a pointer so sizeof(data) = size of the pointer, not the array
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