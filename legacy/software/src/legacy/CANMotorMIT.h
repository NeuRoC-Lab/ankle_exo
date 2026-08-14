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
