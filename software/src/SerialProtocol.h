//
// Created by Oscar Tesniere on 20/07/2026.
// for handling communication between the Teensy and the Nano over UART
// Requirements :
// Serialize and deserialize data to/from teensy and nano
// ease ofn implementation with both 1) direct serial communication b/w laptop and teensy (for debugging purposes) and 2) bluetooth interfacing with the Nano
// Ideally have a datapayload struct which is common to both the pre-serialization (when teensy sends data to nano for ex) and post-deserialization(ex after the nano has deserialized the data). This makes it more modular
//

struct DataPayload {
    // load cells
    unsigned long LeftLoadCell1;
    unsigned long LeftLoadCell2;
    unsigned long RightLoadCell1;
    unsigned long RightLoadCell2;
    // encoders

    EncoderPositions positions;

    MotorReply* motorReplies; // to account for the two motors
}

