
# useful resources about the subject
"""
someone who built a wrapper around Bleak :  https://ladvien.com/python-serial-terminal-with-arduino-and-bleak/
bleak pypi : https://pypi.org/project/bleak/
bleak readthedocs : https://bleak.readthedocs.io/en/latest/
bleak github : https://github.com/hbldh/bleak
"""

# the goal of this code is to connect to the Arduino Nano through BLE and subscribe to the main serve to get updated data from the load cells, encoders, and motors
# for now we primarily focus on _receiving_ data, later on we'll implement some user command pipelines in particular for the motor.

import asyncio
import struct
from bleak import BleakClient, BleakScanner


SERVICE_UUID = "CF45813E-4358-4903-B961-09996BB081FB" # the UUID of the service used
DEVICE_NAME = "AnkleExo"

LOAD_CELL_UUID = "CA87289F-102B-4078-AD8C-8F53063547A6"
MOTOR_UUID = "E0D883F6-705C-4A11-B117-E2B0909CC68E"
ENCODER_UUID = "094A717B-0C7F-4A23-BFD1-A4924E6E7DAB"

async def main() -> None:
    # could also have used find_device_by_name ; on the Nano this is set with BLE.setLocalName("AnkleExo"); so "AnkleExo" is the local name
    device = await BleakScanner.find_device_by_filter(
        lambda device, advertisement:
        SERVICE_UUID.lower()
        in [uuid.lower() for uuid in advertisement.service_uuids],
        timeout=10.0,
    )

    if device is None:
        print("Could not find the NANO BLE by searching its advertised service UUID. Trying by searching with its local name")
        asyncio.sleep(1)

    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME,
        timeout=10.0,
    )

    if device is None:
        print("Failed to find the NANO Service")

    async with BleakClient(device) as client:
        print("Connected")
        print(client.services)
        # read a characteristic here


        print("Reading GATT characteristics once for the encoders, motors, and load cells ")
        data = await client.read_gatt_char(MOTOR_UUID)
        '''
        typedef struct
        {
            uint8_t can_id;       // 1 byte
            float position;       // 4 bytes
            float velocity;       // 4 bytes
            float torque;         // 4 bytes
            uint8_t temperature;  // 1 byte
            uint8_t error;        // 1 byte
        } MotorReply;
        '''
        print("Motor Characteristic")
        print(data.hex(),struct.unpack("<B3x3fBB2x", data))
        data = await client.read_gatt_char(LOAD_CELL_UUID)
        print("Load Cells Characteristic")
        print(data.hex(),struct.unpack("<4f", data))
        data = await client.read_gatt_char(ENCODER_UUID)
        print("Encoders Characteristic")
        print(data.hex(),struct.unpack("<2H", data))


asyncio.run(main())

## Test script
# next we need to implement SUBSCRIBE mechanism to get updates from encoders, motors and load cells

## Can get the "feed" as an iterator :
'''
async BleakScanner.advertisement_data() → AsyncGenerator[tuple[BLEDevice, AdvertisementData], None]
'''


'''
Warning

Although example code frequently initializes BleakClient with a Bluetooth address for simplicity, it is not recommended to do so for more complex use cases. There are several known issues with providing a Bluetooth address as the address_or_ble_device argument.

    macOS does not provide access to the Bluetooth address for privacy/ security reasons. Instead it creates a UUID for each Bluetooth device which is used in place of the address on this platform.

    Providing an address or UUID instead of a BLEDevice causes the connect() method to implicitly call BleakScanner.discover(). This is known to cause problems when trying to connect to multiple devices at the same time.

'''