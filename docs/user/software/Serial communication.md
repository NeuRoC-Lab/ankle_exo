### Serial communication between the Teensy 4.1 and the host computer

!!! info "Note for later iterations" 
    The use of Serial communication (UART) seemed like an ideal choicen for us because 1) it is easier to implement than SPI 2) we can easily transpose it to Bluetooth later on

## Teensy payload formating

A typical packet from the Teensy looks like this, as seen from the Serial Monitor

```python
 {
    "LLC1":2.512674,
    "LLC2":2.532015,
    "RLC1":0.032234,
    "RLC2":0.030623,
    "LENC":28671,
    "RENC":65295,
    "MOTORS":[{"MTR_ID_DEC":2,"MTR_POS_RAD":0.476654,"MTR_VEL_RADS":0.032967,"MTR_TRQ_NM":-0.032967,"MTR_TEMP_DEG":64,"MTR_ERR_DEC":0}]
}
```
the main fields are : 

* `LLCx` : left load cell x (1 or 2)
* `RLCx` : right load cell x (1 or 2)
* `LENC` : left encoder count
* `RENC` : right encoder count
* `MOTORS` : motors field, contains a list of motor telemetry data (one per each) with keys `MTR_ID_DEC`, `MTR_POS_RAD`, `MTR_VEL_RADS`, `MTR_TEMP_DEG`, `MTR_ERR_DEC`

On python, you need to do the following to retrieve data (See `testUart.py` for an application)

1. Parse the line from the Serial with `line = Serial.readLines('\n')` 
2. Parse the JSON to convert the string into a dictionnary-like array with `packet = json.loads(line)`
3. Extract individual fields with `.get()` method. For example :
```python
left_load_cell = packet.get("LLC1")  # left load cell 1 voltage
right_encoder = packet.get("LENC") # left encoder count
```