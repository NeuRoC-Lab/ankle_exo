## Python API

A self-contained Python API has been written to interface the exoskeleton from a BLE compatible device in Python.

The file to use is `src/python-scripts/SingleLeg/single-exo.py`

The main `Exoskeleton` class supports context management framework, which internally handles connection to and disconnection from the BLE-enabled Arduino Nano

A typical script skeleton loooks like this : 

```python
from single-exo import Exoskeleton
# assumes your python file is in `src/python-scripts/SingleLeg`

with Exoskeleton() as ankle_exo:
    # your code goes here, connection and disconnection is being handled internally 
    pass
```

Please see the available functions to read from the sensors and to send commands to the Motors.