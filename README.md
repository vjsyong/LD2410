
# LD2410 Interface for Python

  

 More documentation coming soon. In the meantime, you can read example.py to figure out how to use the module

A python interface to control and get data from a Hi-Link LD2410 module over a serial connection
  

## How to install

Run `pip install LD2410` using your package manager of choice

## Usage Instructions

An example of how to get full data from the module

```
from  LD2410  import *
import time

radar = LD2410(port="/dev/ttyUSB0") # Replace <port> with your serial port. e.g "COM3" or "/dev/ttyS0" etc. 

radar.enable_engineering_mode() # Enable engineering mode to get full data

# You must call call start() before getting data, or you get nothing
radar.start() # Start the radar module

# Get 60 data frames
for _ in range(60):
	print(radar.get_data())
	time.sleep(1) # Radar module will keep running in the background, but we only retrieve the latest data every 1 second

radar.stop()

```

### How to get the data

Use `radar.get_data()` 

It returns the following:

**In standard mode:**
`([Detection Type, Movement Gate Detection Dist, Movement Gate Energy, Static Gate Detection Dist, Static Gate Energy, Detection Dist], None, None)`

|   | Detection Type            |
|---|---------------------------|
| 0 | No Target                 |
| 1 | Moving Target             |
| 2 | Static Target             |
| 3 | Moving and Static Targets |

`Movement Gate Detection Dist`: The distance of the detected moving target from the sensor
`Movement Gate Energy`: The "energy" level of the detected moving target
`Static Gate Detection Dist`: The distance of the detected static target from the sensor
`Static Gate Energy`: The "energy" level of the detected static target


**What is a gate?**
\* *A **Gate** is a region of **N \* 75 to (N + 1 \* 75) cm** region from the sensor. For example, Gate 2 refers to a region between 1.5m and 2.25m away from the sensor*

**In engineering mode:**
`([Same as above], [Moving gate 0 energy...Moving Gate 8 Energy], [Static gate 0 energy...Static Gate 8 Energy])`

`Moving gate 0 energy...Moving Gate 8 Energy`: Get the energy levels of each gate returned as a list of integers

`Static gate 0 energy...Static Gate 8 Energy`: Get the energy levels of each gate returned as a list of integers

## Todo

1. Expand on how to set params

2. Expand on how to change sensitivities

If you are determined to use this module, check out example.py for a more indepth example of how to use this module

