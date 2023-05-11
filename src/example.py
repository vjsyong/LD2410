from ld2410 import LD2410
from ld2410_consts import *
import logging
import time


def main():
    radar = LD2410("/dev/ttyUSB0", PARAM_DEFAULT_BAUD, verbosity=logging.INFO) # Set desired level of verbosity [DEBUG, INFO, WARNING]
    
    # Get Radar Firmware version
    fw_ver = radar.read_firmware_version()
    print(fw_ver)

    # Set max detection gate for moving to 2, static to 3, and empty timeout to 1s
    radar.edit_detection_params(2, 3, 1)

    # Set the gate 3 moving energy sentivity to 50 and static sensitivity to 40
    # Note: Static sensitivity cannot be set for gate 1 and 2, it must be set to zero e.g (1, 50, 0)
    radar.edit_gate_sensitivity(3, 50, 40)

    # Retrieve the set detection parameters
    # Returns 3 arrays 
    # 1. Gate parameters. we set it earlier to (2, 3, 1)
    # 2. Motion energy sensitivities for gates 0-8 
    # 3. Static energy sensitivities for gates 0-8 
    detection_params = radar.read_detection_params()
    print(detection_params)

    # Get data in standard mode

    # Start the radar polling
    radar.start() # The radar polls asynchronously at 10Hz
    
    # Get 3 data frames 1s apart. If you use a delay less than 0.1, you will get repeat data
    for _ in range(3):
        print(radar.get_data()) # The right 2 arrays will be blank since we are polling in standard mode
        time.sleep(1)
    # Get data in engineering mode

    radar.enable_engineering_mode()
    
    # Get 3 data frames 1s apart. If you use a delay less than 0.1, you will get repeat data
    for _ in range(3):
        print(radar.get_data()) # The right 2 arrays will be blank since we are polling in standard mode
        time.sleep(1)

    radar.disable_engineering_mode()

    radar.stop() # Stop polling the radar

    # Restart the radar
    radar.restart_module()

    # Enable bluetooth (On by default, you don't have to call it)
    radar.bt_enable()

    # Get bluetooth MAC address
    print(radar.bt_query_mac())

    # Set module baud rate
    # radar.set_baud_rate(PARAM_BAUD_115200) 

    # Factory reset the module
    # radar.factory_reset()

if __name__ == "__main__":
    main()