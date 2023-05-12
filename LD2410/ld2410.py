from .ld2410_consts import *
from collections import deque
import serial
import struct
import threading
import time
import logging
import signal


class Queue:
    def __init__(self, max_size):
        self.queue = deque(maxlen=max_size)
    
    def add(self, element):
        self.queue.append(element)
    
    def byte_str(self):
        return b"".join(list(self.queue))



class LD2410():
    def __init__(self, port, baud_rate=PARAM_DEFAULT_BAUD, timeout=1, verbosity=logging.DEBUG) -> None:
        self.port = port
        self.baudrate = baud_rate
        self.timeout = timeout
        self.eng_mode = False
        self.read_fail_count = 0
        
        # threading data variables
        self.last_detection = None
        self._lock = threading.Lock()
        self._worker_thread = None
        self._stop_event = threading.Event()

        logging.basicConfig(level=verbosity)

        self.ser = serial.Serial(port, BAUD_LOOKUP[baud_rate], timeout=timeout)
        logging.info(f"Serial port initialised at {port}, with baud rate {BAUD_LOOKUP[baud_rate]}")

    # Validate that the input is within a valid range
    @staticmethod
    def validate_range(input, lower, upper):
        if input in range(lower, upper):
            return True
        else:
            input_name = f'{input=}'.split('=')[0]
            raise Exception(f"{input_name} {input} is not a valid setting, please pick a value between {lower} and {upper}")

    # Helper functions
    @staticmethod
    def frame_wrapper(command):
        return bytes.fromhex(CMD_HEADER + command + CMD_MFR)

    # Convert a decimal integer to a 4 byte little endian string
    @staticmethod
    def int_to_4b(num):
        hex_string = bytearray.fromhex(struct.pack('>I', num).hex())
        hex_string.reverse()
        return bytes(hex_string).hex()

    # Base functions

    # Sends a dataframe encoded as bytes enclosed within a format specific header
    # Returns the acknowledged data received from the radar
    def send_frame(self, command):
        while True:
            try:
                # Wrap up the command 
                command_bytes = self.frame_wrapper(command)
                
                self.ser.write(command_bytes)
                ret_bytes = self.ser.read(MAX_BUFFER_SIZE)
                
                logging.debug(f"Sending data:  {command_bytes.hex(' ')}")
                logging.debug(f"Received data: {ret_bytes.hex(' ')}")
    
                return ret_bytes # Returns the response given by the radar module
            
            except Exception as e:
                logging.debug(e)
    
    def send_command(self, command):
        # Enable config mode
        self.send_frame(CMD_CONFIG_ENABLE)
        # Send command
        ret_bytes = self.send_frame(command)
        # Disable config mode
        self.send_frame(CMD_CONFIG_DISABLE)

        return ret_bytes
        

    # Configure Detection Gates and Detect Duration
    def edit_detection_params(self, moving_max_gate, static_max_gate, timeout):
        logging.info("Editing detection parameters")
        self.validate_range(moving_max_gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(static_max_gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(timeout, TIMEOUT_MIN, TIMEOUT_MAX+1)

        command = CMD_PARAM_EDIT + PARAM_MAX_MOVING_GATE + self.int_to_4b(moving_max_gate) \
                      + PARAM_MAX_STATIC_GATE + self.int_to_4b(static_max_gate) \
                      + PARAM_EMPTY_DURATION + self.int_to_4b(timeout)
        
        self.send_command(command)

    # Read the currently configured parameters 
    #
    # Returns 3 arrays, 
    # 
    # first array: [moving gate threshold, static gate threshold, empty timeout]
    # second array: [moving gate sens 0.... moving gate sens 8]
    # third array: [static gate sens 0.... static gate sens 8]

    def read_detection_params(self):
        # Send command to retrieve parameters
        logging.info("Reading detection parameters")
        ret = self.send_command(CMD_PARAM_READ)
        
        # Process response
        # Threshold Params
        thresholds = [ret[REF_MAX_MOVING_GATE], ret[REF_MAX_STATIC_GATE], ret[REF_EMPTY_TIMEOUT]]


        # Movement Gate Sensitivities
        move_sens = [int(byte) for byte in ret[REF_MOVING_GATE_SENS_0:REF_MOVING_GATE_SENS_8]]
        # Static Gate Sensitivities
        static_sens = [int(byte) for byte in ret[REF_STATIC_GATE_SENS_0:REF_STATIC_GATE_SENS_8]]
        
        logging.debug(f"Thresholds:{thresholds}, Movement Sens:{move_sens}, Static Sens:{static_sens}")
        return thresholds, move_sens, static_sens
        
    # Enable Engineering Mode
    # Adds energy level of each gate to the radar output
    def enable_engineering_mode(self):
        logging.info("Enabling engineering mode")
        self.eng_mode = True
        self.send_command(CMD_ENG_MODE_ENABLE)
        
    # Disable Engineering Mode
    def disable_engineering_mode(self):
        logging.info("Disabling engineering mode")
        self.eng_mode = False
        self.send_command(CMD_ENG_MODE_DISABLE)

    # Configure Gate Movement and Static Sensitivities
    def edit_gate_sensitivity(self, gate, moving_sens, static_sens):
        logging.info("Editing gate sensitivity")
        self.validate_range(gate, GATE_MIN, GATE_MAX+1)
        self.validate_range(moving_sens, SENS_MIN, SENS_MAX+1)

        if gate == 1 or gate == 2:
            logging.warning("You cannot set gate 1 or 2 static sensitivity to anything other than 0")
            self.validate_range(static_sens, 0, 1)
        else:
            self.validate_range(static_sens, SENS_MIN, SENS_MAX+1)

        command = CMD_GATE_SENS_EDIT + PARAM_GATE_SELECT + self.int_to_4b(gate) \
                      + PARAM_MOVING_GATE_WORD + self.int_to_4b(moving_sens) \
                      + PARAM_STATIC_GATE_WORD + self.int_to_4b(static_sens)
        
        self.send_command(command)

    # Read Firmware Version
    def read_firmware_version(self):
        logging.info("Reading firmware version")
        ret = self.send_command(CMD_FIRMWARE_READ)

        # Need to flip from little endian to big endian
        fw_major = bytes(reversed(ret[REF_FW_MAJOR_HEAD:REF_FW_MAJOR_TAIL]))
        fw_minor = bytes(reversed(ret[REF_FW_MINOR_HEAD:REF_FW_MINOR_TAIL]))

        fw_version = f"V{fw_major[0]}.{fw_major[1]:02}.{fw_minor.hex()}"
        
        return fw_version
    
    # Used to set the baud rate of the module. 
    # Setting "reconnect=True" will allow the driver to 
    # restart and reset the module and adapt the host to the new baud rate

    # Check consts.py to find the right settings
    def set_baud_rate(self, baud_rate, reconnect=True):
        logging.info(f"Setting baud rate to {BAUD_LOOKUP[baud_rate]}")
        if baud_rate not in PARAM_ACCEPTABLE_BAUDS:
            raise Exception(f"{baud_rate} is not a valid setting. Consult consts.py to find an appropriate setting.")
        
        self.send_command(CMD_BAUD_RATE_SET + baud_rate)
        
        if reconnect:
            logging.info("Baud rate set command issued. Calling restart.")
            # Restart the driver with the new baudrate
            self.restart_module(BAUD_LOOKUP[baud_rate])

    def factory_reset(self, reconnect=True):
        logging.warning("Module will now be factory reset")
        self.send_command(CMD_FACTORY_RESET)
        if reconnect:
            self.restart_module(BAUD_LOOKUP[PARAM_DEFAULT_BAUD])

    def restart_module(self, new_baud=None):
        logging.info("Restarting module")
        if new_baud:
            self.baudrate = new_baud

        self.send_command(CMD_RESTART)
        self.ser.close()
        self.ser = self.ser = serial.Serial(self.port, BAUD_LOOKUP[self.baudrate], timeout=self.timeout)
        self.eng_mode = False

        time.sleep(1)

    # Enable Bluetooth
    def bt_enable(self):
        logging.info("Enabling Bluetooth")
        self.send_command(CMD_BT_ENABLE)

    # Disable Bluetooth
    def bt_disable(self):
        logging.info("Disabling Bluetooth")
        self.send_command(CMD_BT_DISABLE)

    # Get Bluetooth MAC Address
    # Returns a string in the format of xx:xx:xx:xx:xx:xx
    def bt_query_mac(self):
        logging.info("Getting Bluetooth Address")
        ret = self.send_command(CMD_BT_MAC_QUERY)
        mac = ret[REF_BT_ADDR_HEAD:REF_BT_ADDR_TAIL].hex(":") 
        logging.debug(f"Bluetooth address is {mac}")
        return mac
    
    # Get Radar Frame
    def get_data_frame(self):
        logging.debug("Getting raw dataframe")
        # Keep cycling the buffer until frame starts
        buffer = Queue(max_size=4)
        while buffer.byte_str() != bytes.fromhex(REF_READ_HEADER):
            try:
                b = self.ser.read()
            except:
                self.read_fail_count += 1
                logging.debug("Serial failed to read data. Trying again")
                self.read_fail_count += 1
                if self.read_fail_count > 32:
                    logging.warning("Serial failed to read data many times in a row. Please check if the baud rate is correct. Hint: Check the firmware version, if it looks weird, it's probably wrong")
                b = b""
            buffer.add(b)
        
        # Different packet lengths depending on whether engineering mode is on
        if self.eng_mode:
            read_len = REF_ENG_MODE_PACKET_LEN
        else:
            read_len = REF_NORMAL_PACKET_LEN

        try:
            ret_candidate = self.ser.read(read_len)
        except:
            
            logging.debug("Serial failed to read data. Skipping this read")
            return None

        logging.debug(f"get_data_frame() returning {ret_candidate.hex(' ')}")

        # Catch engineering mode not set error
        if ret_candidate[REF_ENG_CHECK_IDX] == REF_ENG_CHECK and self.eng_mode == False: # Engineering mode is on, but not set in driver
            logging.warning("Data seems to be in engineering mode format. However, driver isn't set to use parse engineering mode. Setting it now")
            self.eng_mode = True
        elif ret_candidate[REF_PACKET_CRC_IDX:] != bytes.fromhex(REF_PACKET_CRC):
            logging.warning(f'Checksum not correct received this packet {ret_candidate.hex(" ")}')
            # raise Exception("Checksum of received data is wrong. Data may be corrupted")

        if ret_candidate:
            self.read_fail_count = 0
        
        return ret_candidate

    # Sets 3 lists (standard, move_energies, static_energies). If engineering mode is disabled, second and third list is empty
    def get_radar_data(self):
        logging.debug("Getting raw dataframe")

        # Keep trying until a successful read
        ret = self.get_data_frame()
        while not ret:
            ret = self.get_data_frame()

        move_energies = None
        static_energies = None
        
        target_type = ret[REF_TARGET_TYPE] # 0 No Target, 1 Moving, 2 Static, 3 Static + Moving
        
        moving_target_dist = int(bytes(reversed(ret[REF_TARGET_MOVE_DIST_HEAD:REF_TARGET_MOVE_DIST_TAIL+1])).hex(), 16)
        moving_target_energy = ret[REF_TARGET_MOVE_ENERGY]
        
        static_target_dist = int(bytes(reversed(ret[REF_TARGET_STATIC_DIST_HEAD:REF_TARGET_STATIC_DIST_TAIL+1])).hex(), 16)
        static_target_energy = ret[REF_TARGET_STATIC_ENERGY]

        detection_dist = ret[REF_DETECT_DIST]

        standard_frame = [target_type, moving_target_dist, moving_target_energy, static_target_dist, static_target_energy, detection_dist]
        
        logging.debug(f"Returning dataframes {standard_frame}, {move_energies}, {static_energies}")

        if self.eng_mode:
            # Movement Gate Sensitivities
            move_energies = [int(byte) for byte in ret[REF_MOVING_GATE_ENERGY_0:REF_MOVING_GATE_ENERGY_8+1]]
            # Static Gate Sensitivities
            static_energies = [int(byte) for byte in ret[REF_STATIC_GATE_ENERGY_0:REF_STATIC_GATE_ENERGY_8+1]]

        return standard_frame, move_energies, static_energies

    def poll_radar(self):
        while not self._stop_event.is_set():
            with self._lock:
                self.last_detection = self.get_radar_data()
            time.sleep(0.1)

    def start(self):
        logging.info("Radar polling started")
        self._worker_thread = threading.Thread(target=self.poll_radar)
        self._worker_thread.start()
        self._stop_event.clear()
        time.sleep(1) # Allow for a 1s init time

    def stop(self):
        if not self._stop_event.is_set():
            logging.info("Radar polling stopped")
            self._stop_event.set()
            self._worker_thread.join()
        else:
            logging.debug("Calling stop() but radar isn't running. This is normal.")


    def get_data(self):
        logging.debug(f"get_data returning {self.last_detection}")
        with self._lock:
            if not self.last_detection:
                logging.warning("Data is empty, have you started the radar yet?")
            return self.last_detection