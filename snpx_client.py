import socket
import struct
import time
import math
from enum import Enum, auto
from dataclasses import dataclass

# Packet values should be kept in hex value for debugging with Wireshark
@dataclass
class FanucVariable():
    size : int # size in bytes
    multiply : int  # multiply - typically only used for INTs

class VariableTypes:
    INT = FanucVariable(size=2, multiply=1)
    REAL = FanucVariable(size=2, multiply=0)
    STRING = FanucVariable(size=80, multiply=0)

class DigitalSignal:
    """
    Robot digital signal
    """

    def __init__(self, socket, code, address: int):
        self.socket = socket
        self.code = code
        self.address = address

    @staticmethod
    def _decode_digital_outputs(response, requested_bits) -> list[bool]:
        """
        Helper for decoding boolean values from robot byte responses
        """
        if not response:
            print("No response received for digital outputs")
            return []

        if isinstance(response, bytes):
            response = response.hex()

        clean_hex = response.replace(" ", "").replace(":", "")
        data_hex = clean_hex[112:] if len(clean_hex) > 112 else clean_hex[88:-4]

        if len(data_hex) < 2:
            print("Response too short for digital output read")
            return []

        bool_list = []
        bits_decoded = 0

        for i in range(0, len(data_hex), 2):
            if bits_decoded >= requested_bits:
                break

            try:
                byte_val = int(data_hex[i:i+2], 16)
                for bit in range(8):
                    if bits_decoded >= requested_bits:
                        break
                    bool_list.append(bool((byte_val >> bit) & 1))
                    bits_decoded += 1
            except ValueError:
                for _ in range(8):
                    if bits_decoded >= requested_bits:
                        break
                    bool_list.append(False)
                    bits_decoded += 1

        return bool_list

    def read(self, count: int, start_index : int = 1) -> list[bool]:
        """
        Read a list of bools from the robot's IO
        """
        
        start_index = self.address + start_index - 1
        command = BASE_MSG.copy()

        command[2] = count & 0xFF
        command[3] = (count >> 8) & 0xFF
        command[30] = count & 0xFF 
        command[43] = self.code
        command[44] = start_index & 0xFF
        command[45] = (start_index >> 8) & 0xFF

        byte_allocation = ((count + 7) // 8) * 8
        command[46] = byte_allocation & 0xFF
        command[47] = (byte_allocation >> 8) & 0xFF

        self.socket.send(bytearray(command))
        resp = self.socket.recv(1024)
        return self._decode_digital_outputs(resp.hex(), count)


    def write(self, value : list[bool], start_index: int = 1):
        """
        Write a list of boolean values to a digital signal in the robot starting at the specified index
        """
        start_index = self.address + start_index - 1
        command = BASE_MSG.copy()

        count = len(value)
        if count == 0:
            return

        if count <= 48:
            command[9], command[17] = 0x01, 0x01
        else:
            command[9], command[17], command[31] = 0x02, 0x02, 0x80

        command[2] = count & 0xFF
        command[3] = (count >> 8) & 0xFF
        command[30] = count & 0xFF
        command[42], command[43] = 0x07, self.code
        command[44] = start_index & 0xFF
        command[45] = (start_index >> 8) & 0xFF

        byte_allocation = ((count + 7) // 8) * 8
        command[46] = byte_allocation & 0xFF
        command[47] = (byte_allocation >> 8) & 0xFF

        if count > 48:
            command[42:42] = [0x00] * 6
            command[48:48] = [0x01, 0x01]
            command = command[:-8]

        byte_count = math.ceil(count / 8)
        payload = bytearray(byte_count)
        for i in range(byte_count):
            byte = 0
            for j in range(8):
                bit_index = i * 8 + j
                if bit_index < count and value[bit_index]:
                    byte |= (1 << j)
            if i < len(payload):
                payload[i] = byte

        if count > 48:
            command.extend(payload)
            command[4] = len(payload)
        else:
            command[48:48] = payload
            command = command[:-len(payload)]

        self.socket.send(bytearray(command))

        # Cleanup after to remove garbage values
        _ = self.socket.recv(1024)


class PositionData:
    def __init__(self, socket, code, address: int):
        self.code = code
        self.address = address
        self.socket = socket
        self._snpx_seq = 0

    def read(self):
        """
        Read joints from robot
        Should probably make this more dynamic / smarter
        """

        # build packet
        command = BASE_MSG.copy()
        command[2] = 0x04
        command[30] = 0x04 
        command[43] = 0x08 # memory type code
        command[46] = 0x32 # size in bytes

        # Request Joints
        self.socket.sendall(bytearray(command))
        response = self.socket.recv(2048)

        # Trim response down to the joint values
        float_data = response[108:-24]

        # Decode as 32-bit little-endian floats
        joint_values = []
        for i in range(0, len(float_data), 4):
            chunk = float_data[i:i+4]
            if len(chunk) < 4:
                break
            val = struct.unpack("<f", chunk)[0]
            joint_values.append(val)

        return joint_values


class SnpxClient:
    def __init__(self, ip : str = "127.0.0.1", port: int = 60008, connect_on_init : bool = False):
        self.ip = ip
        self.port = port
        self._snpx_seq = 0
        self.socket = None
        self._sys_vars = {}

        if connect_on_init:
            self.connect()
        
    def init_signals(self):
        """
        Initialize signal / memory objects. This is required every time the socket is modified
        """
        self.di = DigitalSignal(socket=self.socket, code=0x48, address=0)
        self.do = DigitalSignal(socket=self.socket, code=0x46, address=0)
        self.ui = DigitalSignal(socket=self.socket, code=0x48, address=6000)
        self.uo = DigitalSignal(socket=self.socket, code=0x46, address=6000)
        self.so = DigitalSignal(socket=self.socket, code=0x46, address=7000) # SOP output
        self.si = DigitalSignal(socket=self.socket, code=0x48, address=7000) # SOP input
        self.cart_pos = PositionData(socket=self.socket, code=0x08, address=12000)
        self.j_pos = PositionData(socket=self.socket, code=0x08, address=12026)
        
        # --- Set any default assignments ---
        # Position variable
        self.set_asg("POS[G1:0] 0.0", FanucVariable(size=50, multiply=0), 1)


    def connect(self):
        """
        Sends packets to the robot controller to initialize the connection, clear past assignments, and define protocols
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        self.socket.settimeout(5.0)
        time.sleep(0.1)

        # Empty initialization
        # Send init message (empty byte array )
        self.socket.send(b'\x00' * 56)
        if self.socket.recv(64)[0] != 1:
            raise Exception("Failed SNPX init")

        # Send init messages, first for protocol, second to clear previous assignments
        for msg in [
            "08:00:01:00:00:00:00:00:00:01:00:00:00:00:00:00:00:01:00:00:00:00:00:00:00:00:00:00:00:00:01:c0:00:00:00:00:10:0e:00:00:01:01:4f:01:00:00:00:00:00:00:00:00:00:00:00:00",
            "02:00:02:00:00:00:00:00:00:01:00:00:00:00:00:00:00:01:00:00:00:00:00:00:00:00:00:00:00:00:02:c0:00:00:00:00:10:0e:00:00:01:01:07:38:00:00:06:00:43:4c:52:41:53:47:00:00"
        ]:
            self.socket.send(bytearray.fromhex(msg.replace(':', '')))
            self.socket.recv(1024)

        self.init_signals()

    def disconnect(self):
        try:
            self.socket.close()
        except Exception as e:
            print(f"Failed to close socket listening on {self.ip}:{self.port} - {e}")

    @staticmethod
    def _recv_snpx_packet(sock) -> bytes:
        """
        Receive a complete SNPX packet from the robot.
        SNPX packets are at least 56 bytes. 
        Max packet size is 81 bytes
        """
        # read 56-byte packet
        packet = bytearray()
        packet = sock.recv(1024)
        
        # Return payload if packet length is 56
        if len(packet) <= 56:
            return bytes(packet)

        # extract payload if packet is longer
        # packet length can be found in bytes 
        count_field = int.from_bytes(packet[46:48], "little")
        payload_len = count_field
        if payload_len < 0:
            payload_len = 0

        # read payload
        payload = bytearray()
        while len(payload) < payload_len:
            chunk = sock.recv(payload_len - len(payload))
            if not chunk or len(chunk) < 1:
                raise ConnectionError("Socket closed before payload complete")
            payload.extend(chunk)

        return bytes(payload)
    

    def check_if_asg_avail(self, num: int, size: int = 1) -> bool:
        """
        Check if an assignment number range is available.
        Returns True if the assignment number range [num, num + size - 1] is available,
        False if any part of the range overlaps with existing variables.
        
        :param num: Starting assignment number to check
        :param size: Size in words (16-bit registers) that need to be available
        """
        # Check if num is within valid range
        if num < 1 or num > 80:
            return False
        
        # Check if the range extends beyond valid range
        if num + size - 1 > 80:
            return False
        
        # Check for overlap with existing variables
        vals = self._sys_vars.values()
        for v in vals:
            # guard in case stored value isn't the expected dict
            try:
                existing_index = v.get("index")
                existing_size = v.get("size", 1)
                
                # Check if the requested range overlaps with this variable's range
                if num < existing_index + existing_size and num + size > existing_index:
                    return False
            except Exception:
                continue
        return True

    def get_next_asg_num(self, size: int = 1) -> int:
        """
        Get the next available assignment number that can accommodate the given size.
        Considers the size of existing variables to find non-overlapping ranges.
        
        :param size: Size in words (16-bit registers) needed for the variable
        :returns: The next available assignment number, or None if no space available
        """
        # Check each possible starting position
        for start_num in range(1, 81):
            if self.check_if_asg_avail(start_num, size):
                return start_num
        
        # No available space found
        return None

    def set_asg(self, var_name: str, var_type: FanucVariable, asg_num: int = None):
        """
        Sets a variable assignment using the SETASG command.

        Variables given will be assigned to the $ASG_NUM at the asg_num specified.
        If no asg_num is given, it will find the next available num 
        """
        # Return if the variable has already been added
        if self._sys_vars.get(var_name) != None:
            return

        # Check assignment number
        if asg_num == None:
            asg_num = self.get_next_asg_num(var_type.size)
            if asg_num is None:
                raise ValueError("No available assignment number found (all 80 slots may be occupied)")
        elif not self.check_if_asg_avail(asg_num, var_type.size):
            raise ValueError(f"Assignment number {asg_num} is not available for size {var_type.size}")
        
        # Verify asg number
        if asg_num < 1 or asg_num > 80:
            raise ValueError("Assignment index out of range (must be 1-80)")

        command_string = f"SETASG {asg_num} {var_type.size} {var_name} {var_type.multiply}"
        
        # Convert string to ASCII bytes
        payload = command_string.encode('ascii')
        payload_len = len(payload)

        # Build command packet
        command = BASE_MSG.copy()
        command[2] = 0x03
        command[4] = len(command_string)
        command[9] = 0x02
        command[17] = 0x00
        command[30] = 0x03
        command[31] = 0x80
        command[42] = 0x00
        command[43] = 0x00
        command[48] = 0x01
        command[49] = 0x01
        command[50] = 0x07
        command[51] = 0x38
        command[54] = payload_len & 0xFF
        command[55] = (payload_len >> 8) & 0xFF
        command.extend(payload)

        # Send
        self.socket.sendall(bytearray(command))

        # Add to dict so we don't set asg again if not needed
        self._sys_vars[var_name] = {
            "size" : var_type.size,
            "multiply": var_type.multiply,
            "index": asg_num
        }
        
        #print(self._sys_vars)

        # Receive Acknowledge (clear buffer)
        response = self.socket.recv(1024)
        return response

    def read_sys_var(self, var_name: str, var_type: FanucVariable):
        """
        Read system variable by first assigning it to an %R register, then reading the register.
        
        :param var_name: Name of variable, usually starts with "$"
        :param var_type: Type of variable, must be one of VariableTypes
        :returns: The decoded value (float, int, or str)
        """

        # 1. Make sure variable is assigned in robot (and retrieve the assignment number/type info)
        self.set_asg(var_name=var_name, var_type=var_type)
        
        var_info = self._sys_vars.get(var_name)
        if var_info is None:
             raise Exception(f"Failed to assign variable {var_name}")

        asg_num = var_info["index"]
        size = var_info["size"] # Number of registers (words) to read
        multiply = var_info["multiply"]
        
        # %R register address is 0-based word address. For %R1, the address is 0 (0x0000).
        start_word_address = asg_num - 1
        
        # 2. Construct the Read Request packet
        command = BASE_MSG.copy()

        # Update word count (bytes 2-3 and 30-31)
        count = size * 2 # Number of 16-bit words (registers)
        
        command[2] = count & 0xFF
        command[3] = (count >> 8) & 0xFF
        command[30] = count & 0xFF 

        # Update Read Command fields (bytes 42-47)
        command[42] = 0x04 # Command: Read (0x04)
        command[43] = 0x08 # Memory Area: %R (Register)
        command[44] = start_word_address & 0xFF
        command[45] = (start_word_address >> 8) & 0xFF
        
        # Length of data in bytes (size * 2 bytes/word)
        payload_len = size #* 2 #int(size / 2)
        command[46] = payload_len & 0xFF
        command[47] = (payload_len >> 8) & 0xFF

        # 3. Send the packet and receive the response
        self.socket.send(bytearray(command))
        payload = SnpxClient._recv_snpx_packet(self.socket)
        #print_bytes_with_index(payload)

        # Extract data payload
        data_start = 44
        data_end = data_start + (size * 2)
        data_bytes = payload[data_start : data_end]
        
        if len(payload) < 4:
             # The received data is less than expected
             # In a real network setup, you might need to try a different offset
             raise ValueError(f"Received only {len(data_bytes)} bytes of data for an expected {payload_len} bytes.")

        # Decode the payload
        value = None

        if var_type == VariableTypes.REAL:
            # REAL (4 bytes / 2 registers) -> 32-bit little-endian float (<f)
            value = struct.unpack("<f", data_bytes)[0]
        
        elif var_type == VariableTypes.INT:
            # INT (4 bytes / 2 registers) -> 32-bit little-endian signed integer (<i)
            raw_value = struct.unpack("<i", data_bytes)[0]
            # Apply scaling if the user intended it
            if multiply != 0:
                value = raw_value / multiply
            else:
                value = raw_value

        elif var_type == VariableTypes.STRING:
            # STRING (160 bytes / 80 registers) -> ASCII string
            value = data_bytes.decode('ascii').strip('\x00')
            
        else:
             raise ValueError("Unsupported or incomplete variable type/data received.")

        return value
        
    def write_sys_var(self, var_name: str, var_type: FanucVariable, value):
        """
        Write system variable by first assigning it to an %R register, then writing to the register.
        
        :param var_name: Name of variable, usually starts with "$"
        :param var_type: Type of variable, must be one of VariableTypes
        :returns: The decoded value (float, int, or str)
        """

        # 1. Make sure variable is assigned in robot (and retrieve the assignment number/type info)
        self.set_asg(var_name=var_name, var_type=var_type)
        
        var_info = self._sys_vars.get(var_name)
        if var_info is None:
             raise Exception(f"Failed to assign variable {var_name}")

        asg_num = var_info["index"]
        size = var_info["size"] # Number of registers (words) to read
        multiply = var_info["multiply"]
        
        # %R register address is 0-based word address. For %R1, the address is 0 (0x0000).
        start_word_address = asg_num - 1
        
        # 2. Construct the Read Request packet
        command = BASE_MSG.copy()

        # Update word count (bytes 2-3 and 30-31)
        count = size * 2 # Number of 16-bit words (registers)
        
        command[2] = count & 0xFF
        command[3] = (count >> 8) & 0xFF
        command[30] = count & 0xFF 

        # Update Read Command fields (bytes 42-47)
        command[42] = 0x07 # Command: Read (0x04)
        command[43] = 0x08 # Memory Area: %R (Register)
        command[44] = start_word_address & 0xFF
        command[45] = (start_word_address >> 8) & 0xFF
        
        # Length of data in bytes (size * 2 bytes/word)
        payload_len = size #* 2 #int(size / 2)
        command[46] = payload_len & 0xFF
        command[47] = (payload_len >> 8) & 0xFF
        
        # determine how many bytes the variable occupies (size is in 16-bit words)
        byte_len = var_type.size * 2

        # build payload bytes (little-endian to match your read logic)
        if var_type == VariableTypes.REAL:
            # 32-bit IEEE float
            payload = struct.pack('<f', float(value))
        elif var_type == VariableTypes.INT:
            # apply multiply scaling if used when reading (inverse for writing)
            if var_type.multiply and var_type.multiply != 0:
                raw = int(value * var_type.multiply)
            else:
                raw = int(value)
            payload = struct.pack('<i', raw)    # 4-byte signed int little-endian
        elif var_type == VariableTypes.STRING:
            payload = str(value).encode('ascii', errors='replace')[:byte_len]
            if len(payload) < byte_len:
                payload = payload + b'\x00' * (byte_len - len(payload))
        else:
            raise ValueError("Unsupported variable type for write")

        # ensure payload is exactly byte_len
        if len(payload) < byte_len:
            payload = payload + b'\x00' * (byte_len - len(payload))
        elif len(payload) > byte_len:
            payload = payload[:byte_len]

        # Place payload into command starting at byte index 48 (one byte per entry)
        # command is a list of ints so iterating payload (bytes) yields ints 0-255
        for i, b in enumerate(payload):
            command[48 + i] = b

        # 3. Send the packet and receive the response
        self.socket.sendall(bytearray(command))

        # Cleanup socket
        _ = self.socket.recv(1024)

 
def print_bytes_with_index(bytearr : bytes):
    for i in range(0, len(bytearr)):
        print(f"[{i}] - {bytearr[i]}")


BASE_MSG = [
    0x02,        # 00 - Type (03 is Return, 02 is Transmit)
    0x00,        # 01 - Reserved/Unknown
    0x00,        # 02 - Seq Number - FILL AT RUNTIME
    0x00,        # 03 - Reserved/Unknown
    0x00,        # 04 - Text Length - FILL AT RUNTIME ???
    0x00,        # 05 - Reserved/Unknown
    0x00,        # 06 - Reserved/Unknown
    0x00,        # 07 - Reserved/Unknown
    0x00,        # 08 - Reserved/Unknown
    0x01,        # 09 - Reserved/Unknown*
    0x00,        # 10 - Reserved/Unknown
    0x00,        # 11 - Reserved/Unknown
    0x00,        # 12 - Reserved/Unknown
    0x00,        # 13 - Reserved/Unknown
    0x00,        # 14 - Reserved/Unknown
    0x00,        # 15 - Reserved/Unknown
    0x00,        # 16 - Reserved/Unknown
    0x01,        # 17 - Reserved/Unknown*
    0x00,        # 18 - Reserved/Unknown
    0x00,        # 19 - Reserved/Unknown
    0x00,        # 20 - Reserved/Unknown
    0x00,        # 21 - Reserved/Unknown
    0x00,        # 22 - Reserved/Unknown
    0x00,        # 23 - Reserved/Unknown
    0x00,        # 24 - Reserved/Unknown
    0x00,        # 25 - Reserved/Unknown
    0x00,        # 26 - Time Seconds - FILL AT RUNTIME
    0x00,        # 27 - Time Minutes - FILL AT RUNTIME
    0x00,        # 28 - Time Hours   - FILL AT RUNTIME
    0x00,        # 29 - Reserved/Unknown
    0x00,        # 30 - Size ?????
    0xc0,        # 31 - Message Type
    0x00,        # 32 - Mailbox Source
    0x00,        # 33 - Mailbox Source
    0x00,        # 34 - Mailbox Source
    0x00,        # 35 - Mailbox Source
    0x10,        # 36 - Mailbox Destination
    0x0e,        # 37 - Mailbox Destination
    0x00,        # 38 - Mailbox Destination
    0x00,        # 39 - Mailbox Destination
    0x01,        # 40 - Packet Number
    0x01,        # 41 - Total Packet Number
    0x04,        # 42 - Service Request Code - (Operation Type SERVICE_REQUEST_CODE)
    0x46,        # 43 - Request Dependent Space (For Reading: set MEMORY_TYPE_CODE)
    0x00,        # 44 - Request Dependent Space (For Reading: set to Address - 1)(LSB)
    0x00,        # 45 - Request Dependent Space (For Reading: set to Address - 1)(MSB)
    0x00,        # 46 - Request Dependent Space (For Reading: Data Size Bytes)(LSB)
    0x00,        # 47 - Request Dependent Space (For Reading: Data Size Bytes)(MSB)
    0x00,        # 48 - Reserved/Unknown
    0x00,        # 49 - Reserved/Unknown
    0x00,        # 50 - Reserved/Unknown
    0x00,        # 51 - Reserved/Unknown
    0x00,        # 52 - Reserved/Unknown
    0x00,        # 53 - Reserved/Unknown
    0x00,        # 54 - Reserved/Unknown
    0x00         # 55 - Reserved/Unknown
]

# --- TODO keeping for future implementation. Would make it easier to build commands  ---
# --- TODO should change these to a dataclass instead of a dict. Would make it easier to use ---
SERVICE_REQUEST_CODE = {
    "PLC_STATUS"             : b'\x00',
    "RETURN_PROG_NAME"       : b'\x03',
    "READ_SYS_MEMORY"        : b'\x04',    # Used to read general memory register (Example: %R12344)
    "READ_TASK_MEMORY"       : b'\x05',
    "READ_PROG_MEMORY"       : b'\x06',
    "WRITE_SYS_MEMORY"       : b'\x07',
    "WRITE_TASK_MEMORY"      : b'\x08',
    "WRITE_PROG_MEMORY"      : b'\x09',
    "RETURN_DATETIME"        : b'\x25',
    "RETURN_CONTROLLER_TYPE" : b'\x43'
}


# Used at byte locaiton 43
MEMORY_TYPE_CODE = {
    "R"  :   b'\x08',    # Register (Word)
    "AI" :   b'\x0a',    # Analog Input (Word)
    "AQ" :   b'\x0c',    # Analog Output (Word)
    "I"  :   b'\x10',    # Descrete Input (Byte)
    "Q"  :   b'\x12',    # Descrete Output (Byte)
}

