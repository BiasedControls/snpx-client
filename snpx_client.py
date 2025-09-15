import socket
import struct
import time
import math

# Packet values should be kept in hex value for debugging with Wireshark

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
        command = BASE_MESSAGE.copy()

        command[2] = count & 0xFF
        command[3] = (count >> 8) & 0xFF
        command[30] = count & 0xFF 
        command[31] = 0xC0
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
        command = BASE_MESSAGE.copy()

        count = len(value)
        if count == 0:
            return

        if count <= 48:
            command[9], command[17], command[31] = 0x01, 0x01, 0xC0
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


class PositionData:
    def __init__(self, socket, code, address: int):
        self.code = code
        self.address = address
        self.socket = socket
        self._snpx_seq = 0

        # Sends a string command "SETASG". Should add a functionality for writing any command to the robot so we can clean this and the init message up
        SETASG_PACKET = bytes.fromhex("""
        02 00 03 00
        19 00 00 00 00 02 00 00 00 00 00 00 00 02 00 00
        00 00 00 00 00 00 00 00 00 00 03 80 00 00 00 00
        10 0e 00 00 01 01 00 00 00 00 00 00 01 01 07 38
        00 00 19 00 53 45 54 41 53 47 20 31 20 35 30 20
        50 4f 53 5b 47 31 3a 30 5d 20 30 2e 30
        """)

        self.socket.sendall(SETASG_PACKET)

        # Step 2: Wait for ACK
        ack = self.socket.recv(1024)
        if not ack:
            raise RuntimeError("No ACK received after SETASG")

    def read(self):
        """
        Read joints from robot
        Should probably make this more dynamic / smarter
        """

        READ_PACKET = bytes.fromhex("""
        02 00 04 00
        00 00 00 00 00 01 00 00 00 00 00 00 00 01 00 00
        00 00 00 00 00 00 00 00 00 00 04 c0 00 00 00 00
        10 0e 00 00 01 01 04 08 00 00 32 00 00 00 00 00
        00 00 00 00
        """)

        # Request Joints
        self.socket.sendall(READ_PACKET)
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

        if connect_on_init:
            self.connect()

        self.di = DigitalSignal(socket=self.socket, code=0x48, address=0)
        self.do = DigitalSignal(socket=self.socket, code=0x46, address=0)
        self.ui = DigitalSignal(socket=self.socket, code=0x48, address=6000)
        self.uo = DigitalSignal(socket=self.socket, code=0x46, address=6000)
        self.cart_pos = PositionData(socket=self.socket, code=0x08, address=12000)
        self.j_pos = PositionData(socket=self.socket, code=0x08, address=12026)


    def connect(self):
        """
        Sends packets to the robot controller to initialize the connection, clear past assignments, and define protocols
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
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

    def disconnect(self):
        try:
            self.socket.close()
        except Exception as e:
            print(f"Failed to close socket listening on {self.ip}:{self.port} - {e}")


BASE_MESSAGE = [
    0x02, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x00, 0x00, 
    0x01, 0x01, 0x04, 0x46, 0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

]
