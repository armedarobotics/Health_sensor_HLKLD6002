import serial
import struct
import time

# Command to know which ports are available:
# python3 -m serial.tools.list_ports
# /dev/ttyUSB0        
# 1 ports found


class HLK_LD6002_Decoder:
    """
    Decodes data frames from the HLK-LD6002 radar module via serial port.
    """
    def __init__(self, port, baudrate=1382400, timeout=0.1):
        """
        Initializes the serial connection.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
        # --- Frame Constants ---
        self.SOF = 0x01 # Start of Frame [cite: 18, 21, 26, 29]
        self.FRAME_MIN_LEN = 1 + 2 + 2 + 2 + 1 + 1 # SOF + ID + LEN + TYPE + HEAD_CKSUM + DATA_CKSUM
        
        # Message Type IDs
        self.TYPE_PHASE = 0x0A13    # Total/Breath/Heart phase [cite: 16]
        self.TYPE_BREATH_RATE = 0x0A14 # Breath rate [cite: 19]
        self.TYPE_HEART_RATE = 0x0A15 # Heartbeat rate [cite: 22]
        self.TYPE_DISTANCE = 0x0A16  # Detection target distance [cite: 27]

    def connect(self):
        """
        Opens the serial port.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            self.ser = None

    def close(self):
        """
        Closes the serial port.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")

    def _calculate_checksum(self, data):
        """
        Calculates the 8-bit XOR checksum (ret = ~ret) for a sequence of bytes.
        This is an implementation of the provided C code logic. [cite: 47, 48, 51, 52, 54, 56]
        """
        ret = 0
        for byte in data:
            ret ^= byte
        # Invert the final result
        return (~ret) & 0xFF

    def _le_bytes_to_float(self, data_bytes):
        """
        Converts 4 bytes (little-endian, uint32) to an IEEE 754 single-precision float.
        The TF frame Data bit is little-endian order. 
        """
        # The bytes are already in little-endian order from the data frame 
        # Example: 0x66,0x66,0xA2,0x41 -> 0x41A26666
        # struct.unpack('<f', ...) handles the conversion from little-endian bytes to float.
        return struct.unpack('<f', data_bytes)[0]
    
    def _le_bytes_to_uint32(self, data_bytes):
        """
        Converts 4 bytes (little-endian) to an unsigned 32-bit integer (uint32).
        """
        return struct.unpack('<I', data_bytes)[0]

    def decode_frame(self):
        """
        Reads data from the serial port and decodes one complete TF frame.
        Returns a dictionary with the decoded data or None on failure/no data.
        """
        if not self.ser or not self.ser.is_open:
            return None

        
        # 1. Synchronize to SOF (Start of Frame)
        byte = self.ser.read(1)


        while byte and byte[0] != self.SOF: #CHecks whether byte is the first byte of Frame
            byte = self.ser.read(1)
        
        if not byte:
            return None # No SOF found within timeout

        # Start capturing the frame data from the SOF (which is 1 byte)
        frame_start = byte 

        # 2. Read the rest of the header and length (ID, LEN, TYPE, HEAD_CKSUM)
        # ID (2), LEN (2), TYPE (2), HEAD_CKSUM (1) = 7 bytes
        header_tail = self.ser.read(7)
        if len(header_tail) < 7:
            return None # Incomplete header
        
        # The full header for CKSUM is SOF + ID + LEN + TYPE
        header_for_cksum = frame_start + header_tail[:6]
        
        
        # Verify HEAD_CKSUM
        expected_head_cksum = header_tail[6]
        calculated_head_cksum = self._calculate_checksum(header_for_cksum)
        
        if expected_head_cksum != calculated_head_cksum:
            # print(f"Head Checksum Error! Expected: {expected_head_cksum}, Got: {calculated_head_cksum}")
            return None # Checksum failed, discard frame

        # Extract information from header
        # ID is bytes 1-2, LEN is bytes 3-4, TYPE is bytes 5-6 (little-endian)
        # ID is always 0x0000 in examples [cite: 18, 21, 26, 29]
        # struct.unpack('<H', ...) reads 2 bytes as little-endian unsigned short (uint16)
        data_len = struct.unpack('>H', header_tail[2:4])[0]
        frame_type = struct.unpack('>H', header_tail[4:6])[0]

        # 3. Read DATA and DATA_CKSUM
        # Total data length is data_len (Data bytes) + 1 (DATA_CKSUM byte)
        data_and_cksum = self.ser.read(data_len + 1)

        if len(data_and_cksum) < data_len + 1:
            return None # Incomplete data
        
        data_payload = data_and_cksum[:-1]
        expected_data_cksum = data_and_cksum[-1]
        
        # The DATA_CKSUM covers all DATA bytes plus the DATA_CKSUM byte itself 
        # In the provided C code, the CKSUM is calculated over `data` of length `len`, 
        # where `data` points to the first byte of DATA. 
        # The CKSUM result is placed in the DATA_CKSUM field.
        # Let's assume the CKSUM is calculated only over the DATA payload *before* the CKSUM byte, 
        # which is the standard interpretation for checksums protecting a data block.
        # The source text is ambiguous ("The first byte of DATA to the last byte of the DATA CKSUM bit") 
        # but the C function `getCksum(unsigned char *data, unsigned char len)` is usually 
        # called with `data` being the payload and `len` being its length.
        
        # **Using the standard interpretation: Checksum over Data Payload only**
        calculated_data_cksum = self._calculate_checksum(data_payload)

        if expected_data_cksum != calculated_data_cksum:
            # print(f"Data Checksum Error! Expected: {expected_data_cksum}, Got: {calculated_data_cksum}")
            return None # Checksum failed, discard frame

        # 4. Decode the data payload based on TYPE
        decoded_data = {
            'type_id': hex(frame_type),
            'data_len': data_len
        }
        
        try:
            if frame_type == self.TYPE_PHASE:
                # 4 bytes [total phase] float, 4 bytes [breath phase] float, 4 bytes [heart phase] float
                # Total 12 bytes. LEN should be 0x000C or 12. Examples show LEN as 00 04, which is contradictory. 
                # Assuming the table structure is correct and expecting 12 bytes of data.
                # Since the table structure in the source implies 3 * 4-byte floats, we'll try to read 12 bytes.
                if len(data_payload) >= 12:
                    decoded_data['Total_Phase'] = self._le_bytes_to_float(data_payload[0:4])
                    decoded_data['Breath_Phase'] = self._le_bytes_to_float(data_payload[4:8])
                    decoded_data['Heart_Phase'] = self._le_bytes_to_float(data_payload[8:12])
                    decoded_data['description'] = "Phase Test Result (Total/Breath/Heart)"
                else:
                    decoded_data['error'] = "Payload length mismatch for TYPE_PHASE"
            
            elif frame_type == self.TYPE_BREATH_RATE:
                # 4 bytes [rate] float
                if len(data_payload) == 4:
                    decoded_data['Breath_Rate_bpm'] = self._le_bytes_to_float(data_payload)
                    decoded_data['description'] = "Breath Rate Test Result"
                else:
                    decoded_data['error'] = "Payload length mismatch for TYPE_BREATH_RATE"

            elif frame_type == self.TYPE_HEART_RATE:
                # 4 bytes [rate] float
                if len(data_payload) == 4:
                    decoded_data['Heart_Rate_bpm'] = self._le_bytes_to_float(data_payload)
                    decoded_data['description'] = "Heartbeat Rate Test Result"
                else:
                    decoded_data['error'] = "Payload length mismatch for TYPE_HEART_RATE"

            elif frame_type == self.TYPE_DISTANCE:
                # 4 bytes [flag] Uint32, 4 bytes [range] float
                if len(data_payload) == 8:
                    flag = self._le_bytes_to_uint32(data_payload[0:4])
                    range_cm = self._le_bytes_to_float(data_payload[4:8])
                    decoded_data['Flag'] = flag
                    decoded_data['Range_cm'] = range_cm
                    decoded_data['description'] = "Detection Target Distance"
                    if flag == 1:
                        decoded_data['Note'] = "Distance output (unit: cm)" [cite: 30]
                    elif flag == 0:
                        decoded_data['Note'] = "No distance output" [cite: 31]
                    else:
                        decoded_data['Note'] = "Unknown flag value"
                else:
                    decoded_data['error'] = "Payload length mismatch for TYPE_DISTANCE"
            
            else:
                decoded_data['description'] = "Unknown/Unimplemented Frame Type"
                decoded_data['raw_data'] = data_payload.hex()

        except Exception as e:
            # print(f"Error during data conversion: {e}")
            decoded_data['error'] = f"Conversion error: {e}"
        
        return decoded_data


# --- Example Usage ---
def main():
    # ⚠️ IMPORTANT: Replace 'COM_PORT_NAME' with your actual serial port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
    COM_PORT = '/dev/ttyUSB0' 
    
    decoder = HLK_LD6002_Decoder(COM_PORT)
    decoder.connect()

    if not decoder.ser or not decoder.ser.is_open:
        print("Exiting.")
        return

    print("\n--- Listening for HLK-LD6002 Data ---")
    print("Press Ctrl+C to stop.")
    
    try:
        start_time = time.time()
        while time.time() - start_time < 30: # Run for 30 seconds
            data = decoder.decode_frame()
            if data:

                print("-" * 30)
                print(f"Time: {time.strftime('%H:%M:%S')}")
                # Print all key-value pairs in the dictionary
                for key, value in data.items():
                    # Format float values for better readability
                    if isinstance(value, float):
                        print(f"{key}: {value:.3f}")
                    else:
                        print(f"{key}: {value}")
            
    except KeyboardInterrupt:
        print("\nStopping data collection.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        
    finally:
        decoder.close()

if __name__ == "__main__":
    main()
    #print("Note: To run this code, you must replace 'COM_PORT_NAME' with your actual serial port name.")
