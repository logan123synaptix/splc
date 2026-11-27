
import serial
import struct
import time


# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT = "COM35"       # USB-RS485 COM port
BAUDRATE = 9600
SLAVE_ID = 1

TIMEOUT = 1.5
READ_INTERVAL = 1.0

# Serial configuration: 9600, 8N1
BYTESIZE = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE


# ============================================================
# MODBUS CRC16
# ============================================================

def modbus_crc16(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


# ============================================================
# MODBUS RTU - READ INPUT REGISTERS (FC04)
# ============================================================

def read_input_registers(ser, slave_id, start_address, quantity):

    request = bytearray([
        slave_id,
        0x04,
        (start_address >> 8) & 0xFF,
        start_address & 0xFF,
        (quantity >> 8) & 0xFF,
        quantity & 0xFF
    ])

    crc = modbus_crc16(request)

    # Modbus CRC is transmitted low byte first
    request.append(crc & 0xFF)
    request.append((crc >> 8) & 0xFF)

    ser.reset_input_buffer()

    ser.write(request)
    ser.flush()

    expected_length = 5 + quantity * 2
    response = bytearray()

    start_time = time.monotonic()

    while len(response) < expected_length:

        remaining = expected_length - len(response)
        chunk = ser.read(remaining)

        if chunk:
            response.extend(chunk)

        if time.monotonic() - start_time > TIMEOUT:
            raise TimeoutError(
                f"Modbus timeout. Received "
                f"{len(response)}/{expected_length} bytes. "
                f"RX: {response.hex(' ')}"
            )

    # Check Slave ID
    if response[0] != slave_id:
        raise ValueError(
            f"Wrong Slave ID: {response[0]}"
        )

    # Check Modbus exception response
    if response[1] & 0x80:
        raise RuntimeError(
            f"Modbus exception code: 0x{response[2]:02X}"
        )

    # Check Function Code
    if response[1] != 0x04:
        raise ValueError(
            f"Wrong Function Code: 0x{response[1]:02X}"
        )

    # Check byte count
    if response[2] != quantity * 2:
        raise ValueError(
            f"Wrong byte count: {response[2]}"
        )

    # Verify CRC
    received_crc = response[-2] | (response[-1] << 8)
    calculated_crc = modbus_crc16(response[:-2])

    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC error: RX=0x{received_crc:04X}, "
            f"CALC=0x{calculated_crc:04X}"
        )

    # Extract 16-bit registers
    registers = []

    for i in range(3, 3 + quantity * 2, 2):
        value = (response[i] << 8) | response[i + 1]
        registers.append(value)

    return registers


# ============================================================
# DECODE FLOAT32 - WORD SWAPPED
# ============================================================

def decode_float_swapped(registers, index):

    low_word = registers[index]
    high_word = registers[index + 1]

    # Swap 16-bit word order
    raw_bytes = struct.pack(
        ">HH",
        high_word,
        low_word
    )

    return struct.unpack(">f", raw_bytes)[0]


# ============================================================
# READ SELEC EM2M-1P-C-100A-CE
# ============================================================

def read_meter(ser):

    # Register offset 14 -> 23
    # FC04, Quantity = 10 registers
    #
    # 14-15: Active Power       (P)
    # 16-17: Reactive Power     (Q)
    # 18-19: Apparent Power     (S)
    # 20-21: Voltage L-N        (U)
    # 22-23: Current            (I)

    registers = read_input_registers(
        ser=ser,
        slave_id=SLAVE_ID,
        start_address=14,
        quantity=10
    )

    data = {
        "P": decode_float_swapped(registers, 0),
        "Q": decode_float_swapped(registers, 2),
        "S": decode_float_swapped(registers, 4),
        "U": decode_float_swapped(registers, 6),
        "I": decode_float_swapped(registers, 8),
    }

    return data


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 65)
    print("       SELEC EM2M-1P-C-100A-CE MODBUS RTU READER")
    print("=" * 65)

    print(f"COM PORT : {SERIAL_PORT}")
    print(f"BAUDRATE : {BAUDRATE}")
    print(f"SLAVE ID : {SLAVE_ID}")
    print("FORMAT   : 8N1")
    print("FC       : 04 - READ INPUT REGISTERS")
    print("=" * 65)

    try:

        with serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=0.2
        ) as ser:

            print("\nUSB-RS485 connected.")
            print("Start reading meter...\n")

            print(
                f"{'P (kW)':>12} | "
                f"{'Q (kVAr)':>12} | "
                f"{'S (kVA)':>12} | "
                f"{'U (V)':>12} | "
                f"{'I (A)':>12}"
            )

            print("-" * 75)

            while True:

                try:

                    data = read_meter(ser)

                    print(
                        f"{data['P']:12.3f} | "
                        f"{data['Q']:12.3f} | "
                        f"{data['S']:12.3f} | "
                        f"{data['U']:12.2f} | "
                        f"{data['I']:12.3f}"
                    )

                except Exception as error:
                    print(f"Read error: {error}")

                time.sleep(READ_INTERVAL)

    except serial.SerialException as error:
        print(f"\nSerial port error: {error}")

    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
