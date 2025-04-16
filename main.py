import serial
import asyncio
import time

#ser = serial.Serial(
#    port="/dev/ttyUSB0",
#    baudrate=9600,
    #parity=serial.PARITY_ODD,
    #stopbits=serial.STOPBITS_TWO,
    #bytesize=serial.SEVENBITS
#)

def decode_distance_packet(packet):
    """Decode distance measurement packet"""
    if len(packet) < 14:
        return None

    # Distance is typically in the first 16-bit value (little-endian)
    distance_mm = (packet[5] << 8) | packet[6]

    # Checksum verification
    calculated_csum = sum(packet[:-1]) & 0xFF
    valid = packet[-1] == calculated_csum

    return {
        'distance_mm': distance_mm,
        'raw_values': [
            (packet[5] << 8) | packet[6],  # Primary distance
            (packet[7] << 8) | packet[8],  # Often secondary/confirmation
            (packet[9] << 8) | packet[10]  # Usually diagnostic
        ],
        'checksum': f"{packet[-1]:02X}",
        'valid': valid
    }

async def monitor_sensor(port: str, baudrate: int = 9600):
    with serial.Serial(port, baudrate, timeout=1) as ser:
        buffer = bytearray()

        while True:
            buffer.extend(ser.read(ser.in_waiting))
            read_distance: str = ""
            # Process complete packets (14 bytes)
            while len(buffer) >= 14:
                # Find packet start (AA 0F 10)
                start_idx = next((i for i in range(len(buffer)-2)
                                if buffer[i] == 0xAA
                                and buffer[i+1] == 0x0F
                                and buffer[i+2] == 0x10), -1)

                if start_idx == -1:
                    buffer.clear()
                    break

                if len(buffer) - start_idx >= 14:
                    packet = buffer[start_idx:start_idx+14]
                    decoded = decode_distance_packet(packet)
                    read_distance = decoded["distance_mm"]

                    buffer = buffer[start_idx+14:]
                else:
                    break

            print(f"{port}: {read_distance}")
            await asyncio.sleep(.5)


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.create_task(monitor_sensor("/dev/ttyUSB0"))
    loop.create_task(monitor_sensor("/dev/ttyUSB1"))

    loop.run_forever()
