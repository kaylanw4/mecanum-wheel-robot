#!/usr/bin/env python3
import serial
import time

def test_serial_connection(port='/dev/ttyUSB0', baudrate=115200):
    """Test basic serial communication with STM32 board"""
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud")
        
        # Give the board time to initialize
        time.sleep(2)
        
        # Send a test command (check if board responds)
        # Most STM32 boards respond to simple queries
        test_commands = [
            b'\r\n',  # Newline might trigger a response
            b'?\r\n',  # Common query command
            b'help\r\n',  # Help command
        ]
        
        for cmd in test_commands:
            print(f"\nSending: {cmd}")
            ser.write(cmd)
            time.sleep(0.1)
            
            # Read response
            if ser.in_waiting:
                response = ser.read(ser.in_waiting)
                print(f"Received: {response}")
        
        # Close connection
        ser.close()
        print("\nSerial test completed")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Update this with your actual serial port
    test_serial_connection('/dev/ttyACM0', 115200)