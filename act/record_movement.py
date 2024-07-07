#!/usr/bin/env python3
import serial
import time
import glob
import argparse

def main(play, file):
    port_pattern = '/dev/tty.usbmodem*'
    port = find_serial_port(port_pattern)
    baudrate = 115200
    if not file:
        file = 'output.txt'
    if not play:
        read_from_serial(port, baudrate, file)
    write_to_serial( port, baudrate, file)

def read_from_serial(port, baudrate, output_file):
    ser = None
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baudrate.")
        
        # Open the output file
        with open(output_file, 'w') as file:
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    numbers = line.split(',')
                    
                    # Check if the line contains exactly two numbers
                    if len(numbers) == 2:
                        try:
                            num1 = int(numbers[0])
                            num2 = int(numbers[1])
                            file.write(f"{num1}, {num2}\n")
                            print(f"Wrote: {num1}, {num2}")
                        except ValueError:
                            print(f"Invalid numbers: {line}")
                    else:
                        print(f"Invalid line format: {line}")

                # Sleep 
                time.sleep(0.005)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        if ser is not None:
            ser.close()
            print("Serial port closed.")

def write_to_serial(port, baudrate, input_file):
    ser = None
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baudrate.")
        
        # Read the input file
        with open(input_file, 'r') as file:
            for line in file:
                line = line.strip()
                numbers = line.split(',')
                
                # Check if the line contains exactly two numbers
                if len(numbers) == 2:
                    try:
                        num1 = int(numbers[0])//10
                        num2 = int(numbers[1])//10

                        # Write packet
                        byte_array = bytearray([0x61, 0x1, 0x1, num1, num2, 0x0, 0x0, 0x0])
                        ser.write(byte_array)
                        #print(f"Wrote: {byte_array}")
                        print(f"Wrote: {num1*10}, {num2*10}")
                    except ValueError:
                        print(f"Invalid numbers: {line}")
                else:
                    print(f"Invalid line format: {line}")
                
                # Sleep
                time.sleep(0.005)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except IOError as e:
        print(f"I/O error: {e}")
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        if ser is not None:
            ser.close()
            print("Serial port closed.")

def find_serial_port(pattern):
    ports = glob.glob(pattern)
    if ports:
        return ports[0]
    else:
        raise IOError("No serial ports found matching the pattern")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial port read/write utility")
    parser.add_argument('--play', action='store_true', help="Just play back input file")
    parser.add_argument('--file', type=str, help="Input file to write to the serial port")
    args = parser.parse_args()
    main(args.play, args.file)
