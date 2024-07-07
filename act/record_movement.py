#!/usr/bin/env python3
import serial
import time
import glob
import argparse
import matplotlib.pyplot as plt
import datetime

def main(play, file, plot):
    port_pattern = '/dev/tty.usbmodem*'
    port = find_serial_port(port_pattern)
    baudrate = 115200
    if not file:
        file = 'output.txt'
    if not play:
        read_from_serial(port, baudrate, file)
    write_to_serial(port, baudrate, file, plot)

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

def write_to_serial(port, baudrate, input_file, plot):
    ser = None

    # Initialize lists for plotting
    timestamps = []
    num1_data = []
    num2_data = []

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
                        # Get numbers
                        num1 = int(numbers[0])
                        num2 = int(numbers[1])
                        
                        # Write packet
                        byte_array = bytearray([0x61, 0x1, 0x1, num1//10, num2//10, 0x0, 0x0, 0x0])
                        ser.write(byte_array)
                        print(f"Wrote: {num1}, {num2}")

                        # Append data for plotting
                        timestamp = datetime.datetime.now()
                        timestamps.append(timestamp)
                        num1_data.append(num1)
                        num2_data.append(700 - num2)

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

        # Plot if requested
        if plot:
            plt.figure()
            plt.plot(timestamps, num1_data, label='Gripper', marker='X')
            plt.plot(timestamps, num2_data, label='Elbow', marker='.')
            plt.xlabel('Time')
            plt.ylabel('Value')
            plt.title('Plot')
            plt.legend()
            plt.grid(True)
            plt.show()

def find_serial_port(pattern):
    ports = glob.glob(pattern)
    if ports:
        return ports[0]
    else:
        raise IOError("No serial ports found matching the pattern")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial port read/write utility")
    parser.add_argument('--play', action='store_true', help="Just play back input file")
    parser.add_argument('--file', type=str,            help="Input file to write to the serial port")
    parser.add_argument('--plot', action='store_true', help="Plot Num1 and Num2 over time")
    args = parser.parse_args()
    main(args.play, args.file, args.plot)
