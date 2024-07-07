#!/usr/bin/env python3
import serial
import time
import glob

def main():
    port_pattern = '/dev/tty.usbmodem*'
    port = find_serial_port(port_pattern)
    baudrate = 115200
    output_file = 'output.txt'
    read_from_serial(port, baudrate, output_file)
    write_to_serial( port, baudrate, output_file)

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
                        num1 = int(numbers[0])
                        num2 = int(numbers[1])
                        ser.write(f"{num1}, {num2}\n".encode('utf-8'))
                        print(f"Wrote: {num1}, {num2}")
                    except ValueError:
                        print(f"Invalid numbers: {line}")
                else:
                    print(f"Invalid line format: {line}")
                
                # Sleep
                time.sleep(0.01)
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
    main()

