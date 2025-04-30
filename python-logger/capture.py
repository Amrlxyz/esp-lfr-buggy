from serial.tools import list_ports
from subprocess import call
import serial

import sys
import os
import datetime
import csv
import atexit

serial_port = "COM8"
end_command = b"S"
start_command = b"D"
max_points = 1000
baud = 115200

log_filename: str
data_labels = [
    'time_index',
    'set_point', 
    'measurement', 
    # 'error', 
    'proportional', 
    'integrator', 
    'differentiator', 
    # 'output'
]


def main():
    get_available_ports()
    serialcom = select_port(serial_port)

    setup_log_file()
    print(f"log_file: {log_filename} \n")

    write_first_line()
    serialcom.write(start_command)
    line_read = [False]
    atexit.register(end_function, serialcom, line_read)

    while(True):
        values = get_line(serialcom)
        if values:
            line_read[0] = True
            write_line(values)
    
    

def get_available_ports():
    print("Available Ports:")
    for port in list_ports.comports():
        print(port)
    print(f"Chosen Port: {serial_port}")


def select_port(selected_port):
    try:
        serialCom = serial.Serial(selected_port, baudrate=baud, timeout=0)
        return serialCom
    except serial.serialutil.SerialException:
        print(f"{serial_port} Is invalid")
        exit()


def setup_log_file():
    # Create a folder if it doesn't exist
    folder_name = "logs"
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Get current date and time
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

    run_desc = "default"
    if len(sys.argv) >= 2:
        run_desc = ""
        for desc in sys.argv[1:]:
            run_desc += desc

    # Concatenate folder name and filename
    global log_filename
    log_filename = os.path.join(folder_name, f"data_{run_desc}_{formatted_datetime}.csv")
    

def write_first_line():
    with open("data.csv", "w", newline="") as csvfile1, open(log_filename, "w", newline="") as csvfile2:
        writer1 = csv.writer(csvfile1, delimiter=",")
        writer2 = csv.writer(csvfile2, delimiter=",")
        writer1.writerow(data_labels)
        writer2.writerow(data_labels)
        
    
def get_line(serial_comm):
    prev_data = ""
    try:
        while(True):
            line_bytes = serial_comm.readline()
            if not line_bytes:
                continue
            curr_data = prev_data + str(line_bytes.decode("utf-8"))
            if '\n' in curr_data:
                break
            else:
                prev_data = curr_data
        line = curr_data.strip()
        values = line.split(",")
        print(values)
        if len(values) == len(data_labels):
            print(values)
            return values
        else:
            print(line, "<- Data Incomplete and Not Stored")
            return False
        
    except Exception as e:
        print(f"Error: Line Not Read / {e}")


def write_line(val):
    with open("data.csv", "a", newline="") as csvfile1, open(log_filename, "a", newline="") as csvfile2:       
        writer1 = csv.writer(csvfile1, delimiter=",")
        writer2 = csv.writer(csvfile2, delimiter=",")
        writer1.writerow(val)
        writer2.writerow(val)


def end_function(ser, line_read):
    if not line_read[0]:
        os.remove(log_filename)
        print(f"Deleted '{log_filename}' due to 0 data written")
    ser.write(end_command)


if __name__ == "__main__":
    main()