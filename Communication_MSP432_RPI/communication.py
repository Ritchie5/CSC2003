import serial
from time import sleep

# open serial port
try:
    serial_port = serial.Serial("/dev/ttyACM1", baudrate=9600, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
except serial.SerialException as e:
    print("could not open serial port '{}': {}".format("/dev/ttyACM1", e))

def serial_com():
    serial_port.flush()
    try:
        while True:
            x = input("Enter a character: ")
            # write to serial port
            serial_port.write(bytes(x, 'utf-8'))

            line = serial_port.read()  # read serial port
            sleep(0.03)
            data_left = serial_port.inWaiting()  # check for remaining byte
            line += serial_port.read(data_left)
            print(line.decode("utf-8"))  # print received data
    except KeyboardInterrupt:
        # close the serial port
        serial_port.close()

serial_com()
