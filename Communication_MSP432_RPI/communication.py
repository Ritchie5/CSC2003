import serial

# pi send & receive

ser = serial.Serial("/dev/ttyACM0", baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
try:
    while True:
        x = input("Enter a character: ")
        ser.write(bytes(x, 'utf-8'))
        print("sent " + x)

        if(ser.in_waiting > 0):
            line = ser.readline()
            print(line)
except KeyboardInterrupt:
        ser.close()