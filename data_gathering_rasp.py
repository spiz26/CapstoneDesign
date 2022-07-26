import serial

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flush()

file_name = "file_name.csv"

#columns names
f = open(file_name, mode="w")
head = "roll,pitch,yaw,ax,ay,az,gx,gy,gz\n"
f.write(head)

while True:
    f = open(file_name, mode='at')
    if ser.in_waiting > 0:
        
        line = ser.readline().decode('utf-8').rstrip()
        writedata = ("%s\n" %line)
        f.write (writedata)
        f.close()
        print(line)