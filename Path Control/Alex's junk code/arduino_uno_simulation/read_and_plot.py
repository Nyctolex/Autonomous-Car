import pandas as pd
import matplotlib.pyplot as plt
import serial

def is_point(string):
    for char in string:
        if not (char.isdigit() or char == '.' or char == ',' or char == '-'):
            return False
    return True

def readserial(comport, baudrate):
    max_itterations = 600
    itteration = 0
    ser = serial.Serial(comport, baudrate, timeout=1)         # 1/timeout is the frequency at which the port is read
    connection_initiated = False
    done = False
    x_path = []
    y_path = []
    to_point = lambda str_point: [float(x) for x in str_point.split(',')]
    
    ser.write('run'.encode())
    while itteration < max_itterations:
        try:
            ser.write('run\r\n'.encode())
            data = ser.readline().decode().strip()
            if data:
                # print("test", data)
                if not connection_initiated and data == "running":
                    connection_initiated = True
                    print("Connection initiated")
                elif connection_initiated and data == "done":
                    done = True
                    print("is done")
                    break
                elif not is_point(data):
                    print(data)
                    
                elif connection_initiated:
                    x, y= to_point(data)
                    x_path.append(x)
                    y_path.append(y)
                    print(data, itteration)
                    itteration = itteration+1
                 
        except:
            pass
    ser.close()
    plt.plot(x_path, y_path)
    plt.show()
    


if __name__ == '__main__':

    readserial('COM3', 9600)