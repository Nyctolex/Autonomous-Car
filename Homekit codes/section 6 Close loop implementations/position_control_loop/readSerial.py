import serial
import numpy as np
import matplotlib.pyplot as plt
import datetime
fig, ax = plt.subplots()
ax.set_ylim(0, 1024)
fig.show()

threshole = 10
start_time = datetime.datetime.now()
sensor_data = []
ser = serial.Serial('COM6', 10000, timeout=1)
while (datetime.datetime.now()-start_time).seconds < threshole:
    line = ser.readline()
    if line:
        string = line.decode().strip()
        sensor_value = int(string)
        sensor_data.append(sensor_value)
        ax.cla()
        ax.plot(sensor_data[-20:], color='blue')
        ax.set_ylim(0, 1024)
        fig.canvas.draw()
ser.close()