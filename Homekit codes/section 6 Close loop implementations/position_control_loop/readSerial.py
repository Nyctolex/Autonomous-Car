import serial
import numpy as np
import matplotlib.pyplot as plt
import datetime

class Plotter:
    def __init__(self, number_of_points_in_frame:int, timeout=None, stop_after_n_points:int = 0) -> None:
        self.number_of_points_in_frame = number_of_points_in_frame
        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(0, 1024)
        self.fig.show()
        self.lines: dict[str, list[float]] = {}
        self.timeout = timeout
        self.stop_after_n_points = stop_after_n_points
        self.start_time = datetime.datetime.now()
        self.ser = serial.Serial('COM6', 115200, timeout=1)
        if (self.timeout is None) and (self.stop_after_n_points == 0):
            self.stop_after_n_points = self.number_of_points_in_frame
        self.point_counter = 0
        
    def check_running_conditions(self):
        timeout_condition = False
        point_condition = False
        if self.timeout:
            timeout_condition = (datetime.datetime.now()-self.start_time).seconds < self.timeout
        if self.stop_after_n_points:
            point_condition = self.stop_after_n_points > self.point_counter
        return timeout_condition or point_condition
    
    def run(self):
        while self.check_running_conditions():
            line = self.ser.readline()
            if line:
                string = line.decode().strip()
                self.point_counter += 1
                new_values = Plotter.packet_to_values_dict(string)
                self.append_lines(new_values)
                self.clear_garbage_lines()
                self.render_canvas()
        self.ser.close()
        
    def render_canvas(self):
        self.ax.cla()
        for title, line in self.lines.items():
            self.ax.plot(line[-self.number_of_points_in_frame:], label=title)
        self.ax.legend()
        self.fig.canvas.draw()
    
    @staticmethod
    def packet_to_values_dict(packet: str, sep:str='\t') ->dict[str, float]:
        """
        Gets the a packet from the arduino and return a dictionary where the key is the name of the plot and the value is the new value
        """
        headers = packet.strip().split('\t')
        values_dict = {}
        for header in headers:
            if ":" in header:
                name, value = header.split(":")
                values_dict[name] = float(value)
        return values_dict

    def append_lines(self, values_dict:dict[str, float])-> None:
        """
        Append the new values given in the value_dict to the lists in lines
        """
        for key, value in values_dict.items():
            if key in self.lines:
                self.lines[key].append(value)
            else:
                self.lines[key] = [value]
                
    def clear_garbage_lines(self)->None:
        if not len(self.lines.values()):
            return
        longest_list = max([len(line) for line in self.lines.values()])
        line_to_remove = []
        for name, line in self.lines.items():
            if len(line) < longest_list - 5:
                line_to_remove.append(name)
        for line in line_to_remove:
            self.lines.pop(line)
        
    


def main():
    plotter = Plotter(1000)
    plotter.run()
    input()
  
if __name__ == "__main__":
    main()
