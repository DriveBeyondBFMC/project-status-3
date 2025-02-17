import numpy as np
import math
import matplotlib.pyplot as plt
import time
import threading
import sys
import math
import socket
import serial
import tty
import termios
from scipy.interpolate import CubicSpline

condition = threading.Condition()
serial_port = '/dev/ttyACM0'
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)
message =""
# Parameters
k = 0.1 
Lfc = 0.1
Kp = 1.0 
dt = 0.1 
WB = 0.27

show_animation = True


x = 0
y = 0
cx = []
cy = []

steer_param = 0
speed_param = 0

cmd = {'steer': 90, 'speed': 0, 'parkingMode': 'False', 'avoidMode': 'False'}
host = "localhost"      

parkingPort = 8002
laneGuidePort = 8001


laneGuidanceSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
laneGuidanceSocket.connect((host, laneGuidePort))

parkingModeSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
parkingModeSocket.connect((host, parkingPort))

commands = {
    '0': '#kl:30;;\r\n',
    '1': '#battery:0;;\r\n',
    '2': '#instant:0;;\r\n',
    '3': '#imu:0;;\r\n',
    '4': '#resourceMonitor:0;;\r\n',
    '5': '#speed:{0};;\r\n',
    '6': '#steer:{0};;\r\n',
    '7': '#vcd:200;0;121;\r\n',
}


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

        print("x: ", self.x)

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a

def send_command(command):
    ser.write(command.encode())

def read_response():
    if ser.in_waiting > 0:  # Kiểm tra xem có dữ liệu trong buffer hay không
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(response)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())  # Chuyển sang chế độ raw
        ch = sys.stdin.read(1)  # Đọc một ký tự từ bàn phím
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Khôi phục cấu hình ban đầu
    return ch

def readMessage(socketer):
    global cmd
    while True:
        buffer = ""
        data = socketer.recv(1024).decode("utf-8")    
        buffer += data

        while "\n" in buffer:
            message, buffer = buffer.split("\n", 1)

        splitCmd = message.split("|")
        for unprocessedCmd in splitCmd:
            key, value = unprocessedCmd.split(": ")
            cmd.update({key: value})

def no_packing():
   run("no_packing")

def avoid_object():

    run("lane_switching")

def parking():
    run("parking")
    

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
            print(">>>")
            send_command(commands['6'].format(0))

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    global steer_param
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
        tx_ = trajectory.cx[ind-1]
        ty_ = trajectory.cy[ind-1]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    
    alpha_ = math.atan2(ty - ty_, tx - tx_)
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    theta_degrees_ = math.degrees(alpha_)

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    steer_param = (theta_degrees_ / 30) * 200
    print("steer_param", steer_param)
    if steer_param > 200: steer_param = 200
    if steer_param < -200: steer_param = -200
    send_command(commands['6'].format(steer_param))
    return delta, ind

def array_mode(option):
    global cx, cy
    if(option == "parking"):
        cx = (0, 7.04, 13.50, 15.6514, 20)
        cy = (0, 3.37, 0, 2.0774, 5)
    elif(option == "no_packing"):
        cx = (0, 2, 9.97, 13.0, 13.3)
        cy = (0, 0, -5.75, -4, -5)

    elif(option == "lane_switching"):
        cx = (0, 5.71, 12.43, 14.51, 15)
        cy = (0, -3.30, -1.49, 2.69, -2)

def operation_mode(target_ind, option):
    global speed_param, steer_param
    if(option == "no_packing"):
        if(target_ind == 2):
            speed_param = 100
            send_command(commands['5'].format(speed_param))
        elif(target_ind == 1):
            speed_param = -90
            send_command(commands['5'].format(speed_param))
        else:
            speed_param = 100
            send_command(commands['5'].format(speed_param))

    if(option == "parking"):
        if(target_ind == 3):
            print("run")
            speed_param = 100
            send_command(commands['5'].format(speed_param))
        else:
            speed_param = -90
            send_command(commands['5'].format(speed_param))

    if(option == "lane_switching"):
        speed_param = 120
        send_command(commands['5'].format(speed_param))





def run(option):
    global speed_param, steer_param
    array_mode(option)

    target_speed = 1.6 / 3.6  # [m/s]

    T = 1000.0 

    # initial state
    state = State(x=0.0, y=-0.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time_ = 0.0
    states = States()
    states.append(time_, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time_ and lastIndex > target_ind:
        with condition:
            ai = proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit_steer_control(
                state, target_course, target_ind)

            state.update(ai, di)  # Control vehicle

            time_ += dt
            states.append(time_, state)


            operation_mode(target_ind, option)


        if show_animation:  # pragma: no cover
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            # plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    print("Go to Goal!")
    speed_param = 0
    steer_param = 0
    send_command(commands['6'].format(steer_param))
    time.sleep(1)
    send_command(commands['5'].format(0))
    assert lastIndex >= target_ind, "Cannot goal"

def main():
    global speed_param, steer_param, cmd

    # In ra menu lựa chọn
    print("****************************")
    print("0: Enable full specification")
    print("1: Disable battery")
    print("2: Disable instant")
    print("3: Disable imu")
    print("4: Disable resource monitor")
    print("5: Control speed")
    print("6: Control steering")
    print("****************************")

    try:
        
        parkingMode = False; angle = 60; speed_param = 160; avoidMode = False
        while True:
            
            time.sleep(.05)
        
            parkingMode = False if cmd["parkingMode"] == "False" else True
            avoidMode = False if cmd["avoidMode"] == "False" else True
            angle = float(cmd['steer'])
            steer_param = int((((90 - angle) / 30) * 230) - 50)
            speed_param = float(cmd['speed'] )
            if steer_param > 230: steer_param = 230
            if steer_param < -230: steer_param = -230
            print(f"Angle: {(90 - angle):.2f}| Steering: {steer_param:.4f}| Speed: {speed_param}| Parking Mode: {parkingMode}| Avoid Mode: {avoidMode}")
            send_command(commands['6'].format(steer_param))
            send_command(commands['5'].format(speed_param))
            
            if avoidMode:
                avoid_object()
                
            if parkingMode == True:                 
                send_command(commands['5'].format(speed_param))
                time.sleep(.5)
                send_command(commands['5'].format(0))
                time.sleep(2)
                parking()
                time.sleep(2)
                no_packing()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        ...

if __name__ == '__main__':
    time.sleep(10)
    print("Pure pursuit path tracking simulation start")
    send_command(commands['0'])
    send_command(commands['1'])
    send_command(commands['2'])
    send_command(commands['4'])

    thread1 = threading.Thread(target = readMessage, args = (laneGuidanceSocket, ))
    thread2 = threading.Thread(target = readMessage, args = (parkingModeSocket, ))
    thread1.start()
    thread2.start()
    main()
    ser.close()

