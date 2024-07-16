import numpy as np
import serial
import socket
import threading
import time
# Instant parameter
serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Set your baud rate

cmd = "RUN"
L = 0.4
vmax = 0.155# origibal 0.16

global leader
global follower
global v_l,v_r
# Semaphores for synchronization
leader_semaphore = threading.Semaphore()
follower_semaphore = threading.Semaphore()

# Function code
def limit_velocity(v_r,v_l):
    if v_r > vmax: 
        v_r = vmax
    elif v_r < -vmax:
        v_r = -vmax

    if v_l > vmax : 
        v_l = vmax
    elif v_l < -vmax:
        v_l = -vmax
    return v_r,v_l
def decoder_frame_data(data_received):
    split_index_CMD_start = data_received.index(':')
    split_index_CMD_stop = data_received.index('#', split_index_CMD_start + 1)

    split_index_X_start = data_received.index(':', split_index_CMD_stop + 1)
    split_index_X_stop = data_received.index('#', split_index_X_start + 1)

    split_index_Y_start = data_received.index(':', split_index_X_stop + 1)
    split_index_Y_stop = data_received.index('#', split_index_Y_start + 1)

    split_index_Theta_start = data_received.index(':', split_index_Y_stop + 1)
    split_index_Theta_stop = data_received.index('#', split_index_Theta_start + 1)

    cmd = data_received[split_index_CMD_start + 1:split_index_CMD_stop]
    X = float(data_received[split_index_X_start + 1:split_index_X_stop])
    Y = float(data_received[split_index_Y_start + 1:split_index_Y_stop])
    Theta = float(data_received[split_index_Theta_start + 1:split_index_Theta_stop])
    return cmd, X, Y, Theta

# PID class
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error 
        derivative = (error - self.prev_error) 
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# Robot class
class Robot:
    def __init__(self, cmd, x, y, theta):
        self.cmd = cmd
        self.x = x
        self.y = y
        self.theta = theta

# Initialization
leader = Robot("STP", 0, 0, 0)
follower = Robot("RUN", -L, 0, np.deg2rad(0))
v_r = 0
v_l = 0
# Initialize PID controllers
pid_vx = PID(15, 0.05, 0.4)
pid_vy = PID(15, 0.05 ,0.4)
pid_w = PID(15,0.01,0.3)
# Create socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.169.84', 12345))

# Create thread to read data from socket server
def read_leader_odom():
    global leader
    while True:
        data_received_socket = client_socket.recv(1024).decode().strip()
        leader_semaphore.acquire()
        leader.cmd, leader.x, leader.y, leader.theta = decoder_frame_data(data_received_socket)
        leader_semaphore.release()

thread_read_leader = threading.Thread(target=read_leader_odom)
thread_read_leader.start()

# Create and open serial port 
ser = serial.Serial(serial_port, baud_rate)

# Create thread to read data from MCU
def read_follower_odom():
    global follower
    while True:
        data_received_mcu = ser.readline().decode('utf-8').strip()
        follower_semaphore.acquire()
        follower.cmd, follower.x, follower.y, follower.theta = decoder_frame_data(data_received_mcu)
        follower_semaphore.release()

thread_read_follower = threading.Thread(target=read_follower_odom)
thread_read_follower.start()


def send_data_to_MCU():
    global v_r,v_l
    while True:
        data_send2MCU = "!cmd:{}#v_r:{:0.2f}#v_l:{:0.2f}#\n".format("RUN", v_r, v_l)
        ser.write(data_send2MCU.encode())
        time.sleep(0.1)

thread_send_dataMCU = threading.Thread(target=send_data_to_MCU)
thread_send_dataMCU.start()

try: 
    # Infinite loop
    while True:
        # global leader,follower,v_r,v_l
        # Print data

        # leader_semaphore.acquire()
        # data_leader = "leader: x:{}, y:{}, theta:{}\n".format(leader.x, leader.y, leader.theta)
        # leader_semaphore.release()
        # print(data_leader)

        follower_semaphore.acquire()
        data_follower = "follower: x:{}, y:{}, theta:{}\n".format(follower.x, follower.y, follower.theta)
        follower_semaphore.release()
        print(data_follower)

        # Calculate desired point 
        leader_semaphore.acquire()
        x_d = leader.x - L * np.cos(np.deg2rad(leader.theta))
        y_d = leader.y - L * np.sin(np.deg2rad(leader.theta))
        theta_d = leader.theta
        leader_semaphore.release()
        # Calculate error ex and ey
        follower_semaphore.acquire()
        x_error = x_d - follower.x
        y_error = y_d - follower.y
        theta_error = np.deg2rad(theta_d) - np.deg2rad(follower.theta)
        follower_semaphore.release()
        # Calculate PID output vx and vy 
        vx = pid_vx.update(x_error)
        vy = pid_vy.update(y_error)
        w_adjust = pid_w.update(theta_error)
        # Calculate v and w (omega)
        follower_semaphore.acquire()
        v = vx * np.cos(np.deg2rad(follower.theta)) + vy * np.sin(np.deg2rad(follower.theta))
        w =   1 * (-vx * np.sin(np.deg2rad(follower.theta)) + vy * np.cos(np.deg2rad(follower.theta))) / 0.0325
        # w = (-vx * np.sin(np.deg2rad(follower.theta)) + vy * np.cos(np.deg2rad(follower.theta))) 
        # print("vx:{:0.2f},vy:{:0.2f},v:{:0.2f},w:{:0.2f}".format(vx,vy,v,w))

        follower_semaphore.release()

        # Calculate vr and vl
        v_r = v + (w * 0.2 / 2)
        v_l = v - (w * 0.2 / 2)
        v_r, v_l = limit_velocity(v_r, v_l)

        # print("velocity: !cmd:{}#v_r:{}#v_l:{}".format(follower.cmd, v_r, v_l))
        # Stop condition
        leader_semaphore.acquire()
        follower_semaphore.acquire()

        # if(np.sqrt((x_d - follower.x)**2 + (y_d - follower.y)**2) <= 0.005) and (leader.theta -follower.x)<=10:
        #     v_r = v_l = 0
        if np.sqrt((leader.x - follower.x)**2 + (leader.y - follower.y)**2) <= 0.385:
            v_r  = (w_adjust * 0.2)/2
            v_l  = -(w_adjust*0.2)/2
            if(np.deg2rad(leader.theta) - np.deg2rad(follower.theta)) <=0.05: # 0.1745
                v_r = v_l = 0

        if(np.sqrt((x_d - follower.x)**2 + (y_d - follower.y)**2) <= 0.05):
            v_r  = (w_adjust * 0.2)/2
            v_l  = -(w_adjust*0.2)/2
            if(np.deg2rad(leader.theta) - np.deg2rad(follower.theta)) <=0.05: #0.1745
                v_r = v_l = 0
            
            


        leader_semaphore.release()
        follower_semaphore.release()
        time.sleep(0.1)


except KeyboardInterrupt:
    client_socket.close()
    ser.close()
    thread_read_leader.join()
    thread_read_follower.join()
    send_data_to_MCU.join()
