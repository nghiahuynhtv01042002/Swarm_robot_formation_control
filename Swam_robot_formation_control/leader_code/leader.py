import serial
import time
import math
import obstacles as ob
import numpy as np
import socket



def decoder_frame_data( data_received):

    split_index_CMD_start = data_received.index(':')
    split_index_CMD_stop = data_received.index('#',split_index_CMD_start + 1)

    split_index_X_start = data_received.index(':',split_index_CMD_stop+1)
    split_index_X_stop = data_received.index('#',split_index_X_start+1)

    split_index_Y_start = data_received.index(':',split_index_X_stop+1)
    split_index_Y_stop = data_received.index('#',split_index_Y_start+1)

    split_index_Theta_start = data_received.index(':',split_index_Y_stop+1)
    split_index_Theta_stop = data_received.index('#',split_index_Theta_start+1)

    cmd = data_received[split_index_CMD_start+1:split_index_CMD_stop]
    X = float(data_received[split_index_X_start+1:split_index_X_stop])
    Y = float(data_received[split_index_Y_start+1:split_index_Y_stop])
    Theta = float(data_received[split_index_Theta_start+1:split_index_Theta_stop])   
    return cmd,X,Y,Theta

def decoder_frame_vel(data_received):
    split_index_CMD_start = data_received.index(':')
    split_index_CMD_stop = data_received.index('#',split_index_CMD_start + 1)
    split_index_vr_start = data_received.index(':',split_index_CMD_stop+1)
    split_index_vr_stop = data_received.index('#',split_index_vr_start+1)
    split_index_vl_start = data_received.index(':',split_index_vr_stop+1)
    split_index_vl_stop = data_received.index('#',split_index_vl_start+1)

    cmd = data_received[split_index_CMD_start+1:split_index_CMD_stop]
    vr = float(data_received[split_index_vr_start+1 : split_index_vr_stop])
    vl = float(data_received[split_index_vl_start+1 : split_index_vl_stop])
    return cmd ,vr, vl 

def decoder_frame_enc(data_received):
    split_index_CMD_start = data_received.index(':')
    split_index_CMD_stop = data_received.index('#',split_index_CMD_start + 1)
    split_index_enc_r_start = data_received.index(':',split_index_CMD_stop+1)
    split_index_enc_r_stop = data_received.index('#',split_index_enc_r_start+1)
    split_index_enc_l_start = data_received.index(':',split_index_enc_r_stop+1)
    split_index_enc_l_stop = data_received.index('#',split_index_enc_l_start+1)

    cmd = data_received[split_index_CMD_start+1:split_index_CMD_stop]
    enc_r = float(data_received[split_index_enc_r_start+1 : split_index_enc_r_stop])
    enc_l = float(data_received[split_index_enc_l_start+1 : split_index_enc_l_stop])
    return cmd ,enc_r, enc_l 

# def send_point(x, y, theta):
#     sending_data = "!cmd:RUN#x:{}#y:{}#theta:{}#\n".format(x, y, theta)
#     ser.write(sending_data.encode())
#     print(sending_data)

# def send_destiny(*points):
#     if not points:
#         return
#     temp_point = points.pop(0);
#     send_point(temp_point)

def send_point(points,cmd):
    # global cmd
    if not points:
        return 
    if cmd == "STP":
        # time.sleep(10)
        temp = points.pop(0)

        sending_data = "!cmd:RUN#x:{}#y:{}#theta:0.00#\n".format(temp[0],temp[1])
        ser.write(sending_data.encode())
        print("data send to MCU: "+sending_data)
        print("################################################ points was sent ##############################33#############################3\n")
        # cmd = "RUN"

#PSO here

def random_initialization(swarm_size):
    """
    Random initializations of PSO particles location and velocity.

    Parameters:
        swarm_size (int): How many particles are present.

    Returns:
        List[List[float, float]]: List of starting coordinates of the particles, total number = swarm_size * DIM * 2.
        List[float, float]: List of starting velocity of the particles, total number = swarm_size * DIM * 2.
        List[floats]: Each particle best loss function result.
        List[List[float, float]]: List of best velocity of the particles (there's only the starting in the initialization).
        float: Best loss function result.
        List[float, float]: The best location.
    """

    # set the location and velocity of the particle's
    # particles_loc = np.random.rand(swarm_size, DIM, 2) * 20
    particles_loc_x = np.random.rand(swarm_size,DIM)*20
    particles_loc_y = np.random.rand(swarm_size,DIM)*10
    particles_loc = np.stack((particles_loc_x,particles_loc_y),axis= -1)

    particles_vel = np.random.rand(swarm_size, DIM, 2)

    # set the initial particle best location and value
    particles_lowest_loss = [
        loss_function(particles_loc[i, :, :]) for i in range(0, len(particles_loc))
    ]
    particles_best_location = np.copy(particles_loc)

    # set the initial global best location and value
    global_lowest_loss = np.min(particles_lowest_loss)
    global_best_location = particles_loc[np.argmin(particles_lowest_loss)].copy()

    return (
        particles_loc,
        particles_vel,
        particles_lowest_loss,
        particles_best_location,
        global_lowest_loss,
        global_best_location,
    )


def loss_function(x):
    """
    Loss function to find the shortest path.

    Parameters:
        x (List[Point]): List of points representing the path of the agent.

    Returns:
        int: The loss function result
    """

    z = (x[0, 0] - START.x) ** 2 + (x[0, 1] - START.y) ** 2
    for i in range(DIM - 1):
        z = z + ((x[i, 0] - x[i + 1, 0]) ** 2 + (x[i, 1] - x[i + 1, 1]) ** 2)
    z = z + (x[DIM - 1, 0] - END.x) ** 2 + (x[DIM - 1, 1] - END.y) ** 2
    return math.sqrt(z)



# def loss_function(x):
#     """
#     Loss function to find the shortest path including the maximum distance between consecutive points.

#     Parameters:
#         x (List[Point]): List of points representing the path of the agent.

#     Returns:
#         float: The loss function result
#     """

#     # Initialize the loss with the distance from the start to the first point
#     z = (x[0, 0] - START.x) ** 2 + (x[0, 1] - START.y) ** 2
#     distance_Start2End = math.sqrt((START.x - END.x)**2 + (END.y - START.y)**2)
#     # Initialize the variable to track the maximum distance
#     max_distance = 0
#     penaty_distance = 0
#     # Calculate the sum of squared distances between consecutive points and track the maximum distance
#     for i in range(DIM - 1):
#         distance = (x[i, 0] - x[i + 1, 0]) ** 2 + (x[i, 1] - x[i + 1, 1]) ** 2
#         z += distance
#         if distance > (distance_Start2End)/(DIM+1):
#             # max_distance = distance
#             penaty_distance +=1
        
    
#     # Add the distance from the last point to the end
#     z += (x[DIM - 1, 0] - END.x) ** 2 + (x[DIM - 1, 1] - END.y) ** 2
#     z += 1*z
#     # Add the maximum distance to the loss
#     z += penaty_distance*1
    
#     return math.sqrt(z)






def is_valid(circles, p):
    """
    Check if the point p is valid (i.e. doesn't go into obstacles).

    Parameters:
        circles (List[Obstacle_Circle]): The list of obstacles.
        p (Point): The point to check.

    Returns:
        boolean: wheter the point is valid
    """
    to_add = ob.Point(0, 0)
    point_p = ob.Point(p[0], p[1])
    for i in range(len(circles)):
        if circles[i].inside_circle(point_p):
            to_add = ob.Point(
                circles[i].how_to_exit_x(point_p.x) + to_add.x,
                circles[i].how_to_exit_y(point_p.y) + to_add.y,
            )
    return to_add
        
def particle_swarm_optimization(
    max_iterations, swarm_size, max_vel, step_size, inertia, c1, c2, circles
):
    """
    Implementation of the Particle Swarm Optimization algorithm randomly.

    Parameters:
        max_iterations (int): The maximum number of iterations.
        swarm_size (int): The number of particles.
        max_vel (int): Maximum velocity for a particle.
        step_size (int): The step size for updating each particle (how far a particle travels before its velocity is readjusted).
        inertia (int): The inertia of a particle.
        c1 (int): Factor for the particles local best.
        c2 (int): Factor for the the global best.
        circles (Obstacle_Circle): Obstacle of circular shape.

    Returns:
        boolean: the best path
    """
    (
        particles_loc,
        particles_vel,
        particles_lowest_loss,
        particles_best_location,
        global_lowest_loss,
        global_best_location,
    ) = random_initialization(swarm_size)
    best_location = []
    print_iteration = 1

    for iteration_i in range(max_iterations):
        if iteration_i % 20 == 0:
            print("%i%%" % int(iteration_i / max_iterations * 100))
            print_iteration += 1
        for particle_i in range(swarm_size):
            dim_i = 0
            while dim_i < DIM:
                # update the velocity vector in a given dimension
                error_particle_best = (
                    particles_best_location[particle_i, dim_i]
                    - particles_loc[particle_i, dim_i]
                )
                error_global_best = (
                    global_best_location[dim_i] - particles_loc[particle_i, dim_i]
                )
                new_vel = (
                    inertia
                    * min(1, (dim_i ** (0.5)) / 4)
                    * particles_vel[particle_i, dim_i]
                    + c1 * np.random.rand(1) * error_particle_best
                    + c2 * np.random.rand(1) * error_global_best
                )

                # bound a particle's velocity to the maximum value
                if new_vel[0] < -max_vel:
                    new_vel[0] = -max_vel
                elif new_vel[0] > max_vel:
                    new_vel[0] = max_vel
                if new_vel[1] < -max_vel:
                    new_vel[1] = -max_vel
                elif new_vel[1] > max_vel:
                    new_vel[1] = max_vel

                # update the particle location and velocity
                particles_loc[particle_i, dim_i] = (
                    particles_loc[particle_i, dim_i] + new_vel[:] * step_size
                )
                particles_vel[particle_i, dim_i] = new_vel[:]

                particle_help = is_valid(circles, particles_loc[particle_i, dim_i, :])
                particles_loc[particle_i, dim_i, 0] += particle_help.x
                particles_loc[particle_i, dim_i, 1] += particle_help.y
                if abs(particle_help.x) > 0.0 or abs(particle_help.y) > 0.0:
                    dim_i -= 1
                dim_i += 1
            # for the new location, check if this is a new local or global best (if it's valid)
            particle_error = loss_function(particles_loc[particle_i, :])
            if particle_error < particles_lowest_loss[particle_i]:  # local best
                particles_lowest_loss[particle_i] = particle_error
                particles_best_location[particle_i, :] = particles_loc[particle_i,:].copy()
            if particle_error < global_lowest_loss:  # global best
                global_lowest_loss = particle_error
                global_best_location = particles_loc[particle_i, :].copy()

        best_location = global_best_location.copy()
    return best_location




try:
    DIM = 9
    # DIM = 
    # START = ob.Point(1, 1)
    # END = ob.Point(10, 20)
    START = ob.Point(0,0)
    END = ob.Point(20,8)

    #test case 1 
    # obstacle1 = ob.Obstacle_Circle(1.0, ob.Point(8, 4))
    # obstacle2 = ob.Obstacle_Circle(1.0, ob.Point(10, 2))  
    # obstacle3 = ob.Obstacle_Circle(1.0, ob.Point(10, 6))
    # obstacle4 = ob.Obstacle_Circle(1.0, ob.Point(12, 4))
    
    ##test case 2
    obstacle1 = ob.Obstacle_Circle(1.0, ob.Point(4, 8))  
    obstacle2 = ob.Obstacle_Circle(1.0, ob.Point(10, 4))
    obstacle3 = ob.Obstacle_Circle(1.0, ob.Point(12, 2))
    obstacle4 = ob.Obstacle_Circle(1.0, ob.Point(16, 9))

    ##test case 3
    # obstacle1 = ob.Obstacle_Circle(1.0, ob.Point(2, 8))
    # obstacle2 = ob.Obstacle_Circle(1.0, ob.Point(6, 1))  
    # obstacle3 = ob.Obstacle_Circle(1.0, ob.Point(8, 4))
    # obstacle4 = ob.Obstacle_Circle(1.0, ob.Point(10, 1))


    obstacles = [obstacle1, obstacle2, obstacle3,obstacle4]

    best_location = []
    path = []
    print("Finding best path with PSO:")
    print("Loading:")
    best_location = particle_swarm_optimization(
        max_iterations=100,
        swarm_size=200,
        max_vel=3,
        step_size=1,
        inertia=0.9,
        c1=2.05,
        c2=2.05,
        circles=obstacles,
    )
    print("100% completed!")
    ##add start point và end point 
    # path.append([START.x,START.y])

    for loc in best_location:
        path.append([loc[0], loc[1]])
    path.append([END.x,END.y])
    for i in range(len(path)):
        path[i][0] /= 10
        path[i][1] /= 10
    print("best path:{}".format(path))

    #create socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.169.84', 12345))
    server_socket.listen(5)
    print("Server is listening...")
    # accept conection from client
    client_socket, addr = server_socket.accept()
    print("Connection from {} has been established.".format(addr))
    #open serial port
    ser = serial.Serial('/dev/ttyUSB0', 115200)  
    print("Start Moving!!!!!!!!!!\n")
    while True:
        data_received = ser.readline().decode('utf-8').strip()
        # print(data_received+"\n")
        #send data to client
        data_received += "\n"
        client_socket.sendall(data_received.encode())
        print("Data sent from server: "+ data_received)
        cmd,X,Y,Theta = decoder_frame_data(data_received)
        # cmd,enc_r, enc_l =decoder_frame_enc(data_received)
        send_point(path,cmd)
        # print(f"cmd: {cmd}\nx: {X}\ny: {Y}\ntheta: {Theta}\n")
        # cmd,vr,vl = decoder_frame_vel(data_received);
        # print("cmd: {}\nvr: {}\nvl: {}\n".format(cmd,vr,vl))
               
except KeyboardInterrupt:
    ser.close()
    server_socket.close()