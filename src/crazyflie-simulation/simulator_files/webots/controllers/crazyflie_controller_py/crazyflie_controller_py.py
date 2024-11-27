#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from controller import Supervisor
# mrr
from controller import Display

import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse  # Import the Empty service type

from math import cos, sin
import random
import math

import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1

def ros_init():
    rospy.init_node('controller_node', anonymous=True)
    reload_world_service_server()
    rospy.Subscriber("desired", String, desired_callback)

forward_desired = 0
sideways_desired = 0
yaw_desired = 0
states = []
def desired_callback(data):
    global forward_desired, sideways_desired, yaw_desired, states
    obj = json.loads(data.data)
    forward_desired = float(obj['forward_desired'])
    sideways_desired = float(obj['sideways_desired'])
    yaw_desired = float(obj['yaw_desired'])
    states_str = str(obj['states'])
    states = list(map(float, states_str.split()))
    # rospy.loginfo(rospy.get_caller_id() + "action= %s", data.data)
    # print(states)

def publish_string_message(str_message):
    string_publisher = rospy.Publisher('string_message', String, queue_size=10)
    message = String()
    message.data = str_message
    string_publisher.publish(message)

def handle_reload_world_service(req):
    global surpervisor, is_reset
    # surpervisor.worldReload()
    # print("simulation is resetting!")
    surpervisor.simulationResetPhysics()
    is_reset = True
    response = EmptyResponse()
    return response

def reload_world_service_server():
    service = rospy.Service('reload_world_service', Empty, handle_reload_world_service)


def clear_plot(plot_display, plot_width, plot_height):
    global states
    robot_i = 0
    robot_j = 4
    grid_size = 8
    grid_value = 4

    # Clear the plot display with a white background
    plot_display.setColor(0xFFFFFF)  # White
    plot_display.fillRectangle(0, 0, plot_width, plot_height)

    # Draw the robot's initial position as a blue square
    # plot_display.setColor(0x0000FF)  # Blue
    # plot_display.fillRectangle(robot_j, robot_i, 1, 1)

    # # Draw a line from the robot's initial position to a new position
    # new_i = robot_i + int(0.2 * math.cos(0) * grid_value)
    # new_j = robot_j + int(0.2 * math.sin(0) * grid_value)
    # plot_display.drawLine(robot_j, robot_i, new_j, new_i)

    if len(states) > 0:
        states_skipped = states[2:]
        # Reshape the states array into an 8 × 8 grid
        grid = [states_skipped[i:i + grid_size] for i in range(0, len(states_skipped), grid_size)]

        # Draw each cell in the grid
        for i in range(grid_size):
            for j in range(grid_size):
                # Normalize the value in `states` to calculate red intensity (assuming states[i][j] is between 0 and 1)
                red_intensity = int(abs(grid[i][j]) * 255)  # Scale to 0-255
                color = (red_intensity << 16)  # Shift by 16 bits for the red component

                # Set the color and fill the cell
                plot_display.setColor(color)
                plot_display.fillRectangle(j, i, 1, 1)



# def mark_states_in_grid(plot_display, yaw):
#     global states
#     i = 2
#     while i < len(states):
#         # angle [-pi/2,pi/2]
#         angle_raw = states[i] 
#         i += 1
#         # distance
#         distance_raw = states[i]
#         i += 1
#         # probability
#         p = states[i] 
#         i += 1
#         new_i = robot_i + int(distance_raw * math.cos(angle_raw) * grid_diameter)
#         new_j = robot_j + int(distance_raw * math.sin(angle_raw) * grid_diameter)
#         if p > 0:
#             red_intensity = int(p * 255)  # Calculate the intensity (0 to 255)
#             color = (red_intensity << 16)  # Shift the intensity to the red component
#             # Set the color with calculated intensity
#             plot_display.setColor(color)
#         else:
#             red_intensity = int(abs(p) * 255)  # Calculate the intensity (0 to 255)
#             color = (red_intensity << 8)  # Shift by 8 bits for the green component
#             # Set the color with calculated intensity
#             plot_display.setColor(color)
#         # print("new_i", new_i)
#         # print("new_j", new_j)
#         if 0 <= new_i < grid_size and 0 <= new_j < grid_size:
#             plot_display.fillRectangle(new_j, new_i, 1, 1)
            



global surpervisor, is_reset
if __name__ == '__main__':
    ros_init()
    is_reset = False
    # robot = Robot()
    robot = Supervisor() # Robot()
    surpervisor = robot

    timestep = int(robot.getBasicTimeStep())


    # Access Rubber Duck node
    rubber_duck_node = robot.getFromDef("RubberDuck")

    # Access the Obstacles node
    obstacle_01_node = robot.getFromDef("Obstacle01")
    obstacle_02_node = robot.getFromDef("Obstacle02")
    obstacle_03_node = robot.getFromDef("Obstacle03")
    obstacle_04_node = robot.getFromDef("Obstacle04")
    obstacle_05_node = robot.getFromDef("Obstacle05")
    obstacle_06_node = robot.getFromDef("Obstacle06")
    obstacle_07_node = robot.getFromDef("Obstacle07")
    obstacle_08_node = robot.getFromDef("Obstacle08")
    obstacle_09_node = robot.getFromDef("Obstacle09")
    obstacle_10_node = robot.getFromDef("Obstacle10")

    plot_display = robot.getDevice("display")
    if not plot_display:
        print("Warning: 'Plot' Display device not found.")
    
    plot_width = plot_display.getWidth()
    plot_height = plot_display.getHeight()
    # Plot data on the Display
    clear_plot(plot_display, plot_width, plot_height)

    obstacles = [
        {
            "speed":0.2,
            "distance_limit": 0.4,
            "direction": 1,
            "x": 0,
            "y": 0,
            "current_x": 0,
            "current_y": 0
         },
        {
            "speed":0.2,
            "distance_limit": 0.3,
            "direction": 1,
            "x": -1.5,
            "y": 1,
            "current_x": -1.5,
            "current_y": 1
        },
        {
            "speed":0.1,
            "distance_limit": 0.2,
            "direction": 1,
            "x": 1,
            "y": -1,
            "current_x": 1,
            "current_y": -1
        },
        {
            "speed":0.25,
            "distance_limit": 0.6,
            "direction": -1,
            "x": 0,
            "y": 2,
            "current_x": 0,
            "current_y": 2
        },
        {
            "speed":0.25,
            "distance_limit": 0.6,
            "direction": 1,
            "x": -1.3,
            "y": 1.7,
            "current_x": -1.3,
            "current_y": 1.7
        },
        {
            "speed":0.25,
            "distance_limit": 0.6,
            "direction": -1,
            "x": 0.6,
            "y": -1.7,
            "current_x": 0.6,
            "current_y": -1.7
        },
        {
            "speed":0.3,
            "distance_limit": 0.5,
            "direction": 1,
            "x": 1.8,
            "y": -0.7,
            "current_x": 1.8,
            "current_y": -0.7
        },
        {
            "speed":0.1,
            "distance_limit": 0.3,
            "direction": -1,
            "x": 1,
            "y": 0,
            "current_x": 1,
            "current_y": 0
         },
         {
            "speed":0.2,
            "distance_limit": 0.3,
            "direction": -1,
            "x": -1.5,
            "y": 0,
            "current_x": -1.5,
            "current_y": 0
        },
        {
            "speed":0.2,
            "distance_limit": 0.3,
            "direction": -1,
            "x": 0,
            "y": -1,
            "current_x": 0,
            "current_y": -1
        }]

    # target position
    # Generate a random angle theta between 0 and 2*pi

    # theta = random.uniform(0, 2 * math.pi)
    # rnd_number = random.uniform(0, 4)
    # if 0 <= rnd_number <= 1 :
    #     theta = 0.15 # 8 degrees
    # elif 1 < rnd_number <= 2:
    #     theta = -0.15
    # elif 2 < rnd_number <= 3:
    #     theta = 0.8 # 45 degrees
    # else:
    #     theta = -0.8

    # theta = -0.15
    
    # Calculate random x and y coordinates on the circle
    # radius = 3.5
    target_x = 1.7 # radius * math.cos(theta)
    target_y = 1.7 # radius * math.sin(theta)

    # target_x = 3
    # target_y = -2
    
    if rubber_duck_node is not None:
        # Get translation and rotation
        # translation = rubber_duck_node.getField("translation").getSFVec3f()
        # rotation = rubber_duck_node.getField("rotation").getSFRotation()

        # Set Rubber Duck's position to target_x and target_y
        rubber_duck_node.getField("translation").setSFVec3f([target_x, target_y, 0.0])

        # rubber_duck_node.getField("translation").setSFVec3f([1.5, 0, 0.0])
        
        # Get updated translation
        translation = rubber_duck_node.getField("translation").getSFVec3f()
        
        print(f"Rubber Duck (Target) Translation: {translation}")
        # print(f"Rubber Duck Rotation: {rotation}")
    else:
        print("Rubber Duck node not found in the scene.")

    ## Initialize motors
    m1_motor = robot.getDevice("m1_motor");
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor");
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor");
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor");
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    ## Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    # camera = robot.getDevice("camera")
    # camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    # range_back = robot.getDevice("range_back")
    # range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    ## Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    ## Initialize variables
    robot_statrt_x = -1.2
    robot_statrt_y = -1.2
    robot_statrt_z = 0
    robot_start_yaw = 0.0

    past_x_global = robot_statrt_x
    past_y_global = robot_statrt_y
    past_time = robot.getTime()

    # Crazyflie velocity PID controller
    PID_CF = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE

    print("\n")

    print("====== Controls =======\n\n")

    print(" The Crazyflie can be controlled from your keyboard!\n")
    print(" All controllable movement is in body coordinates\n")
    print("- Use the up, back, right and left button to move in the horizontal plane\n")
    print("- Use Q and E to rotate around yaw ")
    print("- Use W and S to go up and down\n ")

    # Main loop:
    while robot.step(timestep) != -1 and not rospy.is_shutdown():
        dt = robot.getTime() - past_time
        actual_state = {}

        ## Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt

        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        # print(f"yaw={yaw}, x_global={x_global}, v_x_global={v_x_global}, y_global={y_global}, v_y_global={v_y_global}, v_x={v_x}, v_y={v_y}")

        ## Initialize values
        desired_state = [0, 0, 0, 0]
        # forward_desired = 0
        # sideways_desired = 0
        # yaw_desired = 0
        height_diff_desired = 0

        key = keyboard.getKey()
        while key>0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired =  + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1

            key = keyboard.getKey()


        height_desired += height_diff_desired * dt

        ## Example how to get sensor data
        range_front_value = range_front.getValue()
        # range_back_value = range_back.getValue()
        range_left_value = range_left.getValue()
        range_right_value = range_right.getValue()
        ## cameraData = camera.getImage()

        # print(f"roll={roll}, pitch={pitch}, height_desired={height_desired}, altitute={altitude}, v_x={v_x}, v_y={v_y}")
        motor_power = []
        if is_reset:
            PID_CF = pid_velocity_fixed_height_controller()
            # Reset robot's position and rotation
            initial_position = [robot_statrt_x, robot_statrt_y, robot_statrt_z]  
            initial_rotation = [0, 0, 1, robot_start_yaw] 
            robot_node = surpervisor.getFromDef("Crazyflie")
            translation_field = robot_node.getField("translation")
            rotation_field = robot_node.getField("rotation")
            translation_field.setSFVec3f(initial_position)
            rotation_field.setSFRotation(initial_rotation)
            x_global = robot_statrt_x
            y_global = robot_statrt_y
            past_x_global = robot_statrt_x
            past_y_global = robot_statrt_y
            yaw = robot_start_yaw
            forward_desired = 0
            sideways_desired = 0
            yaw_desired = 0
            yaw_rate = 0
            height_desired = FLYING_ATTITUDE
            v_x = 0
            v_y = 0

            ## Initialize Sensors
            imu.disable()
            imu.enable(timestep)
            gps.disable()
            gps.enable(timestep)
            gyro.disable()
            gyro.enable(timestep)
            # camera.disable()
            # camera.enable(timestep)
            range_front.disable()
            range_front.enable(timestep)
            range_left.disable()
            range_left.enable(timestep)
            # range_back.disable()
            # range_back.enable(timestep)
            range_right.disable()
            range_right.enable(timestep)
            is_reset = False
            # print("RESET MOOOOOOOOOOOOOOOOODDDDDDDDDDDDDDDDEEEEEEEEEEEEEEEEEEEEEE")

        # PID velocity controller with fixed height
        motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        # print(f"dt={dt}, forward_desired={forward_desired}, sideways_desired={sideways_desired}, yaw_desired={yaw_desired}, height_desired={height_desired}, roll={roll}, pitch={pitch}, yaw_rate={yaw_rate}, altitude={altitude}, v_x={v_x}, v_y={v_y}")
        # print(motor_power)
        # print(f"m1={m1_motor.getVelocity()}, m2={m2_motor.getVelocity()}, m3={m3_motor.getVelocity()}, m4={m4_motor.getVelocity()}")
        
        # If the obstacle exists, move it dynamically
        if obstacle_01_node is not None and obstacle_02_node is not None and \
            obstacle_03_node is not None and obstacle_04_node is not None and obstacle_05_node is not None\
            and obstacle_06_node is not None and obstacle_07_node is not None and\
            obstacle_08_node is not None and obstacle_09_node is not None and obstacle_10_node is not None:

            time_obs = robot.getBasicTimeStep() / 1000.0
            obstacles[0]["current_x"] += obstacles[0]["direction"] * obstacles[0]["speed"] * time_obs
            obstacles[1]["current_y"] += obstacles[1]["direction"] * obstacles[1]["speed"] * time_obs
            obstacles[2]["current_x"] += obstacles[2]["direction"] * obstacles[2]["speed"] * time_obs
            obstacles[2]["current_y"] = (-3 * obstacles[2]["current_x"] + 1) / 2
            obstacles[3]["current_x"] += obstacles[3]["direction"] * obstacles[3]["speed"] * time_obs
            obstacles[3]["current_y"] = (2 * obstacles[3]["current_x"] + 3) / 3
            obstacles[4]["current_x"] += obstacles[4]["direction"] * obstacles[4]["speed"] * time_obs
            obstacles[5]["current_x"] += obstacles[5]["direction"] * obstacles[5]["speed"] * time_obs
            obstacles[6]["current_y"] += obstacles[6]["direction"] * obstacles[6]["speed"] * time_obs
            obstacles[7]["current_x"] += obstacles[7]["direction"] * obstacles[7]["speed"] * time_obs
            obstacles[8]["current_y"] += obstacles[8]["direction"] * obstacles[8]["speed"] * time_obs
            obstacles[9]["current_x"] += obstacles[9]["direction"] * obstacles[9]["speed"] * time_obs



            # Check if the obstacle has reached the distance limit and reverse direction if needed
            if abs(obstacles[0]["current_x"] - obstacles[0]["x"]) >= obstacles[0]["distance_limit"] :
                obstacles[0]["direction"] *= -1  # Reverse the direction

            if abs(obstacles[1]["current_y"] - obstacles[1]["y"]) >= obstacles[1]["distance_limit"]:
                obstacles[1]["direction"] *= -1  # Reverse the direction


            if abs(obstacles[2]["current_x"] - obstacles[2]["x"]) >= obstacles[2]["distance_limit"]:
                obstacles[2]["direction"] *= -1  # Reverse the direction


            if abs(obstacles[3]["current_x"] - obstacles[3]["x"]) >= obstacles[3]["distance_limit"]:
                obstacles[3]["direction"] *= -1  # Reverse the direction

            if abs(obstacles[4]["current_x"] - obstacles[4]["x"]) >= obstacles[4]["distance_limit"]:
                obstacles[4]["direction"] *= -1  # Reverse the direction
            
            if abs(obstacles[5]["current_x"] - obstacles[5]["x"]) >= obstacles[5]["distance_limit"]:
                obstacles[5]["direction"] *= -1  # Reverse the direction
            
            if abs(obstacles[6]["current_y"] - obstacles[6]["y"]) >= obstacles[6]["distance_limit"]:
                obstacles[6]["direction"] *= -1  # Reverse the direction
            
            if abs(obstacles[7]["current_x"] - obstacles[7]["x"]) >= obstacles[7]["distance_limit"] :
                obstacles[7]["direction"] *= -1  # Reverse the direction

            if abs(obstacles[8]["current_y"] - obstacles[8]["y"]) >= obstacles[8]["distance_limit"]:
                obstacles[8]["direction"] *= -1  # Reverse the direction

            if abs(obstacles[9]["current_x"] - obstacles[9]["x"]) >= obstacles[9]["distance_limit"] :
                obstacles[9]["direction"] *= -1  # Reverse the direction

            # Update obstacle's position (keeping y constant or modify similarly if needed)
            obstacle_01_node.getField("translation").setSFVec3f([obstacles[0]["current_x"], obstacles[0]["current_y"], 0.75])
            obstacle_02_node.getField("translation").setSFVec3f([obstacles[1]["current_x"], obstacles[1]["current_y"], 0.75])
            obstacle_03_node.getField("translation").setSFVec3f([obstacles[2]["current_x"], obstacles[2]["current_y"], 0.75])
            obstacle_04_node.getField("translation").setSFVec3f([obstacles[3]["current_x"], obstacles[3]["current_y"], 0.75])
            obstacle_05_node.getField("translation").setSFVec3f([obstacles[4]["current_x"], obstacles[4]["current_y"], 0.75])
            obstacle_06_node.getField("translation").setSFVec3f([obstacles[5]["current_x"], obstacles[5]["current_y"], 0.75])
            obstacle_07_node.getField("translation").setSFVec3f([obstacles[6]["current_x"], obstacles[6]["current_y"], 0.75])
            obstacle_08_node.getField("translation").setSFVec3f([obstacles[7]["current_x"], obstacles[7]["current_y"], 0.75])
            obstacle_09_node.getField("translation").setSFVec3f([obstacles[8]["current_x"], obstacles[8]["current_y"], 0.75])
            obstacle_10_node.getField("translation").setSFVec3f([obstacles[9]["current_x"], obstacles[9]["current_y"], 0.75])
        
        data = {
            "x": x_global,
            "y": y_global,
            "z": altitude,
            "front": range_front_value,
            "back": 0, # range_back_value,
            "left": range_left_value,
            "right": range_right_value,
            "tx": target_x,
            "ty": target_y,
            "timestep": timestep,
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "v_x": v_x,
            "v_y": v_y,
            "yaw_rate": yaw_rate,
            "obs_01_x": obstacles[0]["current_x"],
            "obs_01_y": obstacles[0]["current_y"],
            "obs_02_x": obstacles[1]["current_x"],
            "obs_02_y": obstacles[1]["current_y"],
            "obs_03_x": obstacles[2]["current_x"],
            "obs_03_y": obstacles[2]["current_y"],
            "obs_04_x": obstacles[3]["current_x"],
            "obs_04_y": obstacles[3]["current_y"],
            "obs_05_x": obstacles[4]["current_x"],
            "obs_05_y": obstacles[4]["current_y"],
            "obs_06_x": obstacles[5]["current_x"],
            "obs_06_y": obstacles[5]["current_y"],
            "obs_07_x": obstacles[6]["current_x"],
            "obs_07_y": obstacles[6]["current_y"],
            "obs_08_x": obstacles[7]["current_x"],
            "obs_08_y": obstacles[7]["current_y"],
            "obs_09_x": obstacles[8]["current_x"],
            "obs_09_y": obstacles[8]["current_y"],
            "obs_10_x": obstacles[9]["current_x"],
            "obs_10_y": obstacles[9]["current_y"]
        }
        json_string = json.dumps(data)
        publish_string_message(json_string)
        # print(f"front={range_front_value}, left={range_left_value}, right={range_right_value}")
        # print(yaw)


        


        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global

        clear_plot(plot_display, plot_width, plot_height)
        # mark_states_in_grid(plot_display, yaw)

        # rospy.spin()
        