#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse

import logging
import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from std_msgs.msg import String
import math

import json
from planning.classic_based.Process import Point

URI = 'radio://0/80/2M/E7E7E7E705'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def pixels_to_meters(pixels):
    dpi = 96
    # 1 inch = 0.0254 meters (conversion factor)
    meters_per_inch = 0.0254
    # 1 inch = 2.54 centimeters (conversion factor)
    centimeters_per_inch = 2.54
    inches = pixels / dpi
    meters = inches * centimeters_per_inch
    return meters

def pixels_to_centimeters_calibrated(pixels):
    return pixels #* 18 / 120 # 70 pixel = 18 cm

def callback2(data): #MRR whitout flight command
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    points_new = [Point(pixels_to_centimeters_calibrated(point['x']), pixels_to_centimeters_calibrated(point['y'])) for point in json.loads(data.data)]

    print("mrrrrrrrrrr")
    for item in points_new:
        print(item.x)
        print(item.y)
   
    ###################################################
    # Initialize the low-level drivers (don't list the debug drivers)
    # cflib.crtp.init_drivers(enable_debug_driver=False)

    # with SyncCrazyflie(URI) as scf:
    print('Take off command1')
    # We take off when the commander is created
    # with MotionCommander(scf) as mc:
    print('Taking off!')
    print('Sleep for 5 seconds!')
    # time.sleep(5)
    rate = 10
    velocity = 0.5
    pre_angle = 0
    for i in range(len(points_new)):
        if i == (len(points_new) - 1) : break
        
        dx = points_new[i + 1].x - points_new[i].x
        dy = points_new[i + 1].y - points_new[i].y
        degree = int(math.degrees(math.atan2(dy, dx)))
        dist = round(math.sqrt((dx * dx) + (dy * dy)) / 100.0 , 1)
        
        if i == 0 :
            print("Turning Left => angle_degrees= " + str(abs(degree)) + " rate= " + str(rate) )
            # mc.turn_left(angle_degrees=abs(degree), rate=rate)
        else:
            if turn(points_new[i - 1], points_new[i], points_new[i + 1]) == -1:
                print("Turning Right => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
                # mc.turn_right(angle_degrees=abs(degree - pre_angle), rate=rate)
            elif turn(points_new[i - 1], points_new[i], points_new[i + 1]) == 1:
                print("Turning Left => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
                # mc.turn_left(angle_degrees=abs(degree - pre_angle), rate=rate)
        # print('Sleep for 2 seconds')
        # time.sleep(2)
        print("Moving Forward => distance_m= " + str(dist) + " velocity= " + str(velocity))
        # mc.forward(distance_m=dist, velocity=velocity)
        pre_angle = degree

        # print('Sleep for 2 seconds')
        # time.sleep(2)

    print('Landing!')

def callback(data): #MRR
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    points_new = [Point(pixels_to_centimeters_calibrated(point['x']), pixels_to_centimeters_calibrated(point['y'])) for point in json.loads(data.data)]
   
    ###################################################
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        print('Take off command1')
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            print('Taking off!')
            print('Sleep for 3 seconds!')
            time.sleep(3)
            rate = 10
            velocity = 0.5
            pre_angle = 0
            for i in range(len(points_new)):
                if i == (len(points_new) - 1) : break
                
                dx = points_new[i + 1].x - points_new[i].x
                dy = points_new[i + 1].y - points_new[i].y
                degree = int(math.degrees(math.atan2(dy, dx)))
                dist = round(math.sqrt((dx * dx) + (dy * dy)) / 100.0 , 1)
                
                if i == 0 :
                    print("Turning Left => angle_degrees= " + str(abs(degree)) + " rate= " + str(rate) )
                    mc.turn_left(angle_degrees=abs(degree), rate=rate)
                else:
                    if turn(points_new[i - 1], points_new[i], points_new[i + 1]) == -1:
                        print("Turning Right => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
                        mc.turn_right(angle_degrees=abs(degree - pre_angle), rate=rate)
                    elif turn(points_new[i - 1], points_new[i], points_new[i + 1]) == 1:
                        print("Turning Left => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
                        mc.turn_left(angle_degrees=abs(degree - pre_angle), rate=rate)
                # print('Sleep for 2 seconds')
                # time.sleep(2)
                print("Moving Forward => distance_m= " + str(dist) + " velocity= " + str(velocity))
                mc.forward(distance_m=dist, velocity=velocity)
                pre_angle = degree

                # print('Sleep for 1 seconds')
                # time.sleep(1)

            print('Landing!')

            # There is a set of functions that move a specific distance
            # We can move in all directions
            # print('Moving forward 0.5m')
            # mc.forward(0.5)
            # # Wait a bit
            # time.sleep(1)

            # print('Moving up 0.2m')
            # mc.up(0.2)
            # # Wait a bit
            # time.sleep(1)

            # print('Doing a 270deg circle');
            # mc.circle_right(0.5, velocity=0.5, angle_degrees=270)

            # print('Moving down 0.2m')
            # mc.down(0.2)
            # # Wait a bit
            # time.sleep(1)

            # print('Rolling left 0.2m at 0.6m/s')
            # mc.left(0.2, velocity=0.6)
            # # Wait a bit
            # time.sleep(1)

            # print('Moving forward 0.4m')
            # mc.forward(0.4)
            # time.sleep(1)

            # We land when the MotionCommander goes out of scope
            # print('Landing!')


    ###################################################

# def callback1(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     points_new = [Point(point['x'], point['y']) for point in json.loads(data.data)]
#     # print("result: ")
#     # result_msg = ""
#     # for pt in points_new:
#     #     result_msg += "X: " + str(pt.x) + " Y: " + str(pt.y) + "\n"
#     #     print("X:", pt.x, "Y:", pt.y)
    

#     ###################################################
#     # Initialize the low-level drivers (don't list the debug drivers)
#     # cflib.crtp.init_drivers(enable_debug_driver=False)

#     # with SyncCrazyflie(URI) as scf:
#         # We take off when the commander is created
#         # with MotionCommander(scf) as mc:
#     print('Taking off!')
#     print('Sleep for 5 seconds')
#     time.sleep(5)
    
#     rate = 10
#     velocity = 0.5
#     pre_angle = 0
#     for i in range(len(points_new)):
#         if i == (len(points_new) - 1) : break
        
#         dx = points_new[i + 1].x - points_new[i].x
#         dy = points_new[i + 1].y - points_new[i].y
#         degree = int(math.degrees(math.atan2(dy, dx)))
#         dist = round(math.sqrt((dx * dx) + (dy * dy)) / 100 , 1)
        
#         if i == 0 :
#             print("Turning Left => angle_degrees= " + str(abs(degree)) + " rate= " + str(rate) )
#             # mc.turn_left(angle_degrees=abs(degree), rate=rate)
#         else:
#             if turn(points_new[i - 1], points_new[i], points_new[i + 1]) == -1:
#                 print("Turning Right => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
#                 # mc.turn_right(angle_degrees=abs(degree - pre_angle), rate=rate)
#             elif turn(points_new[i - 1], points_new[i], points_new[i + 1]) == 1:
#                 print("Turning Left => angle_degrees= " + str(abs(degree - pre_angle)) + " rate= " + str(rate) )
#                 # mc.turn_left(angle_degrees=abs(degree - pre_angle), rate=rate)
#         print('Sleep for 1 seconds')
#         time.sleep(1)
#         print("Moving Forward => distance_m= " + str(dist) + " velocity= " + str(velocity))
#         # mc.forward(distance_m=dist, velocity=velocity)
#         pre_angle = degree

#         print('Sleep for 1 seconds')
#         time.sleep(1)

#     print('Landing!')

#     ###################################################

def turn(p, q, r):
        result = (r.x - q.x) * (p.y - q.y) - (r.y - q.y) * (p.x - q.x)
        if result < 0:
            return -1  # P->Q->R is a right turn
        if result > 0:
            return 1  # P->Q->R is a left turn
        return 0  # P->Q->R is a straight line, i.e. P, Q, R are collinear
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    # rospy.init_node('service_flight_cf')
    # rospy.Service('move_b', Trigger, move_b)
    # rospy.loginfo("Service flight ready to move the agent.")
    # rospy.spin()
