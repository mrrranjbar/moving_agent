#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest  # Import the Trigger service message types
from planning.classic_based.GUI import Preparation
from planning.classic_based.Process import Process
from std_msgs.msg import String
import json

shared_input_string = ""

def move_a(request):
    global shared_input_string  # Declare the variable as global
    rospy.loginfo("Received request!")
    # rospy.loginfo("request: %s", request)

    # Unsubscribe from the topic
    rospy.Subscriber("input_str", String, callback, queue_size=10).unregister()
    # Wait for 2 seconds
    rospy.sleep(2)

    # rospy.loginfo("Received message: %s", shared_input_string)

    preparation = Preparation()
    preparation.Initialize(shared_input_string)
    pr = Process(preparation.rings, preparation.Source)
    pr.MainLoop()
    path_temp = pr.MainFunction(preparation.Target)
    json_str = json.dumps([point.__dict__ for point in path_temp])
    # print("result: ")
    # result_msg = ""
    # for pt in path_temp:
    #     result_msg += "X: " + str(pt.x) + " Y: " + str(pt.y) + "\n"
    #     print("X:", pt.x, "Y:", pt.y)

    # Process the request and generate a response
    response = TriggerResponse()
    response.success = True
    response.message = json_str #"Response test for sending to the client!"

    talker(json_str)

    return response

def talker(message):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.loginfo(message)
    pub.publish(message)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(message)
        # rate.sleep()

def callback(data):
    global shared_input_string  # Declare the variable as global
    # Process the received message
    # rospy.loginfo("Received message: %s", data.data)
    shared_input_string = data.data


if __name__ == '__main__':
    rospy.init_node('service_node')
    rospy.Service('move_a', Trigger, move_a)  # Use the Trigger service message type
    
    # Subscribe to the topic
    rospy.Subscriber("input_str", String, callback, queue_size=10)

    rospy.loginfo("Service node ready to move the agent.")
    rospy.spin()
