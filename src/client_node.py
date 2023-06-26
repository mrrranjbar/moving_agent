#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest  # Import the Trigger service message type
# from moving_agent.srv import CustomTrigger

def client():
    rospy.wait_for_service('move_a')
    try:
        move_a = rospy.ServiceProxy('move_a', Trigger)  # Use the Trigger service message type
         # Create the service request
        request = TriggerRequest()
        response = move_a(request)  # Call the service. note: the request can not send any parameter when using srd_srvs.srv (using custom service)
        rospy.loginfo("Received response: %s", response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('client_node')
    client()
