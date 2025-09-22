#!/usr/bin/env python
import rospy
from hrpsys_ros_bridge.srv import OpenHRP_ServoControllerService_getTorque

def call_torque_service():
    rospy.init_node('torque_service_caller')
    
    n = 30 # frequency in Hz
    rate = rospy.Rate(n)
    
    rospy.wait_for_service('/ServoControllerServiceROSBridge/getTorque')
    
    try:
        get_torque = rospy.ServiceProxy(
            '/ServoControllerServiceROSBridge/getTorque', 
            OpenHRP_ServoControllerService_getTorque
        )
        while not rospy.is_shutdown():
            response = get_torque(id=2)
            rospy.loginfo("Torque response: %s", response)
            rate.sleep()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    call_torque_service()
