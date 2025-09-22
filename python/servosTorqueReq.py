#!/usr/bin/env python
import rospy
from hrpsys_ros_bridge.srv import OpenHRP_ServoControllerService_getTorque

def call_torque_service():
    rospy.init_node('torque_service_caller')
    
    n = 30  # frequency in Hz
    rate = rospy.Rate(n)
    
    rospy.wait_for_service('/ServoControllerServiceROSBridge/getTorque')
    get_torque = rospy.ServiceProxy(
        '/ServoControllerServiceROSBridge/getTorque', 
        OpenHRP_ServoControllerService_getTorque
    )
    
    # Joint ID groups
    right_joints = [2, 3, 4, 5]
    left_joints = [6, 7, 8, 9]
    
    while not rospy.is_shutdown():
        right_torques = []
        left_torques = []
        
        for joint_id in right_joints:
            try:
                response = get_torque(id=joint_id)
                right_torques.append(response.torque)  # adjust if attribute is different
            except rospy.ServiceException as e:
                right_torques.append(None)
                rospy.logerr("Service call failed for joint %d: %s", joint_id, e)
        
        for joint_id in left_joints:
            try:
                response = get_torque(id=joint_id)
                left_torques.append(response.torque)
            except rospy.ServiceException as e:
                left_torques.append(None)
                rospy.logerr("Service call failed for joint %d: %s", joint_id, e)
        
        rospy.loginfo("Right: %s | Left: %s", right_torques, left_torques)
        rate.sleep()

if __name__ == "__main__":
    call_torque_service()
