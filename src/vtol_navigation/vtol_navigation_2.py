#!/usr/bin/env python
# coding: utf-8
# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped,Pose 
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
import math
import numpy as np
import time



class Modes:
    def __init__(self):
        pass
 
    def setArm(self):
        rospy.wait_for_service('standard_vtol_0/mavros/cmd/arming')
        armService = rospy.ServiceProxy('standard_vtol_0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)

 
    def set_mode(self, mode):
        rospy.wait_for_service('standard_vtol_0/mavros/set_mode')
        setModeService = rospy.ServiceProxy('standard_vtol_0/mavros/set_mode', mavros_msgs.srv.SetMode)
        setModeService(custom_mode=mode)


    def vtol_trans(self, vstate):
        rospy.wait_for_service('standard_vtol_0/mavros/cmd/vtol_transition')
        VtolTransitionService = rospy.ServiceProxy('standard_vtol_0/mavros/cmd/vtol_transition', mavros_msgs.srv.CommandVtolTransition)
        VtolTransitionService(state=vstate)

 
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        self.local_p=PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        self.target=Pose()
        self.recive=False
        
    def stateCb(self, msg):
        self.state = msg
    
    def positionCb(self,msg):
        self.local_p=msg

    def check_p(self,x,y,r):
        if self.local_p.pose.position.x>x-r and self.local_p.pose.position.x<x+r:
            if self.local_p.pose.position.y>y-r and self.local_p.pose.position.y<y+r:
                return True
        return False
    
    def target_cb(self,msg):
        self.target=msg
        self.recive=True

def main():
    rospy.init_node('nav2', anonymous=True)
    local_pos_pub=rospy.Publisher('/xtdrone/standard_vtol_0/cmd_pose_enu',Pose, queue_size=10)
    rate = rospy.Rate(20.0)
    stateMt = stateMoniter()
    md = Modes()
    rospy.Subscriber("/standard_vtol_0/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber("/standard_vtol_0/mavros/local_position/pose",PoseStamped, stateMt.positionCb)
    rospy.Subscriber("/target_final",Pose,stateMt.target_cb)

    sign1=Bool()
    sign1.data=True
    arrive_pub = rospy.Publisher('/ship', Bool, queue_size=10)


    #设置循环，未接受目标点不开启下方流程
    while  not rospy.is_shutdown() and  not stateMt.recive:
        print ("未收到")
        time.sleep(1)

    target = [stateMt.target .position.x,stateMt.target.position.y]
    
    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
        print ("Waiting arming")

    pose = Pose()
    pose.position.x = 10  # x坐标，单位：米
    pose.position.y = 10  # y坐标，单位：米
    pose.position.z = 25  # z坐标，单位：米，表示高度

    md.vtol_trans(4)
    print ("切换固定翼")
    i=0
    while not rospy.is_shutdown() and i<len(target):
        # 设置目标位置
        pose.position.x = target[0]  # x坐标，单位：米
        pose.position.y = target[1]  # y坐标，单位：米
        pose.position.z = 20  # z坐标，单位：米，表示高度，可调
        md.set_mode("OFFBOARD")
        print ("Switching to OFFBOARD")
        while not stateMt.check_p(target[0],target[1],60):
            rospy.loginfo("Publishing position1")
            local_pos_pub.publish(pose)
            rate.sleep()
        rospy.loginfo("Reached1")
        i=i+1
        if i==len(target):
            md.vtol_trans(3)
            print ("切换四旋翼")

    # md.set_mode("OFFBOARD")
    # print "Switching to OFFBOARD"
    while not stateMt.check_p(target[0],target[1],1) :
        # if stateMt.check_p(target[0],target[1],6):
        #     arrive_pub.publish(sign1)
        #     print("到达")
        rospy.loginfo("Publishing position2")
        local_pos_pub.publish(pose)   
        rate.sleep()
    rospy.loginfo("Reached2")

    md.set_mode("AUTO.LOITER")
    print ("Switching to AUTO.LOITER" )
    while not rospy.is_shutdown() :
        if stateMt.check_p(target[0],target[1],1):
            arrive_pub.publish(sign1)


if __name__ == '__main__':
    main()

