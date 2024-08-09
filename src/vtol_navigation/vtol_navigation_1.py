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

# 卡尔曼滤波参数
Q = 1e-5  # 过程噪声协方差
R = 1e-2  # 观测噪声协方差
x = np.array([[0], [0]])  # 状态向量 [角度, 角速度]
P = np.eye(2)  # 状态协方差矩阵
dt = 0.1  # 控制周期（秒）
F = np.array([[1, dt], [0, 1]])  # 状态转移矩阵
H = np.array([[1, 0]])  # 观测矩阵
I = np.eye(2)  # 单位矩阵

# def calculate_thunder(point_self,point_thunder):


def kalman_filter(z_values):
    global x, P
    filtered_angles = []

    for z in z_values:
        # 预测
        x = F @ x
        P = F @ P @ F.T + Q * I
        # 更新
        y = z - (H @ x)  # 创新
        S = H @ P @ H.T + R  # 创新协方差
        K = P @ H.T @ np.linalg.inv(S)  # 卡尔曼增益
        x = x + K @ y
        P = (I - K @ H) @ P
        filtered_angles.append(x[0, 0])

    return filtered_angles


def line_intersection(k1, b1, k2, b2):
    # 计算x坐标
    x = (b2 - b1) / (k1 - k2)
    # 计算y坐标，可以使用任一直线的方程
    y = k1 * x + b1
    return [x, y]

def point_distance_line(point,line_point1,line_point2):
	#计算向量
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(vec1,vec2)) / np.linalg.norm(line_point1-line_point2)
    return distance

def choose_closer(a, b, c):
    # 计算a和c之间的差的绝对值
    diff_a = abs(a - c)
    # 计算b和c之间的差的绝对值
    diff_b = abs(b - c)
    # 比较差的绝对值，选择更小的那个
    if diff_a < diff_b:
        return a
    else:
        return b

def path(o,r):
    # start= np.array([-200,100])
    # end= np.array([300,150])
    start= np.array([0,0])
    end= np.array([-3500,-3500])
   
    #计算是否与圆相交
    distance = point_distance_line(o,start,end)
    if distance>=r:
        return end
    
    se=end-start #起点与终点连线
    k_se=se[1]/float(se[0] )#起点与终点连线斜率
    ang_se=math.atan(k_se)

    #计算出起点一侧切线方程
    ang=tangentline(o,start,r)
    ang1=choose_closer(ang[0],ang[1],ang_se)
    k1=math.tan(ang1)
    #print k1
    b1=start[1]-k1*start[0]

    #计算出终点一侧切线方程
    ang=tangentline(o,end,r)
    ang2=choose_closer(ang[0],ang[1],ang_se)
    k2=math.tan(ang2)
    #print k2
    b2=end[1]-k2*end[0]

    # 计算交点
    intersection = line_intersection(k1, b1, k2, b2)
    return  intersection

def tangentline(o,p,r): #设圆心为o,已知点为p,切点为x
    op=o-p
    k=op[1]/float(op[0])
    d_op= np.linalg.norm(op)
    sin_opx=r/float(d_op)
    ang_op=math.atan(k)
    ang_opx=math.asin(sin_opx)
    ang_px1=ang_op+ang_opx
    ang_px2=ang_op-ang_opx
    # k_px1=math.tan(ang_px1)
    # k_px2=math.tan(ang_px2)
    # return k_px1,k_px2
    return ang_px1,ang_px2

class Strom:
    def __init__(self):
        self.pose=Pose()
        self.r=1050
        self.center=np.array([1,1])
        self.thunder_sub = rospy.Subscriber("/zhihang/thunderstorm",Pose, self.storm_cb)

    def storm_cb(self,msg):
        self.pose=msg
        self.center[0]=self.pose.position.x-2300
        self.center[1]=self.pose.position.y-2300

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
        #创建订阅者
        self.state_sub = rospy.Subscriber("/standard_vtol_0/mavros/state",State, self.stateCb)
        self.pose_sub = rospy.Subscriber("/standard_vtol_0/mavros/local_position/pose",PoseStamped, self.positionCb)
        self.angle_sub = rospy.Subscriber('/zhihang/standard_vtol/angel', Pose, self.angle_callback)
        
        #创建发布者
        self.local_pos_pub = rospy.Publisher('/xtdrone/standard_vtol_0/cmd_pose_enu', Pose, queue_size=10)
        self.arrive_pub = rospy.Publisher("/arrive",Bool,queue_size=10)
        self.angle_pub = rospy.Publisher("/need_angle",Float64,queue_size=10)

        self.state = State()
        self.sp = PositionTarget()
        self.local_p=PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        self.signal = Bool()
        self.data_list = []
        self.need_angle = Float64()

        
    def stateCb(self, msg):
        self.state = msg
    
    def angle_callback(self,msg):
        if self.state.mode == "AUTO.LOITER":
            angle_data = msg.position.x  # 假设接收到的弧度值在x属性中
            # rospy.loginfo("Received angle: %f radians", angle_data)
            self.data_list.append(angle_data)
            data_get=np.array(self.data_list)
            KF_result = kalman_filter(data_get)
            if len(KF_result) >= 10:
                self.need_angle.data = KF_result[-1]
                self.angle_pub.publish(self.need_angle)
                self.signal.data = True
                self.arrive_pub.publish(self.signal)
                rospy.loginfo("arrive!")
                rospy.loginfo("angle_is_publishing")


    
    def positionCb(self,msg):
        self.local_p=msg
        

    def check_p(self,x,y,r):
        if self.local_p.pose.position.x>x-r and self.local_p.pose.position.x<x+r:
            if self.local_p.pose.position.y>y-r and self.local_p.pose.position.y<y+r:
                return True
        return False

def main():
    rospy.init_node('nav', anonymous=True)
    local_pos_pub = rospy.Publisher('/xtdrone/standard_vtol_0/cmd_pose_enu', Pose, queue_size=10)
    rate = rospy.Rate(20.0)
    stateMt = stateMoniter()
    md = Modes()
    strom=Strom()
    time.sleep(1)
    
    point=path(strom.center,1050)
    if point[0]==-3500:
        user_target = [point]  # 用于存储用户输入的目标坐标
    else:
        user_target = [point,[-3512,-3509]]  # 用于存储用户输入的目标坐标

    # Switching the state to auto mode
    while not stateMt.state.mode == "AUTO.TAKEOFF":
        md.set_mode("AUTO.TAKEOFF")
        rate.sleep()
        print ("Switching to AUTO.TAKEOFF")    

    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
        print ("Waiting arming")

    while not stateMt.state.mode == "AUTO.LOITER":
        rate.sleep()
        print("起飞中....")

    pose = Pose()
    #pose.header = rospy.Header()
    pose.position.x = 10  # x坐标，单位：米
    pose.position.y = 10  # y坐标，单位：米
    pose.position.z = 20  # z坐标，单位：米，表示高度
    # for i in range(10):
    #     local_pos_pub.publish(pose)  #建立连接

    md.vtol_trans(4)
    print ("切换固定翼")
    i=0
    while not rospy.is_shutdown() and i<len(user_target):
        target=user_target[i]
        # 设置目标位置
        pose.position.x = target[0]  # x坐标，单位：米
        pose.position.y = target[1]  # y坐标，单位：米
        pose.position.z = 20  # z坐标，单位：米，表示高度，可调
        md.set_mode("OFFBOARD")
        print ("Switching to OFFBOARD")
        while not stateMt.check_p(target[0],target[1],30):
            rospy.loginfo("Publishing position")
            stateMt.local_pos_pub.publish(pose)
            rate.sleep()
        rospy.loginfo("Reached")
        i=i+1
        if i==len(user_target):
            md.vtol_trans(3)
            print ("切换四旋翼")

    # md.set_mode("OFFBOARD")
    # print "Switching to OFFBOARD"
    while not stateMt.check_p(target[0],target[1],0.1):
        rospy.loginfo("Publishing position")
        stateMt.local_pos_pub.publish(pose)    
        rate.sleep()
    rospy.loginfo("Reached")
    md.set_mode("AUTO.LOITER")
    rospy.loginfo("Transite_LOITER")
    


if __name__ == '__main__':
    main()
    rospy.spin()

