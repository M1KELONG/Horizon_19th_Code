#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, numpy, threading, time, queue
from rclpy.node import Node 
from ai_msgs.msg import PerceptionTargets 
from geometry_msgs.msg import Twist
from origincar_msg.msg import Sign
from std_msgs.msg import Int32
from collections import deque


msg_process_thread = None

# 数据缓冲区
class DataBuffer:
    def __init__(self, max_len=10):
        self.all_target = deque(maxlen=max_len)

    def add_all_target(self, data):
        self.all_target.append(data)

    def get_synced_data(self):
        if len(self.all_target) == 0:
            return 0
        return self.all_target[-1]



class RacingControl(Node): 
    def __init__(self,node_name): 
        super().__init__(node_name)
        self.data_buffer = DataBuffer()
        self.all_targets = queue.Queue()
        self.targets_list = []
        self.mutex = threading.Lock()

        # 类全局变量区
        self.twist = Twist()
        # self.receive = False
        # self.targets_queue = []
        self.line = []
        self.obstacle = []
        self.clock = []
        self.anticlock = []
        self.P = []
        self.process_stop = False
        self.state = 'start'
        self.angular_z_record = []
        self.time_record = []
        self.data = 0
        self.QR_cnt = 0
        self.mission2 = False
        self.mission3 = False
        self.P_cnt = 0
        self.point_target_mutex = threading.Lock()

        # 可调参数区
        self.declare_parameter('receive_t', 0.05)
        self.receive_t = self.get_parameter('receive_t').value
        
        self.declare_parameter('follow_angular_ratio_I', 1.0)
        self.follow_angular_ratio_I = self.get_parameter('follow_angular_ratio_I').value
        self.declare_parameter('follow_linear_speed_I', 0.3)
        self.follow_linear_speed_I = self.get_parameter('follow_linear_speed_I').value
        self.declare_parameter('follow_angular_ratio_III', 1.0)
        self.follow_angular_ratio_III = self.get_parameter('follow_angular_ratio_III').value
        self.declare_parameter('follow_linear_speed_III', 0.3)
        self.follow_linear_speed_III = self.get_parameter('follow_linear_speed_III').value

        self.declare_parameter('ob_bottom_confidence', 0.5)
        self.ob_bottom_confidence = self.get_parameter('ob_bottom_confidence').value
        self.declare_parameter('ob_bottom_th_I', 320)
        self.ob_bottom_th_I = self.get_parameter('ob_bottom_th_I').value
        self.declare_parameter('ob_bottom_th_in_I', 300)
        self.ob_bottom_th_in_I = self.get_parameter('ob_bottom_th_in_I').value
        self.declare_parameter('avoid_angular_ratio_I', 1.0)
        self.avoid_angular_ratio_I = self.get_parameter('avoid_angular_ratio_I').value
        self.declare_parameter('avoid_linear_speed_I', 0.2)
        self.avoid_linear_speed_I = self.get_parameter('avoid_linear_speed_I').value
        self.declare_parameter('ob_bottom_th_III', 320)
        self.ob_bottom_th_III = self.get_parameter('ob_bottom_th_III').value
        self.declare_parameter('ob_bottom_th_in_III', 300)
        self.ob_bottom_th_in_III = self.get_parameter('ob_bottom_th_in_III').value
        self.declare_parameter('avoid_angular_ratio_III', 1.0)
        self.avoid_angular_ratio_III = self.get_parameter('avoid_angular_ratio_III').value
        self.declare_parameter('avoid_linear_speed_III', 0.2)
        self.avoid_linear_speed_III = self.get_parameter('avoid_linear_speed_III').value

        self.declare_parameter('modify_cnt_th_I', 50)
        self.modify_cnt_th_I = self.get_parameter('modify_cnt_th_I').value
        self.declare_parameter('line_cnt_th_I', 2000)
        self.line_cnt_th_I = self.get_parameter('line_cnt_th_I').value
        self.declare_parameter('modify_cnt_th_III', 50)
        self.modify_cnt_th_III = self.get_parameter('modify_cnt_th_III').value
        self.declare_parameter('line_cnt_th_III', 2000)
        self.line_cnt_th_III = self.get_parameter('line_cnt_th_III').value

        self.declare_parameter('line_ob_x_th', 20)
        self.line_ob_x_th = self.get_parameter('line_ob_x_th').value
        self.declare_parameter('sleep_time_I', 0.3)
        self.sleep_time_I = self.get_parameter('sleep_time_I').value
        self.declare_parameter('modify_ratio_I', 0.8)
        self.modify_ratio_I = self.get_parameter('modify_ratio_I').value
        self.declare_parameter('sleep_time_III', 0.3)
        self.sleep_time_III = self.get_parameter('sleep_time_III').value
        self.declare_parameter('modify_ratio_III', 0.8)
        self.modify_ratio_III = self.get_parameter('modify_ratio_III').value

        self.declare_parameter('QR_cnt_th', 5)
        self.QR_cnt_th = self.get_parameter('QR_cnt_th').value
        # self.declare_parameter('clock_conf', 0.5)
        # self.clock_conf = self.get_parameter('clock_conf').value
        # self.declare_parameter('anti_conf', 0.5)
        # self.anti_conf = self.get_parameter('anti_conf').value

        self.declare_parameter('P_bottom_th', 350)
        self.P_bottom_th = self.get_parameter('P_bottom_th').value
        self.declare_parameter('P_cnt_th', 50)
        self.P_cnt_th = self.get_parameter('P_cnt_th').value
        self.declare_parameter('end_cnt_th', 50)
        self.end_cnt_th = self.get_parameter('end_cnt_th').value
        self.declare_parameter('park_s_close', 0.3)
        self.park_s_close = self.get_parameter('park_s_close').value
        self.declare_parameter('park_s_stop', 0.2)
        self.park_s_stop = self.get_parameter('park_s_stop').value
        self.declare_parameter('park_angular_ratio', 2.0)
        self.park_angular_ratio = self.get_parameter('park_angular_ratio').value
        

        # 订阅者&发布者
        self.minimal_subscriber = self.create_subscription( 
            PerceptionTargets, 
            'hobot_dnn_detection',  
            self.listener_callback, 
            10) 
        self.minimal_subscriber  # prevent unused variable warning 

        self.foxglove_sub = self.create_subscription( 
            Int32, 
            'sign4return',  
            self.foxglove_callback, 
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10)

        self.foxglove_pub = self.create_publisher(
            Sign,
            "sign_switch",
            10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.receive_timer = self.create_timer(self.receive_t, self.receive_timer_callback)

        print("Racing_control initialization finished!")



    def timer_callback(self):
        out_msg = Sign()
        out_msg.sign_data = self.data
        self.foxglove_pub.publish(out_msg)

    def receive_timer_callback(self):
        with self.mutex:
            self.line = []
            self.obstacle = []
            self.clock = []
            self.anticlock = []
            self.P = []
            # targets_list = self.data_buffer.get_synced_data()
            targets_list = self.targets_list
            if targets_list != 0:
                for num, target in enumerate(targets_list):
                    if (target.rois[0].type == "line"):
                        self.line.append(target)
                    elif (target.rois[0].type == "obstacle"):
                        self.obstacle.append(target)
                    elif (target.rois[0].type == "clock"):
                        self.clock.append(target)
                    elif (target.rois[0].type == "anticlock"):
                        self.anticlock.append(target)
                    elif (target.rois[0].type == "P"):
                        self.P.append(target)

    def listener_callback(self, msg):
        with self.mutex:
            self.all_targets.put(msg.targets)
            if self.all_targets.qsize() > 1:
                self.targets_list = self.all_targets.get()

        # self.data_buffer.add_all_target(msg.targets)

        # # if self.receive == True:
        # # 回调函数，每帧推理结果产生后就会运行这个函数
        # # 这里给出解析的方法
        # # print("\n \033[31m---\033[0m This Frame: FPS = %d  \033[31m---\033[0m"%msg.fps)
        # # with self.point_target_mutex:
        # #     self.targets_queue.append(msg)
        # #     if len(self.targets_queue) > 1:
        # #         self.targets_queue.pop(0)
        # self.line = []
        # self.obstacle = []
        # self.clock = []
        # self.anticlock = []
        # self.P = []
        # for num, target in enumerate(msg.targets):
        #     # print("Traget \033[0;32;40m%d\033[0m: "%num, end="")
        #     # print("Type: %s, x_offset=%d, y_offset=%d, height=%d, width=%d, conf=%.2f"%(target.rois[0].type, 
        #     # target.rois[0].rect.x_offset,
        #     # target.rois[0].rect.y_offset,
        #     # target.rois[0].rect.height,
        #     # target.rois[0].rect.width,
        #     # target.rois[0].confidence))
        #     if (target.rois[0].type == "line"):
        #         self.line.append(target)
        #     elif (target.rois[0].type == "obstacle"):
        #         self.obstacle.append(target)
        #     elif (target.rois[0].type == "clock"):
        #         self.clock.append(target)
        #     elif (target.rois[0].type == "anticlock"):
        #         self.anticlock.append(target)
        #     elif (target.rois[0].type == "P"):
        #         self.P.append(target)
        # # else:
        # #     pass               

    def foxglove_callback(self, msg):
        data = msg.data
        # 点击了“C区出口结束遥测”
        if (data == 6):
            self.mission3 = True
            self.mission2 = False
    
    # 找最近的线
    def line_closest(self):
        list = self.line
        line_bottom_temp = 0
        line_target_ = list[0]
        for i_line,line_target in enumerate(list):
            # line_bottom为线框的下边值
            line_bottom = (line_target.rois[0].rect.y_offset)+(line_target.rois[0].rect.height)
            if line_bottom > line_bottom_temp:
                line_bottom_temp = line_bottom
                # line_index = i
                line_target_ = line_target
        return line_target_

    # 找最远的线
    def line_furthest(self):
        list = self.line
        line_bottom_temp = 0
        line_target_ = list[0]
        for i_line,line_target in enumerate(list):
            # line_bottom为线框的下边值
            line_bottom = (line_target.rois[0].rect.y_offset)+(line_target.rois[0].rect.height)
            if line_bottom < line_bottom_temp:
                line_bottom_temp = line_bottom
                # line_index = i
                line_target_ = line_target
        return line_target_

    def line_follower(self,target):
        # if self.state != 'follow_line': 
        #     self.get_logger().info("Now following line!")
        #     self.angular_z_record = []
        #     self.time_record = []
        #     self.state = 'follow_line'
        if self.mission3 == True:
            follow_linear_speed = self.follow_linear_speed_III
            follow_angular_ratio = self.follow_angular_ratio_III
        else:
            follow_linear_speed = self.follow_linear_speed_I
            follow_angular_ratio = self.follow_angular_ratio_I           

        x = (target.rois[0].rect.x_offset) + 0.5*(target.rois[0].rect.width)
        y = (target.rois[0].rect.y_offset) + 0.5*(target.rois[0].rect.height)

        temp = x - 320.0
        if (x>-20)and(x<=0):
            temp = -20
        elif (x>0)and(x<20):
            temp = 20

        angular_z = follow_angular_ratio * temp / 150.0 * y / 480.0
        self.twist.linear.x = follow_linear_speed
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = -angular_z
        self.cmd_pub.publish(self.twist)

    def obstacle_avoid(self):
        modify_cnt = 0
        if self.mission3 == True:
            avoid_linear_speed = self.avoid_linear_speed_III
            avoid_angular_ratio = self.avoid_angular_ratio_III
            ob_bottom_th = self.ob_bottom_th_III
            modify_cnt_th = self.modify_cnt_th_III
        else:
            avoid_linear_speed = self.avoid_linear_speed_I
            avoid_angular_ratio = self.avoid_angular_ratio_I
            ob_bottom_th = self.ob_bottom_th_I
            modify_cnt_th = self.modify_cnt_th_I

        self.twist.linear.x = avoid_linear_speed
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        while True:
            ob_bottom_temp = 0
            # 有障碍物
            if (len(self.obstacle)!=0):
                for i_ob,ob_target in enumerate(self.obstacle):
                    ob_bottom = (ob_target.rois[0].rect.y_offset)+(ob_target.rois[0].rect.height)
                    if ob_bottom > ob_bottom_temp:
                        ob_bottom_temp = ob_bottom
                        ob_target_ = ob_target
                # 障碍物较远
                if ob_bottom_temp <= ob_bottom_th:
                    modify_cnt += 1
                # 障碍物较近【已经达到阈值】
                else:
                    modify_cnt = 0

                    # 有线、有障碍物
                    if len(self.line) != 0:
                        line_tar = self.line_furthest()
                        line_x = line_tar.rois[0].rect.x_offset + line_tar.rois[0].rect.width / 2 # 线框中心点的x坐标
                        ob_x = ob_target_.rois[0].rect.x_offset + ob_target_.rois[0].rect.width / 2  # 计算障碍物中心点的x坐标
                        # 认为障碍物就在线的正中
                        if (abs(line_x - ob_x) <= self.line_ob_x_th):
                            temp = 20
                        else:
                            temp = -(line_x - ob_x)

                    # 只有障碍物
                    else:
                        center_x = ob_target_.rois[0].rect.x_offset + ob_target_.rois[0].rect.width / 2  # 计算障碍物中心点的x坐标
                        temp = center_x - 320.0

                    if temp == 0:
                        angular_z = 0.0
                    else:
                        if -20 < temp < 0:  # 防止中心值过小时不做出反应
                            temp = -20
                        elif 0 <= temp < 20:
                            temp = 20
                        angular_z = avoid_angular_ratio * 700 / temp

                    self.twist.linear.x = avoid_linear_speed
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.z = angular_z
                    # now_time = time.time()
                    self.cmd_pub.publish(self.twist)
                    self.angular_z_record.append(angular_z)
                    # self.time_record.append(now_time)
                    z = -sum(self.angular_z_record)/len(self.angular_z_record)
            # 没有障碍物
            else:
                modify_cnt += 1

            if modify_cnt >= modify_cnt_th:
                break

        # 开始调整姿态返回巡线
        try:
            self.modify(z)
        except:
            pass        
         
    def modify(self,z):
        P_cnt = 0
        line_cnt = 0
        if self.mission3 == True:
            avoid_linear_speed = self.avoid_linear_speed_III
            sleep_time = self.sleep_time_III
            line_cnt_th = self.line_cnt_th_III
            modify_ratio = self.modify_ratio_III
        else:
            avoid_linear_speed = self.avoid_linear_speed_I
            sleep_time = self.sleep_time_I
            line_cnt_th = self.line_cnt_th_I
            modify_ratio = self.modify_ratio_I

        # 往前走直线一段
        self.twist.linear.x = avoid_linear_speed
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        time.sleep(sleep_time)
        # 往回拐
        while True:
            # 任务三，判断是否只看到P，没看到障碍物
            if self.mission3==True:
                if (len(self.P)!=0) and (len(self.obstacle)==0):
                    P_cnt += 1
                if len(self.line)!=0:
                    line_cnt += 1

                # 看到一定次数的P，进入停车
                if P_cnt >= self.P_cnt_th:
                    self.park()
                # 看到一定次数的线，继续巡线
                elif line_cnt >= line_cnt_th:
                    break
                # 否则继续调整
                else:
                    self.twist.linear.x = avoid_linear_speed
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.z = modify_ratio*z
                    self.cmd_pub.publish(self.twist)
            # 任务一，判断有无识别到二维码
            else:
                if (len(self.clock)!=0) or (len(self.anticlock)!=0):
                    self.QR_cnt += 1
                if len(self.line)!=0:
                    line_cnt += 1
                
                if (self.QR_cnt >= self.QR_cnt_th) and (self.mission2==False):
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_pub.publish(self.twist)
                    self.mission2 = True
                    self.QR_cnt = 0
                    # 顺时针
                    if (len(self.clock)!=0):
                        self.data = 3
                    # 逆时针
                    elif (len(self.anticlock)!=0):
                        self.data = 4
                    break
                if line_cnt >= line_cnt_th:
                    break
                else:
                    self.twist.linear.x = avoid_linear_speed
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.z = modify_ratio*z
                    self.cmd_pub.publish(self.twist)
        self.angular_z_record = []
        self.time_record = []
    
    def park(self):
        end_cnt = 0
        # flag用于判断是否已经靠近过P
        flag = False
        while True:
            P_list = self.P

            # 看到P
            if (len(P_list)!=0):
                end_cnt = 0
                target_P_ = P_list[0]
                x = (target_P_.rois[0].rect.x_offset) + 0.5*(target_P_.rois[0].rect.width)
                y = (target_P_.rois[0].rect.y_offset) + 0.5*(target_P_.rois[0].rect.height)
                bottom = (target_P_.rois[0].rect.y_offset) + (target_P_.rois[0].rect.height)

                if bottom >= self.P_bottom_th:
                    flag = True

                temp = x - 320.0
                if (x>-20)and(x<=0):
                    temp = -20
                elif (x>0)and(x<20):
                    temp = 20

                angular_z = self.park_angular_ratio * temp / 150.0 * y / 480.0
                if flag == True:
                    self.twist.linear.x = self.park_s_stop
                else:
                    self.twist.linear.x = self.park_s_close
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.z = -angular_z
                    self.cmd_pub.publish(self.twist)
            # 看不到P
            else:
                # 已经靠近了P，理论上已经踩在P上了
                if flag==True:
                    self.twist.linear.x = self.park_s_stop
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_pub.publish(self.twist)
                    end_cnt += 1
                # 未靠近过P，因为意外看不到P
                else:
                    pass

            if end_cnt >= self.end_cnt_th:
                break

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        while True:
            pass

    def message_process(self):
        while not self.process_stop:
            ob_bottom_temp = 0

            # 已经进入任务三，才开始检测有没有P
            if (self.mission3 == True):
                ob_bottom_th_in = self.ob_bottom_th_in_III

                # 有P、没有障碍物
                P_list = self.P
                if (len(P_list)!=0) and (len(self.obstacle)==0):
                    self.P_cnt += 1
                
                if self.P_cnt >= self.P_cnt_th:
                    self.park()
            else:
                ob_bottom_th_in = self.ob_bottom_th_in_I

            # 识别到二维码
            if (len(self.clock)!=0)or(len(self.anticlock)!=0):
                self.QR_cnt += 1

            if (self.QR_cnt >= self.QR_cnt_th) and (self.mission2==False) and (self.mission3==False):
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.mission2 = True
                self.QR_cnt = 0
                # 顺时针
                # if (len(self.clock)!=0) and (self.clock[0].rois[0].confidence > self.clock_conf):
                if (len(self.clock)!=0):
                    self.data = 3
                    self.get_logger().info("ClockWise")
                # 逆时针
                # elif (len(self.anticlock)!=0) and (self.anticlock[0].rois[0].confidence > self.anti_conf):
                elif (len(self.anticlock)!=0):
                    self.data = 4
                    self.get_logger().info("AntiClockWise")

            # 没有在进行任务二
            if self.mission2==False:
                # 没有障碍物
                if (len(self.obstacle) == 0):
                    # 有线、没障碍物
                    if (len(self.line) != 0):  # 没加栈出过错
                        line_target_ = self.line_closest()
                        # line_target_ = self.line_furthest()
                        self.line_follower(line_target_)
                    # 没线、没障碍物
                    else:
                        pass
                # 有障碍物
                else:
                    self.P_cnt = 0
                    # 有线、有障碍物
                    if (len(self.line) != 0):
                        line_target_ = self.line_closest()
                        # line_target_ = self.line_furthest()
                        for i_ob,ob_target in enumerate(self.obstacle):
                            ob_bottom = (ob_target.rois[0].rect.y_offset)+(ob_target.rois[0].rect.height)
                            if ob_bottom > ob_bottom_temp:
                                ob_bottom_temp = ob_bottom
                                # ob_index = i
                                ob_target_ = ob_target
                        # 障碍物较远，巡线
                        if ob_bottom_temp <= ob_bottom_th_in:
                            self.line_follower(line_target_)
                        # 障碍物较近，避障
                        else:
                            # 置信度够，避障
                            if ob_target_.rois[0].confidence > self.ob_bottom_confidence:
                                self.obstacle_avoid()
                            # 置信度不够，巡线
                            else:
                                self.line_follower(line_target_)
                    # 没线、有障碍物
                    else:
                        for i_ob,ob_target in enumerate(self.obstacle):
                            ob_bottom = (ob_target.rois[0].rect.y_offset)+(ob_target.rois[0].rect.height)
                            if ob_bottom > ob_bottom_temp:
                                ob_bottom_temp = ob_bottom
                                # ob_index = i
                                ob_target_ = ob_target
                        # 障碍物较近，避障
                        if ob_bottom_temp > ob_bottom_th_in:
                            self.obstacle_avoid()
                        # 没线、障碍物较远
                        else:
                            pass


def main(args=None): 
    global msg_process_thread
    rclpy.init(args=args) 
    racing_control = RacingControl("racing_control")
    if not msg_process_thread:
        msg_process_thread = threading.Thread(target=racing_control.message_process)
        msg_process_thread.start()
    rclpy.spin(racing_control) 
    racing_control.destroy_node() 
    msg_process_thread.join()
    rclpy.shutdown() 

# 给main函数一个入口，省得colcon build编译
if __name__ == '__main__': 
    main()