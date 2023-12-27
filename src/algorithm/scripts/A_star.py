#!/usr/bin/python3
INIT_POSITION = [-2, 3, 2.57]  # in world frame
GOAL_POSITION = [-2, 13, 1.57]  # relative to the initial position
import time
import argparse
import subprocess
import os
from os.path import join
from PIL import Image
import numpy as np
import rospy
import rospkg
import math
from geometry_msgs.msg import Twist
from gazebo_simulation import GazeboSimulation

def world_to_map(world_coor:list):
    map_coor = [0,0]
    map_coor[0] = 99 - int(world_coor[1]/0.15) 
    map_coor[1] = 29 + int(world_coor[0]/0.15) 
    return map_coor

def map_to_world(map_coor:list):
    world_coor = [0,0]
    world_coor[1] = (99 - map_coor[0])*0.15
    world_coor[0] = (map_coor[1] - 29)*0.15
    return world_coor

def distance(box1:list,box2:list):
    return ((box1[0]-box2[0])**2 + (box1[1]-box2[1])**2 )**0.5


class Map_node(object):
    def __init__(self,loc:list,parent=None,H = 0,G = 0,D = 0):
        self.loc = loc
        self.H = H
        self.children = []
        self.parent = parent
        self.G = G
        self.D = D
        
class SearchTree(object):     
    def __init__(self,map_file:np.matrix,map_img:Image.Image,start_loc,target_loc,factor = (2,0,1,0),save_pic = True,Max_Step = 100000,Max_Try = 10):
        self.map = map_file
        self.closed = np.matrix(np.zeros(map_file.shape))
        self.save_pic = save_pic
        self.head = Map_node(start_loc)
        self.target = target_loc
        self.path = [self.head]
        self.openlist = [self.head]
        self.G_factor = factor[0]
        self.H_factor = factor[1]
        self.D_factor = factor[2]
        self.D_init  = factor[3]
        self.map_img = map_img
        self.Max_Step = Max_Step
        self.Max_Try = Max_Try
    def find_children_loc(self,loc:list):
        actions = [[1,0],[1,1],[-1,0],[-1,-1],[-1,1],[1,-1],[0,1],[0,-1]]
        children = []
        for action in actions:
            if loc[0]+action[0] < self.map.shape[0] and loc[0]+action[0] >= 0:
                if loc[1]+action[1] < self.map.shape[1] and loc[1]+action[1] >= 0:
                    if self.map[loc[0]+action[0],loc[1]+action[1]] > 0 and self.closed[loc[0]+action[0],loc[1]+action[1]] != 1:
                        children.append([loc[0]+action[0],loc[1]+action[1]])
        return children                
    def add_to_openlist(self,p_node:Map_node):
        loc = p_node.loc
        actions = [[1,0],[1,1],[-1,0],[-1,-1],[-1,1],[1,-1],[0,1],[0,-1]]
        children = []
        for action in actions:
            if loc[0]+action[0] < self.map.shape[0] and loc[0]+action[0] >= 0:
                if loc[1]+action[1] < self.map.shape[1] and loc[1]+action[1] >= 0:
                    if self.map[loc[0]+action[0],loc[1]+action[1]] > 0 and self.closed[loc[0]+action[0],loc[1]+action[1]] == 0:
                        children.append([loc[0]+action[0],loc[1]+action[1]]) 
                        
        for child_loc in children:
            H = distance(child_loc,self.target)
            node = Map_node(child_loc,p_node,H,p_node.G + 1,self.cal_D(child_loc))
            p_node.children.append(node)
            self.openlist.append(node)
    def cal_D(self,loc):
        i = 0
        while(1):
            i = i+1
            if loc[0]-i < 0 or loc[0] + i >= self.map.shape[0] or loc[1] - i < 0 or loc[1] + i >= self.map.shape[1]:
                D = i
                break
            if not self.map[loc[0]-i:loc[0]+i,loc[1]-i:loc[1]+i].all(): 
                D = i
                break
        return (self.D_init-D)
    
    def find_maxA(self):
        open = self.openlist
        maxA = np.inf
        max_index = 0
        for index in range(open.__len__()):
            node = open[index]
            A = self.D_factor*node.D + self.H_factor*node.H + self.G_factor*node.G
            
            if A < maxA and self.closed[open[index].loc[0],open[index].loc[1]] == 0:
                maxA = A
                max_index = index
        return max_index
        
    def search(self):

        step  = 0
        while(True):
            index = self.find_maxA()
            current_node  = self.openlist.pop(index)
            self.closed[current_node.loc[0],current_node.loc[1]] = 1
            if current_node.loc == self.target:
                self.path = self.back_propagation(current_node)
                print("----Path found with A*----")
                if self.save_pic:
                    pgm = self.map_img.copy()
                    for path_node in self.path:
                        if path_node[0] - 34 < pgm.size[1] and path_node[1] < pgm.size[0] and path_node[0]>34:
                            pgm.putpixel((path_node[1],path_node[0]-34),128)
                    save_path = join(base_path, 'worlds/BARN/A_star_map', world_name)
                    pgm.save(save_path)
                self.path.reverse()
                return self.path
            else:
                self.add_to_openlist(current_node)
                step = step + 1
            if step > self.Max_Step:
                print("Failed! update param")
                break
          # 返回从头到尾的路径坐标 地图像素坐标
        return []
    def Update_factor(self,step = 0.1):
        self.G_factor = self.G_factor + step
    def back_propagation(self,target_node:Map_node):
        node = target_node
        path = []
        while node.parent is not None:
            path.append(node.loc)
            node = node.parent
        return path


from scipy.spatial.transform import Rotation as R


def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('zyx', degrees=True)
    return euler  

def img_dilate(bin_im, kernel,center_coo = [1,1]):
    
    kernel_w = kernel.shape[0]
    kernel_h = kernel.shape[1]
    if kernel[center_coo[0], center_coo[1]] == 0:
        raise ValueError("指定原点不在结构元素内！")
    dilate_img = np.zeros(shape=bin_im.shape)
    for i in range(center_coo[0], bin_im.shape[0] - kernel_w + center_coo[0] + 1):
        for j in range(center_coo[1], bin_im.shape[1] - kernel_h + center_coo[1] + 1):
            a = bin_im[i - center_coo[0]:i - center_coo[0] + kernel_w,
                j - center_coo[1]:j - center_coo[1] + kernel_h]
            dilate_img[i, j] = min(np.max(a * kernel),1)  # 若“有重合”，则点乘后最大值为0
    return dilate_img

def angle_process(ang):
    """
    将角度换算到-pi度到+pi度
    """
    if ang > math.pi:
        ang -= math.pi*2
    if ang < -math.pi:
        ang += math.pi*2
    return ang

def generate_twist(path, pos_x, pos_y, heading):
    """
    优化反馈控制，生成vx、vw控制率，按照上课PPT上来的，公式和参数详细解释看PPT
    以path列表中的第一个点为规划目标，达到目标后删除第一个点
    return: [v, w]
    """
    # ----参数----
    vxmax = 2  # 最大线速度，完全不知道，编的
    vwmax = 4  # 最大角速度，完全不知道，编的
    dist_arrive = 0.5  # 距离多少米算到达了该点，太小车扭屁股，太大会删掉狭缝里的路径点
    k1 = 1  # k1是beta的系数，大k1小车会更多的考虑到达目标点时候朝向下一个点，会将直线走成弧线
    k2 = 5  # k2是kappa的整体增益，增大k2可以使路径更贴合连线，减小转弯半径，加快收敛
    # k3、k4用作从kappa生成v，增大k3、k4转弯半径减小，k3效果更显著
    k3 = 3  # 用作从kapa生成vx，狠狠增大k3可以让小车慢下来蠕动，几乎就是完全跟随路径
    k4 = 1.2

    # ----初始化----
    # 空路径不处理
    if len(path) <= 0:
        return [0, 0]

    # 到达目标点则删除之
    p = math.hypot(path[0][1]-pos_y, path[0][0]-pos_x)  # 计算距离
    if p < dist_arrive:  # 如果接近，就把点删了
        del path[0]
        return [0, 0]
    
    # 为目标点添加heading
    path_heading = math.atan2(path[0][1]-pos_y, path[0][0]-pos_x)  # 机器人与目标点连线角度
    if len(path) <= 1:  # 如果只剩一个点
        obj_heading = path_heading  # 目标点heading设置为机器人与目标点连线
    else:  # 如果还有一坨点
        # obj_heading = path_heading
        obj_heading = math.atan2(path[1][1]-path[0][1], path[1][0]-path[0][0])  # 目标点heading指向路路径下一点

    # ----反馈控制----
    a = angle_process(path_heading-heading)  # ∠α，定义为路径方位角与机器人当前方位角的差
    b = angle_process(path_heading-obj_heading)  # ∠β，定义为当前路径方位角与下一路径方位角的差
    kapa = (k2*(a-math.atan(-k1*b))+(1+k1/(1+(k1*b)**2))*math.sin(a))/p  # 计算κ
    vx = vxmax/(1+k3*(math.fabs(kapa)**k4))  # 生成控制率
    vw = vx*kapa
    if abs(a) > 60:  # 如果机器人当前方位角与路径方位角相差过大（>60°），则原地转圈，避免碰撞
        vx = 0
        vw = vw/vw*vwmax
    print("-----------------------------------------------------")
    print("obj_x=", path[0][0], "obj_y=", path[0][1])
    print("cur_x=", pos_x, "cur_y=", pos_y)
    print("vx=", vx, "vw=", vw)
    return vx, vw  # TODO:对vw输出进行限幅，但不限也没事



    


world_name = ""  # 全局，供保存路径图使用

if __name__ == '__main__':

    rospy.init_node('A_star', anonymous=True)
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')  # 地图文件存放的基本路径


    # ---------------------选择地图进行测试-------------------------------#
    world_idx = rospy.get_param('world_num')  # 获取roscore地图id
    # world_idx = 5  # 手动设置地图id

    

    world_name = "map_pgm_%d.pgm" %(world_idx)
    init_coor = [INIT_POSITION[0], INIT_POSITION[1]]  # 物理坐标，初始点-2，3
    goal_coor =  [GOAL_POSITION[0], GOAL_POSITION[1]]  # 物理坐标，目标点-2，13
    im = Image.open(join(base_path, 'worlds/BARN/map_files', world_name))  # 读占用栅格地图
    
    
    kernel = np.ones((3, 3))    
    pixels = np.concatenate((255*np.ones((34,30)),np.matrix(im)))/255  # 好像是在扩充完整地图，扩充到100*30，地图坐标范围
    #print(1-pixels)
    #pixels = img_dilate(np.array(1-pixels),kernel)
    #print(pixels)
    ST = SearchTree(pixels,im,world_to_map(init_coor),world_to_map(goal_coor))  # 初始化搜索树
    path_map_coor = ST.search()  # 使用A*搜索路径
    print(path_map_coor)
    path_phy_coor = [map_to_world(i) for i in path_map_coor]

    # exit(0)

    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    # hight 66 width 30 AKA 66*30
    # print(world_to_map(init_coor), world_to_map(goal_coor))  # 将物理坐标映射到地图上，地图上起点79,16; 终点13,16
    # 初始点-2 3，终点-2 13，物理坐标
    # TODO: 地图坐标和物理坐标的转换有误差，比如起点物理坐标是-2，3，映射到地图上是79，16，但再映射回来就变成-1.95，3了。
    # 不是双射做不到！就这样了

    while(1):
        curr_time = rospy.get_time()
        
        pos = gazebo_sim.get_model_state().pose.position
        pose_quat = gazebo_sim.get_model_state().pose.orientation
        pose = quaternion2euler([pose_quat.x,pose_quat.y,pose_quat.z,pose_quat.w])  # Euler ZYX
        heading = pose[0]  # in degree, starting at +90

        twist = generate_twist(path_phy_coor, pos.x, pos.y, heading/180*math.pi)  # v w
        gazebo_sim.pub_cmd_vel(twist)  



        curr_time_0 = rospy.get_time()
        if curr_time_0 - curr_time < 0.1:
            time.sleep(0.1 - (curr_time_0 - curr_time))