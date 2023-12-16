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
    return 1.5*((box1[0]-box2[0])**2 + (box1[1]-box2[1])**2 )**0.5
    #return ((box1[0]-box2[0])**2)**0.5 + ((box1[1]-box2[1])**2)**0.5

class Map_node(object):
    def __init__(self,loc:list,parent=None,H = 0,G = 0,D = 0):
        self.loc = loc
        self.H = H
        self.children = []
        self.parent = parent
        self.G = G
        self.D = D
        self.A = self.G + self.D + self.H
        
class SearchTree(object):     
    def __init__(self,map_file:np.matrix,map_img:Image.Image,start_loc,target_loc):
        self.map = map_file
        self.closed = np.matrix(np.zeros(map_file.shape))
        #print(type(self.closed))
        self.head = Map_node(start_loc)
        self.target = target_loc
        self.path_coor = [start_loc]  # 只记录坐标
        self.openlist = [self.head]
        self.D_factor = 0
        self.D_factor1 = 2
        self.map_img = map_img
    def find_children_loc(self,loc:list):
        actions = [[1,0],[1,1],[-1,0],[-1,-1],[-1,1],[1,-1],[0,1],[0,-1]]
        children = []
        for action in actions:
            if loc[0]+action[0] < self.map.shape[0] and loc[0]+action[0] >= 0:
                if loc[1]+action[1] < self.map.shape[1] and loc[1]+action[1] >= 0:
                    if self.map[loc[0]+action[0],loc[1]+action[1]] == 255 and self.closed[loc[0]+action[0],loc[1]+action[1]] !=1:
                        children.append([loc[0]+action[0],loc[1]+action[1]])
        return children                
    def add_to_openlist(self,p_node:Map_node):
        loc = p_node.loc
        actions = [[1,0],[1,1],[-1,0],[-1,-1],[-1,1],[1,-1],[0,1],[0,-1]]
        children = []
        for action in actions:
            if loc[0]+action[0] < self.map.shape[0] and loc[0]+action[0] >= 0:
                if loc[1]+action[1] < self.map.shape[1] and loc[1]+action[1] >= 0:
                    if self.map[loc[0]+action[0],loc[1]+action[1]] == 255 and self.closed[loc[0]+action[0],loc[1]+action[1]] == 0:
                        #print(self.closed[loc[0]+action[0],loc[1]+action[1]])
                        children.append([loc[0]+action[0],loc[1]+action[1]]) 
                        
        for child_loc in children:
            H = distance(child_loc,self.target)
            node = Map_node(child_loc,p_node,H,p_node.G + 1,self.cal_D(child_loc))
            p_node.children.append(node)
            self.openlist.append(node)
        #for child in self.openlist:
            #print(self.map[child.loc])
    def cal_D(self,loc):
        i = 0
        while(1):
            i = i+1
            if loc[0]-i < 0 or loc[0] + i >= self.map.shape[0] or loc[1] - i < 0 or loc[1] + i >= self.map.shape[1]:
                D = i
                break
            #rint(self.map[loc[0]-i:loc[0]+i,loc[1]-i:loc[1]+i])
            if not self.map[loc[0]-i:loc[0]+i,loc[1]-i:loc[1]+i].all(): 
                D = i
                break
        return self.D_factor1*(self.D_factor-D)
    
    def find_maxA(self):
        open = self.openlist
        maxA = np.inf
        max_index = 0
        for index in range(open.__len__()):
            node = open[index]
            if node.A < maxA:
                maxA = node.A
                max_index = index
        #print(maxA)
        return max_index
    
    def search(self):
        while(1):
            index = self.find_maxA()
            current_node  = self.openlist.pop(index)
           #print(current_node.loc)
            self.closed[current_node.loc[0],current_node.loc[1]] = 1
            if current_node.loc == self.target:
                path = self.back_propagation(current_node)  # 从尾到头
                pgm = self.map_img.copy()
                for path_node in reversed(path):
                    print(path_node.loc)
                    self.path_coor.append([path_node.loc[0], path_node.loc[1]])
                    if path_node.loc[0] - 34 < pgm.size[1] and path_node.loc[1] < pgm.size[0] and path_node.loc[0]>34:
                        pgm.putpixel((path_node.loc[1],path_node.loc[0]-34),128)
                save_path = join(base_path, 'worlds/BARN/A_star_map', world_name)
                pgm.save(save_path)
                break
            else:
                self.add_to_openlist(current_node)

        return self.path_coor  # 返回从头到尾的路径坐标 地图像素坐标

    def back_propagation(self,target_node:Map_node):
        node = target_node
        path = []
        while node.parent is not None:
            path.append(node)
            node = node.parent
        return path


from scipy.spatial.transform import Rotation as R

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('zyx', degrees=True)
    return euler  


# TODO
def generate_twist():
    # vel = ((goal_coor[0] - pos.x)**2+(goal_coor[1] - pos.y)**2)**0.3
    return [0,1]


world_name = ""  # 全局，供保存路径图使用

if __name__ == '__main__':

    rospy.init_node('A_star', anonymous=True)
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')  # 地图文件存放的基本路径


    # ---------------------选择地图进行测试-------------------------------#
    # world_idx = rospy.get_param('world_num')  # 获取roscore地图id
    world_idx = 5  # 手动设置地图id



    world_name = "map_pgm_%d.pgm" %(world_idx)

    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    init_coor = [INIT_POSITION[0], INIT_POSITION[1]]  # 物理坐标，初始点-2，3
    goal_coor =  [GOAL_POSITION[0], GOAL_POSITION[1]]  # 物理坐标，目标点-2，13
    im = Image.open(join(base_path, 'worlds/BARN/map_files', world_name))  # 读占用栅格地图
    #hight 66 width 30 AKA 66*30

    pixels = np.concatenate((255*np.ones((34,30)),np.matrix(im)))  # 好像是在扩充完整地图，扩充到100*30，地图坐标范围
    
    
    #print(world_to_map([-0.15,0]))
    #print(world_to_map(GOAL_POSITION))
    #print(map_to_world(world_to_map(GOAL_POSITION)))
    #print(map_to_world(world_to_map(INIT_POSITION)))
    #print(pixels[99,29])
    #while(1):
        #curr_time = rospy.get_time()
        #gazebo_sim.pub_cmd_vel([1, 0])
        #pos = gazebo_sim.get_model_state().pose.position
        #print(pos.x)
        #curr_time_0 = rospy.get_time()
        #if curr_time_0 - curr_time < 0.1:
            #time.sleep(0.1 - (curr_time_0 - curr_time))


    print(world_to_map(init_coor),world_to_map(goal_coor))  # 将物理坐标映射到地图上，地图上起点79,16; 终点13,16

    ST = SearchTree(pixels,im,world_to_map(init_coor),world_to_map(goal_coor))  # 初始化搜索树
    path_map_coor = ST.search()  # 使用A*搜索路径
    path_phy_coor = [ map_to_world(i) for i in path_map_coor]
    # 初始点-2 3，终点-2 13，物理坐标
    # TODO: 地图坐标和物理坐标的转换有误差，比如起点物理坐标是-2，3，映射到地图上是79，16，但再映射回来就变成-1.95，3了。

    while(1):
        curr_time = rospy.get_time()
        
        pos = gazebo_sim.get_model_state().pose.position
        pose = gazebo_sim.get_model_state().pose.orientation
        euler = quaternion2euler([pose.x,pose.y,pose.z,pose.w])

        # print(pose.x)
        # print(euler) 

        twist = generate_twist()
        gazebo_sim.pub_cmd_vel(twist)  # v w



        curr_time_0 = rospy.get_time()
        if curr_time_0 - curr_time < 0.1:
            time.sleep(0.1 - (curr_time_0 - curr_time))