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
        self.path = [self.head]
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
                path = self.back_propagation(current_node)
                pgm = self.map_img.copy()
                for path_node in path:
                    print(path_node.loc)
                    if path_node.loc[0] - 34 < pgm.size[1] and path_node.loc[1] < pgm.size[0] and path_node.loc[0]>34:
                        pgm.putpixel((path_node.loc[1],path_node.loc[0]-34),128)
                pgm.save("/home/stalin/autonomous_navigation/src/jackal_helper/worlds/BARN/map_files/map_pgm_3_0.pgm")
                break
            else:
                self.add_to_openlist(current_node)
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
if __name__ == '__main__':
    
    rospy.init_node('A_star', anonymous=True)
    #joint1 = rospy.get_param("/world_name")
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    init_coor = [INIT_POSITION[0], INIT_POSITION[1]]
    goal_coor =  [GOAL_POSITION[0], GOAL_POSITION[1]]
    im = Image.open("/home/stalin/autonomous_navigation/src/jackal_helper/worlds/BARN/map_files/map_pgm_5.pgm")    # 读取文件\\\
    #30*66 0.0
    #print(data.shape)
    pixels = np.concatenate((255*np.ones((34,30)),np.matrix(im)))
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
    print(world_to_map(init_coor),world_to_map(goal_coor))
    #print(type(pixels))
    ST = SearchTree(pixels,im,world_to_map(init_coor),world_to_map(goal_coor))
    ST.search()
    while(1):
        curr_time = rospy.get_time()
        
        pos = gazebo_sim.get_model_state().pose.position
        pose = gazebo_sim.get_model_state().pose.orientation
        #print(pose.x)
        euler = quaternion2euler([pose.x,pose.y,pose.z,pose.w])
        print(euler)
        vel = ((goal_coor[0] - pos.x)**2+(goal_coor[1] - pos.y)**2)**0.3
        gazebo_sim.pub_cmd_vel([0, 1])#v w
        curr_time_0 = rospy.get_time()
        if curr_time_0 - curr_time < 0.1:
            time.sleep(0.1 - (curr_time_0 - curr_time))