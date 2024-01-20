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
import matplotlib.pyplot as plt
map_offset = -0.15
#地图映射偏置，现在默认-0.5可以跑通2，越大越容易右轮撞墙，越小越容易左轮撞墙
def world_to_map(world_coor:list):
    map_coor = [0,0]
    map_coor[0] = 99 - int(world_coor[1]/0.15) 
    map_coor[1] = 29 + int(world_coor[0]/0.15) 
    return map_coor

def map_to_world(map_coor:list):
    world_coor = [0,0]
    world_coor[1] = (99 - map_coor[0])*0.15 - map_offset
    world_coor[0] = (map_coor[1] - 29)*0.15 + map_offset
    return world_coor

def distance(box1:list,box2:list):
    return ((box1[0]-box2[0])**2 + (box1[1]-box2[1])**2 )**0.5
def average (box1:list,box2:list):
    return [(box1[0]+box2[0])/2,(box1[1]+box2[1])/2]

class Map_node(object):
    def __init__(self,loc:list,parent=None,H = 0,G = 0,D = 0):
        self.loc = loc
        self.H = H
        self.children = []
        self.parent = parent
        self.G = G
        self.D = D
        
class SearchTree(object):  
    """
    计算改进A*的对象，参数如名字所示
    param
    D 到最近障碍物参数(越近越大) = D_init - Distance(nearest obstacles)
    G 步数
    H 到终点距离
    factor 为系数，分别为 G_factor,H_factor,D_factor,D_init
    A = D_factor*D + H_factor*H + G_factor*G
    """   
    def __init__(self,map_file:np.matrix,map_img:Image.Image,start_loc,target_loc,factor = (2,0,1,0),save_pic = True,Max_Step = 100000,Max_Try = 10):
        self.map = map_file
        self.closed = np.matrix(np.zeros(map_file.shape))
        self.save_pic = save_pic
        self.head = Map_node(start_loc)
        self.target = target_loc
        self.path = []
        self.openlist = [self.head]
        self.G_factor = factor[0]
        self.H_factor = factor[1]
        self.D_factor = factor[2]
        self.D_init  = factor[3]
        self.map_img = map_img
        self.Max_Step = Max_Step
        self.Max_Try = Max_Try
        #我 们 仨
        self.downsamplepath = []#删除直线上的点
        self.deepdownsamplepath = []#删除非碰撞点
        self.deeperdownsamplepath = []#合并太近的点
        self.path_node = []
        self.max_D = -np.inf #内部参数别管
        self.collision_factor = 0 #判断碰撞裕度，要是觉得扭屁股就调高点，可能导致撞墙，调低避免撞墙
        self.merge_limit = 4 #第三步合并点距离判断
    def find_children_loc(self,loc:list):
        """
        拓展节点
        """
        actions = [[1,0],[1,1],[-1,0],[-1,-1],[-1,1],[1,-1],[0,1],[0,-1]]
        children = []
        for action in actions:
            if loc[0]+action[0] < self.map.shape[0] and loc[0]+action[0] >= 0:
                if loc[1]+action[1] < self.map.shape[1] and loc[1]+action[1] >= 0:
                    if self.map[loc[0]+action[0],loc[1]+action[1]] > 0 and self.closed[loc[0]+action[0],loc[1]+action[1]] != 1:
                        children.append([loc[0]+action[0],loc[1]+action[1]])
        return children                
    def add_to_openlist(self,p_node:Map_node):
        """
        添加到openlist
        """
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
        """
        计算D
        """
        i = 0
        while(1):
            i = i+1
            if loc[0]-i < 0 or loc[0] + i >= self.map.shape[0] or loc[1] - i < 0 or loc[1] + i >= self.map.shape[1]:
                D = i
                break
            if not self.map[loc[0]-i:loc[0]+i,loc[1]-i:loc[1]+i].all(): 
                D = i
                break
        #return (-math.log(self.D_init*D)) 
        return self.D_init - D
    def find_maxA(self):
        """
        pop openlist中A最小的节点
        排除坐标重复但参数不重复的项
        """
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
        """搜索进程"""
        step  = 0
        while(True):
            index = self.find_maxA()
            current_node  = self.openlist.pop(index)
            self.closed[current_node.loc[0],current_node.loc[1]] = 1
            if current_node.loc == self.target:
                self.path = self.back_propagation(current_node)
                self.path.reverse()
                self.delete_points()
                self.delete_points_nocollision()
                self.deep_delete_points()
                print("----Path found with A*----")
                if self.save_pic:
                    pgm = self.map_img.copy()
                    for path_node in self.path:
                        if path_node[0] - 34 < pgm.size[1] and path_node[1] < pgm.size[0] and path_node[0] > 34:
                            pgm.putpixel((path_node[1],path_node[0]-34),200)
                    for path_node in self.downsamplepath:
                        if path_node[0] - 34 < pgm.size[1] and path_node[1] < pgm.size[0] and path_node[0] > 34:
                            pgm.putpixel((path_node[1],path_node[0]-34),128)
                    for path_node in self.deepdownsamplepath:
                        if path_node[0] - 34 < pgm.size[1] and path_node[1] < pgm.size[0] and path_node[0] > 34:
                            pgm.putpixel((path_node[1],path_node[0]-34),64)       
                    for path_node in self.deeperdownsamplepath:
                        if path_node[0] - 34 < pgm.size[1] and path_node[1] < pgm.size[0] and path_node[0] > 34:
                            pgm.putpixel((int(path_node[1]),int(path_node[0])-34),256)
                    save_path = join(base_path, 'worlds/BARN/A_star_map', world_name)
                    pgm.save(save_path)
                
                return self.path
            else:
                self.add_to_openlist(current_node)
                step = step + 1
            if step > self.Max_Step:
                print("Failed! update param")
                break
          # 返回从头到尾的路径坐标 地图像素坐标
        return []
    def delete_points(self):
        """初步删除"""
        move = [[self.path[1][0] - self.path[0][0],self.path[1][1] - self.path[0][1]]]
        for i in range (1,self.path.__len__() - 1):
            move.append([self.path[i+1][0] - self.path[i][0],self.path[i+1][1] - self.path[i][1]])
            if move[i] != move[i-1]:
                self.downsamplepath.append(self.path[i])
        self.downsamplepath.append(self.target) 
    def delete_points_nocollision(self):
        """进一步删除"""
        self.deepdownsamplepath.append(self.head.loc)
        i = 0
        while self.deepdownsamplepath[-1] != self.downsamplepath[-1]:
            for j in range(i,self.downsamplepath.__len__()):
                #print(j,self.deepdownsamplepath[-1],self.downsamplepath[j],self.downsamplepath.__len__())
                if self.path_collision(self.deepdownsamplepath[-1],self.downsamplepath[j]):
                    if j != i:
                        self.deepdownsamplepath.append(self.downsamplepath[j-1])
                        i = j
                        break
                    else:
                        self.deepdownsamplepath.append(self.downsamplepath[j])
                        i = j + 1
                        break
            if j == self.downsamplepath.__len__()-1:
                self.deepdownsamplepath.append(self.downsamplepath[j])
    def deep_delete_points(self):
        """合并过近的点"""
        i = 0
        while i <= self.deepdownsamplepath.__len__()-2:  # 后面用到了i+1，最后手动补终点
            if distance(self.deepdownsamplepath[i],self.deepdownsamplepath[i+1]) < self.merge_limit:
                self.deeperdownsamplepath.append(average(self.deepdownsamplepath[i],self.deepdownsamplepath[i+1]))
                i += 1  # 跳一个点
            else:
                self.deeperdownsamplepath.append(self.deepdownsamplepath[i])
            i += 1
        self.deeperdownsamplepath.append(self.target)  # 保证终点在队列里
    
    def path_collision(self,loc1,loc2):
        """判断碰撞"""
        if loc1 == loc2:
            return False
        L = loc1[0] - loc2[0]
        H = loc1[1] - loc2[1]

        if abs(L) > abs(H):
            factor = H/L
            for i in range(abs(L)):
                loc = [loc2[0] + i,int(loc2[1] + factor*i)]
                D = self.cal_D(loc)
                if D >= self.max_D + self.collision_factor:
                    return True
        else:
            factor = L/H
            for i in range(abs(H)):
                loc = [int(loc2[0] + factor*i),loc2[1] + i]
                D = self.cal_D(loc)
                if D >= self.max_D + self.collision_factor:
                    return True
        return False
        
    def Update_factor(self,step = 0.1):
        """更新参数, aborted"""
        self.G_factor = self.G_factor + step
    def back_propagation(self,target_node:Map_node):
        """遍历搜索到的路径"""
        node = target_node
        while node.parent is not None:
            if node.D > self.max_D:
                self.max_D = node.D
            self.path.append(node.loc)
            self.path_node.append(node)
            node = node.parent
        #print("max_D")
        #print(self.max_D)
        return self.path


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
    vxmax = 1  # 最大线速度，完全不知道，编的。增大vmax需要减小k5。长直线后冲过端点需要减小vmxax
    vwmax = 2  # 最大角速度，完全不知道，编的
    dist_arrive = 0.2  # 距离多少米算到达了该点，太小车扭屁股，太大会删掉狭缝里的路径点
    k1 = 0.05 # k1是beta的系数，大k1小车会更多的考虑到达目标点时候朝向下一个点，会将直线走成弧线
    k2 = 4  # 主要影响kappa大小的系数，要求远大于1
    # k3、k4用作从kappa生成v，增大k3、k4转弯半径减小，k3效果更显著
    k3 = 4  # 用作从kapa生成vx，狠狠增大k3可以让小车在路径上整体减慢，几乎就是完全跟随路径
    k4 = 2.0  # k4在kappa的指数项上，当路径曲率变大的时候会猛增，增大k4有助于在末端减速，相对来讲直线部分不受影响，但过大会导致当小车没对准的时候压根开不动
    k5 = 3.0  # 倍增角速度控制量，发现仿真环境中角速度控制总是达不到理想值，k5过大可能直线上摇头晃脑，k5过小会导致过冲目标点（后原地掉头

    # ----初始化----
    # 空路径不处理
    if len(path) <= 0:
        return [0, 0]

    # 到达目标点则删除之
    p = math.hypot(path[0][1]-pos_y, path[0][0]-pos_x)  # 计算距离
    if p < dist_arrive and len(path) > 1:  # 如果接近，就把点删了，当然不许删最后一个点
        del path[0]
        # return [0, 0]
    
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
    kapa = (k2*(a-math.atan(-k1*b))+math.sin(a)*(1+k1/(1+(k1*b)*(k1*b))))/(p**2)  # 计算κ
    vx = vxmax/(1+k3*(math.fabs(kapa)**k4))  # 生成控制率
    vw = vx*kapa*k5
    if abs(a) > 15/180*math.pi:  # 如果机器人当前方位角与路径方位角相差过大（>15°），则倒车转圈，避免碰撞
        vx = -0.1*vx
        vw = a/abs(a)*vw/vw*vwmax  # 之前这里忘了加方向所以在转大圈圈
    print("-----------------------------------------------------")
    print("obj_x=%.4f, obj_y=%.4f" % (path[0][0], path[0][1]))
    print("cur_x=%.4f, cur_y=%.4f, heading=%.2f, target_heading=%.2f" % (pos_x, pos_y, heading/math.pi*180, obj_heading/math.pi*180))
    print("vx   =%.4f, vw   =%.4f" % (vx, vw))
    return vx, vw  # TODO:对vw输出进行限幅，但不限也没事



    


world_name = ""  # 全局，供保存路径图使用

if __name__ == '__main__':

    rospy.init_node('A_star', anonymous=True)
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')  # 地图文件存放的基本路径


    # ---------------------选择地图进行测试-------------------------------#
    world_idx = rospy.get_param('world_num')  # 获取roscore地图id
    #world_idx = 200 # 手动设置地图id

    

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
    print(ST.downsamplepath) #减少中间点的结果
    print(ST.deepdownsamplepath)#进一步减少中间点的结果
    print(ST.deeperdownsamplepath)

    # ------------------选用哪个路径----------------------------------------------
    # path_phy_coor = [map_to_world(i) for i in path_map_coor]
    # path_phy_coor = [map_to_world(i) for i in ST.downsamplepath]
    # path_phy_coor = [map_to_world(i) for i in ST.deepdownsamplepath]
    path_phy_coor = [map_to_world(i) for i in ST.deeperdownsamplepath]
    #----------------------------------------------------------------------------

    # 因后期由于物理地图和像素地图对不齐加了offset，起点也飞了，所以删掉起点
    del path_phy_coor[0]

    #print(ST.path_collision([68,21],[36,8]))
    #exit(0)

    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    # hight 66 width 30 AKA 66*30
    # print(world_to_map(init_coor), world_to_map(goal_coor))  # 将物理坐标映射到地图上，地图上起点79,16; 终点13,16
    # 初始点-2 3，终点-2 13，物理坐标
    # 地图坐标和物理坐标的转换有误差，比如起点物理坐标是-2，3，映射到地图上是79，16，但再映射回来就变成-1.95，3了。
    # 不是双射做不到！就这样了

    # 记录轨迹
    A_star_path = np.array(path_phy_coor).T.copy()
    pos_x = [-2, -2, -2, -2]
    pos_y = [3, 3, 3, 3]
    head = [90, 90, 90, 90]

    while(1):
        curr_time = rospy.get_time()
        
        pos = gazebo_sim.get_model_state().pose.position
        pose_quat = gazebo_sim.get_model_state().pose.orientation
        pose = quaternion2euler([pose_quat.x,pose_quat.y,pose_quat.z,pose_quat.w])  # Euler ZYX
        heading = pose[0]  # in degree, starting at +90

        twist = generate_twist(path_phy_coor, pos.x, pos.y, heading/180*math.pi)  # v w
        gazebo_sim.pub_cmd_vel(twist)  

        # 保存作图数据
        pos_x.append(pos.x)
        pos_y.append(pos.y)
        head.append(heading)  # in dgree
        # 设置退出
        if pos.x == pos_x[-4]:  # 有一阵子没动了
            break
        
        # 测试得单轮规划时间约在0.01s
        curr_time_0 = rospy.get_time()
        if curr_time_0 - curr_time < 0.02:
            time.sleep(0.1 - (curr_time_0 - curr_time))

    plt.plot(A_star_path[0,:], A_star_path[1,:], label='A_star_path')
    plt.scatter(A_star_path[0,:], A_star_path[1,:],)
    plt.plot(pos_x, pos_y, label='real_traj')
    
    plt.legend(loc='upper right')
    plt.show()