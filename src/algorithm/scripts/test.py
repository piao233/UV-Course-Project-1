#!/usr/bin/python3
INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position
import time
import argparse
import subprocess
import os
from os.path import join
from PIL import Image
import numpy as np
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_simulation import GazeboSimulation

def read_img():
    im = Image.open("../../jackal_helper/worlds/BARN/map_files/map_pgm_0.pgm")    # 读取文件
    #im.show()    # 展示图片
    print(im.size)   # 输出图片大小
    pixels = np.matrix(im)
   
if __name__ == "__main__":
    read_img()     # 调用read_img()
    