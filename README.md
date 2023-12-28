# 开发记录

## 1228 zyp
+ 调了一晚上参，现在的参数很保守，点密集的话走的特别慢（一般点密集都是在小巷里嘛）
+ 降采样做得好啊！！夸赞！好的很！在A_star.py L380-385可以选择用哪根路径跑
+ 助教PPT上放了world 0，99，199三个必测的，还说要抽三个，所以99一定得过
+ 99的问题在44上更明显，A*的路径是贴着墙的，还需要你再调整一下
+ 但说实话，我总觉得仿真环境有丢包，因为每次仿出来都不太一样，卡顿的时候直接飞速撞墙，不卡的时候走的挺好的，你要是在虚拟机上跑说不定更卡
+ 举一个丢包的例子，如果出现过冲，我写的控制是倒车+全速转弯，但有时候只转弯不倒车，有时候只倒车不转弯，丢包，但只丢了一半。如果你到小车倒车但没有转弯，那就是丢了。倒车的时候一定搭配着全速旋转的。
+ World 0 1 3 4 5 可以以deeper降采样路径通过
+ World 2 可以以deep降采样路径通过
+ world 2 44 99 压着原路径也通不过
+ 一个小疑问，一直是右前轮撞车，有没有可能是在两个坐标系做映射的时候...障碍物都往左上偏移了...？或者从像素坐标转换到物理坐标的时候给的是像素右下角的点？




## 1227 zhy
+ 改了一下A*路径
+ A*路径很简单的，而且其实这玩意不判断是否撞上（

## 1226  zyp
+ 嗨嗨嗨  小车会动啦
+ 用的是PPT里的改进反馈控制法，有四个参数，写在函数最开头了
+ 函数本身没什么bug，因为抄的本科的
+ 关于A*搜索出来的路径太过细碎的问题，应该把无碰的几段连成一段，由于我不太清楚怎么判断碰撞所以只能留给你了hhh
+ 现在去run.py的话小车就可以跑得动了，而且我现在调的很保守（跑的比较慢），你去跑一轮就知道我说的扭屁股是啥意思了
+ run.py L108里有一个debug模式，开启debug时不自动启动navigation stack，并禁用超时，所以可以手动运行A_star.py然后一步一步慢慢调
+ A_star.py L242可以选择使用什么地图，如果要配合run.py使用的话记得改掉

## 1219 zhy

+ 禁止生气，给你删了:smile:

## 1216 zhy

+ 坐标系转换不是双射肯定丢失精度，别改了
+ A_star现在加上改参数的接口了，详细见Search_Tree 的__init__()
+ 修复一些bug，改进算法提高成功率

## 1215

+ run.py中增加 is_debug 选项，True时需要手动执行A_star.py并禁用超时
+ 依旧在研究物理坐标系和地图坐标系的区别

## 1207

+ 所有的路径全部改成相对路径，换机器后需要删除build重新catkin_make；
+ 在ROScore里增加了map_num参数并在run.py中赋值，方便其他地方调用，删除了所有的固定地图调用（如代码里绝对路径找map5）；
+ A*生成的路径图保存到一个新文件夹：.../BARN/A_star_map；
+ 运行前请source。~~在run.py里面source绝对路径之后别人还用不用了！！愤怒！！自己写在~/.bashrc里面去！！！~~


# autonomous_navigation_project_2022w

BARN Challenge in ICRA 2022 as course project

## TODO

implement your own navigation algorithms in run.py

## Run Simulations

```
source devel/setup.sh
python3 run.py --gui --world_idx xxx
```

# Acknowledgements

Code references

[ros_jackal](https://github.com/Daffan/ros_jackal).

[nav-competition-icra2022](https://github.com/Daffan/nav-competition-icra2022).
