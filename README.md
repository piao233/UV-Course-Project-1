# 开发记录
## 1207
+ 所有的路径全部改成相对路径，换机器后需要删除build重新catkin_make；
+ 在ROScore里增加了map_num参数并在run.py中赋值，方便其他地方调用，删除了所有的固定地图调用（如代码里绝对路径找map5）；
+ A*生成的路径图保存到一个新文件夹：.../BARN/A_star_map；
+ 运行前请source。~~在run.py里面source绝对路径之后别人还用不用了！！愤怒！！自己写在~/.bashrc里面去！！！~~
##  1215
+ run.py中增加 is_debug 选项，True时需要手动执行A_star.py并禁用超时
+ 依旧在研究物理坐标系和地图坐标系的区别
##  1216 zhy
+ 坐标系转换不是双射肯定丢失精度，别改了
+ A_star现在加上改参数的接口了，详细见Search_Tree 的__init__()
+ 修复一些bug，改进算法提高成功率
##  1219 zhy
+ 禁止生气，给你删了 :smile:
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
