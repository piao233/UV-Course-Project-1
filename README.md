# 开发记录
## 1207
+ 所有的路径全部改成相对路径，换机器后需要删除build重新catkin_make；
+ 在ROScore里增加了map_num参数并在run.py中赋值，方便其他地方调用，删除了所有的固定地图调用（如代码里绝对路径找map5）；
+ A*生成的路径图保存到一个新文件夹：.../BARN/A_star_map；
+ 运行前请source。在run.py里面source绝对路径之后别人还用不用了！！愤怒！！自己写在~/.bashrc里面去！！！






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
