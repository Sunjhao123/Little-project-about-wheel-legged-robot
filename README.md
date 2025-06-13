# 研究生项目 ROS四足轮腿机器人
研究生项目 проект ВКРМ：колесно-шагающий комбинированный робот

Ubuntu系统是20.04，ROS版本是noetic。
Система: ROS noetic
用廖佬的legged_control项目魔改的，模型是Unitree的B2W四轮足机器人，没做啥复杂的东西，轮式部分用的还是ROS官方的插件整的。
Этот проект построен на основе legged_control.
建立工作空间，假设是leg_ws，编译建议逐个包catkin build XXX
Создать пространство работы имена leg_ws, и построить каждую пакету по catkin build XXX.
打开终端运行程序时记得：source ~/leg_ws/devel/setup.bash

打开仿真环境：roslaunch legged_unitree_description empty_world_b2.launch

启动控制程序：roslaunch legged_controllers load_controller_b2.launch cheater:=false

我整了个总的启动脚本：roslaunch control_demo start_world_b2.launch 记得先编译一下control_demo这个包。
Завершить проект по: roslaunch control_demo start_world_b2.launch
roslaunch control_demo start_world_b2_factory.launch （这个启动的gazebo环境是我自己做的一个简单的工厂环境）

建图построение карты： roslaunch nav_demo mapping.launch

导航планирование пути： roslaunch nav_demo navigation.launch

导航算法是我整的一个改进A*算法，记得按照自定义路径规划插件的方法进行注册（具体步骤我忘了）。也可以换自己的算法。

我给机器人整了个模拟接收温湿度传感器和噪音传感器节点信息，生成映射地图的功能，需要在fac_sensor文件夹下启动：
打开终端运行程序时记得：source ~/leg_ws/devel/setup.bash

python3  temperature_publisher.py

python3  area_temperature_mapper.py

python3 humidity_publisher.py

python3 area_humidity_mapper.py

python3 noise_publisher.py

python3 area_noise_mapper.py

python3 spectrum_visualizer.py

机器人的运动控制用rqt
управлять движением робота с помощью rqt
控制足式运动就在总的启动脚本的控制终端输入 trot，另开一个终端运行rqt，用cmd_vel2做控制。

控制轮式运动就在总的启动脚本的控制终端输入 stance，另开一个终端运行rqt，用cmd_vel做控制，但是不能转弯。

控制组合运动就在总的启动脚本的控制终端输入 trot，另开一个终端运行rqt，用cmd_vel做控制。

我整的挺乱的，其他的launch文件都是错的，要是能帮我改改也行。本来想在机器人上整个机械臂，看了qm_control的代码没看懂，不大会整，就另整了个单独的工作空间，分成两个系统了。

参考项目：
https://github.com/skywoodsz/qm_control

https://github.com/skywoodsz/qm_control

https://github.com/danisotelo/qm_door
