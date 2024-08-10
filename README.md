# Frontier of Aeronautical

## Overview

    “空天先锋”海上垂起固定翼海上救援方案智能算法模块

    Author: Wang Qi , Chen Yu An , Qi Chen , Chen Hong Yu , Yang Wen Di

    运行环境：Ubuntu 20.04 / ROS1 Noetic / python-3.x (x>=7)

### Dependency and Files

1：按照赛事手册完成相关环境的配置

2：本项目使用了YOLOv8，需要进行相关依赖配置

    pip install ultralytics

3：依赖安装好后，解压压缩包，文件夹结构如下

    UAV_Algorithm_Moudle---src---angle_tracker
                        |     |
                        |     ---score_module
                        |     |
                        |     ---ship_detector
                        |     |
                        |     ---ship_tracker
                        |     |
                        |     ---vtol_navigation
                        |     |
                        |     ---Yolov8_ros
                        |
                        ---one_step_to_start.sh
                        |
                        ---README.md        

### Building

    cd UAV_Algorithm_Moudle
    catkin_make -j 

### Runing
    根据赛题文件启动仿真，通信，雷暴，求救信号，位置正值程序文件以及地面站后：

    cd UAV_Algorithm_Moudle
    sudo chmod a+x one_step_to_start.sh //提供了一键启动脚本
    ./one_step_to_start.sh

    若shell文件启动失败，可手动启动
     Bash1:
     cd UAV_Algorithm_Moudle/src/vtol_navigation
     python3 vtol_navigation_1.py

     Bash2:
     cd src/vtol_navigation
     python3 vtol_navigation_2.py

     Bash3:
     cd UAV_Algorithm_Moudle
     . devel/setup.bash
     roslaunch ship_detector ship_detectors.launch

### Pakages

- [ship_detector] 
    订阅图像流进行船体所载特征区域的颜色识别并计算识别区域质心，发布各识别对象的像素中心
- [ship_tracker]
    订阅识别节点发布的像素中心坐标，发布速度控制指令，跟踪像素中心并完成降落任务
- [vtol_navigation]
    订阅雷暴区域中心点，生成二维路径,并发布航点完成导航任务
- [angle_tracker]
    订阅求救信号角度信息，进行卡尔曼滤波等操作并发布目标航点
- [Yolov8_ros]
    包含yolov8识别算法以及追踪器模块
- [score_module]
    包含一个自设裁判系统用于测试

### Addition

    本项目由重庆大学“空天先锋”团队开发，若依赖配置或使用过程有疑问或者报错，欢迎联系团队负责人王琦 QQ：2100855823 Tel：15755690788，e-mail:miaronsa@163.com
    如能在使用过程提供意见与反馈，不胜感激








